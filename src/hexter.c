/* hexter DSSI software synthesizer plugin
 *
 * Copyright (C) 2004, 2009, 2011, 2012, 2014, 2018 Sean Bolton and others.
 *
 * Portions of this file may have come from Peter Hanappe's
 * Fluidsynth, copyright (C) 2003 Peter Hanappe and others.
 * Portions of this file may have come from Chris Cannam and Steve
 * Harris's public domain DSSI example code.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this program; if not, write to the Free
 * Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA 02110-1301 USA.
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>

#include <ladspa.h>
#include <alsa/seq_event.h>
#include <dssi.h>

#include "hexter_types.h"
#include "hexter.h"
#include "hexter_synth.h"
#include "dx7_voice.h"
#include "dx7_voice_data.h"

static LADSPA_Descriptor *hexter_LADSPA_descriptor = NULL;
static DSSI_Descriptor   *hexter_DSSI_descriptor = NULL;

static LADSPA_Data volume;

static int
dx7_patchbank_load_init(const char *filename, dx7_patch_t *firstpatch,
                  int maxpatches, char **errmsg);

static void
hexter_cleanup(LADSPA_Handle instance);

static void
hexter_run_synth(LADSPA_Handle instance, unsigned long sample_count,
                 snd_seq_event_t *events, unsigned long event_count);

/* ---- mutual exclusion ---- */

static inline int
dssp_voicelist_mutex_trylock(hexter_instance_t *instance)
{
    int rc;

    /* Attempt the mutex lock */
    rc = pthread_mutex_trylock(&instance->voicelist_mutex);
    if (rc) {
        instance->voicelist_mutex_grab_failed = 1;
        return rc;
    }
    /* Clean up if a previous mutex grab failed */
    if (instance->voicelist_mutex_grab_failed) {
        hexter_instance_all_voices_off(instance);
        instance->voicelist_mutex_grab_failed = 0;
    }
    return 0;
}

inline int
dssp_voicelist_mutex_lock(hexter_instance_t *instance)
{
    return pthread_mutex_lock(&instance->voicelist_mutex);
}

inline int
dssp_voicelist_mutex_unlock(hexter_instance_t *instance)
{
    return pthread_mutex_unlock(&instance->voicelist_mutex);
}

/* ---- LADSPA interface ---- */

/*
 * hexter_instantiate
 *
 * implements LADSPA (*instantiate)()
 */
static LADSPA_Handle
hexter_instantiate(const LADSPA_Descriptor *descriptor,
                   unsigned long sample_rate)
{
    hexter_instance_t *instance;
    int i;

    instance = (hexter_instance_t *)calloc(1, sizeof(hexter_instance_t));
    if (!instance) {
        return NULL;
    }

    /* do any per-instance one-time initialization here */
    for (i = 0; i < HEXTER_MAX_POLYPHONY; i++) {
        instance->voice[i] = dx7_voice_new();
        if (!instance->voice[i]) {
            DEBUG_MESSAGE(-1, " hexter_instantiate: out of memory!\n");
            hexter_cleanup(instance);
            return NULL;
        }
    }
    if (!(instance->patches = (dx7_patch_t *)malloc(128 * DX7_VOICE_SIZE_PACKED))) {
        DEBUG_MESSAGE(-1, " hexter_instantiate: out of memory!\n");
        hexter_cleanup(instance);
        return NULL;
    }

    instance->sample_rate = (float)sample_rate;
    instance->nugget_remains = 0;
    dx7_eg_init_constants(instance);  /* depends on sample rate */

    instance->note_id = 0;
    instance->polyphony = HEXTER_DEFAULT_POLYPHONY;
    instance->monophonic = DSSP_MONO_MODE_OFF;
    instance->max_voices = instance->polyphony;
    instance->current_voices = 0;
    instance->last_key = 0;
    pthread_mutex_init(&instance->voicelist_mutex, NULL);
    instance->voicelist_mutex_grab_failed = 0;
    pthread_mutex_init(&instance->patches_mutex, NULL);
    instance->pending_program_change = -1;
    instance->current_program = 0;
    instance->overlay_program = -1;
    hexter_data_performance_init(instance->performance_buffer);

    // Load default patches
    const char* default_bank_path = getenv("HEXTER_DEFAULT_BANK_PATH");
    if (default_bank_path) {
		dx7_patchbank_load_init(default_bank_path, instance->patches,
								128, NULL);
		printf("Loaded bank: %s\n", default_bank_path);
	} else {
		hexter_data_patches_init(instance->patches);
		printf("Set HEXTER_DEFAULT_BANK_PATH to change the bank\n");
    }

    hexter_instance_select_program(instance, 0, 0);
    hexter_instance_init_controls(instance);

	// Read external volume
	const char* volume_var = getenv("HEXTER_VOLUME");
	if (volume_var) {
		volume = (LADSPA_Data) atof(volume_var);
		printf("Volume: %f\n", volume);
	} else {
		printf("Set HEXTER_VOLUME to change the gain\n");
	}

    return (LADSPA_Handle)instance;
}

/*
 * hexter_connect_port
 *
 * implements LADSPA (*connect_port)()
 */
static void
hexter_connect_port(LADSPA_Handle handle, unsigned long port, LADSPA_Data *data)
{
    hexter_instance_t *instance = (hexter_instance_t *)handle;

	switch (port) {
      case HEXTER_PORT_OUTPUT:  instance->output = data;  break;
      case HEXTER_PORT_TUNING:  instance->tuning = data;  break;
      case HEXTER_PORT_VOLUME:  instance->volume = data;  break;
      default:
        break;
    }
}

/*
 * hexter_activate
 *
 * implements LADSPA (*activate)()
 */
static void
hexter_activate(LADSPA_Handle handle)
{
    hexter_instance_t *instance = (hexter_instance_t *)handle;

    hexter_instance_all_voices_off(instance);  /* stop all sounds immediately */
    instance->current_voices = 0;
    dx7_lfo_reset(instance);
}

/*
 * hexter_ladspa_run
 */
static void
hexter_ladspa_run(LADSPA_Handle instance, unsigned long sample_count)
{
	hexter_run_synth(instance, sample_count, NULL, 0);
}

// optional:
//  void (*run_adding)(LADSPA_Handle Instance,
//                     unsigned long SampleCount);
//  void (*set_run_adding_gain)(LADSPA_Handle Instance,
//                              LADSPA_Data   Gain);

/*
 * hexter_deactivate
 *
 * implements LADSPA (*deactivate)()
 */
void
hexter_deactivate(LADSPA_Handle handle)
{
    hexter_instance_t *instance = (hexter_instance_t *)handle;

    hexter_instance_all_voices_off(instance);  /* stop all sounds immediately */
}

/*
 * hexter_cleanup
 *
 * implements LADSPA (*cleanup)()
 */
static void
hexter_cleanup(LADSPA_Handle handle)
{
    hexter_instance_t *instance = (hexter_instance_t *)handle;
    int i;

    if (instance) {
        hexter_deactivate(instance);

        if (instance->patches) free(instance->patches);
        for (i = 0; i < HEXTER_MAX_POLYPHONY; i++) {
            if (instance->voice[i]) {
                free(instance->voice[i]);
                instance->voice[i] = NULL;
            }
        }
        free(instance);
    }
}

/* ---- DSSI interface ---- */

/*
 * hexter_configure
 *
 * implements DSSI (*configure)()
 */
char *
hexter_configure(LADSPA_Handle handle, const char *key, const char *value)
{
    hexter_instance_t *instance = (hexter_instance_t *)handle;

    DEBUG_MESSAGE(DB_DSSI, " hexter_configure called with '%s' and '%s'\n", key, value);

    if (strlen(key) == 8 && !strncmp(key, "patches", 7)) {

        return hexter_instance_handle_patches(instance, key, value);

    } else if (!strcmp(key, "edit_buffer")) {

        return hexter_instance_handle_edit_buffer(instance, value);

    } else if (!strcmp(key, "performance")) {  /* global performance parameters */

        return hexter_instance_handle_performance(instance, value);

    } else if (!strcmp(key, "monophonic")) {

        return hexter_instance_handle_monophonic(instance, value);

    } else if (!strcmp(key, "polyphony")) {

        return hexter_instance_handle_polyphony(instance, value);

#ifdef DSSI_GLOBAL_CONFIGURE_PREFIX
    } else if (!strcmp(key, DSSI_GLOBAL_CONFIGURE_PREFIX "polyphony")) {
#else
    } else if (!strcmp(key, "global_polyphony")) {
#endif

        DEBUG_MESSAGE(DB_DSSI, " -- global polyphony limiting is no longer supported --\n");

#ifdef DSSI_PROJECT_DIRECTORY_KEY
    } else if (!strcmp(key, DSSI_PROJECT_DIRECTORY_KEY)) {

        return NULL; /* plugin has no use for project directory key, ignore it */

#endif
    }
    return strdup("error: unrecognized configure key");
}

/*
 * hexter_get_program
 *
 * implements DSSI (*get_program)()
 */
const DSSI_Program_Descriptor *
hexter_get_program(LADSPA_Handle handle, unsigned long index)
{
    hexter_instance_t *instance = (hexter_instance_t *)handle;
    static DSSI_Program_Descriptor pd;

    DEBUG_MESSAGE(DB_DSSI, " hexter_get_program called with %lu\n", index);

    if (index < 128) {
        hexter_instance_set_program_descriptor(instance, &pd, 0, index);
        return &pd;
    }
    return NULL;
}

/*
 * hexter_select_program
 *
 * implements DSSI (*select_program)()
 */
void
hexter_select_program(LADSPA_Handle handle, unsigned long bank,
                      unsigned long program)
{
    hexter_instance_t *instance = (hexter_instance_t *)handle;

    DEBUG_MESSAGE(DB_DSSI, " hexter_select_program called with %lu and %lu\n", bank, program);

    /* ignore invalid program requests */
    if (bank || program >= 128)
        return;

    /* Attempt the patch mutex, return if lock fails. */
    if (pthread_mutex_trylock(&instance->patches_mutex)) {
        instance->pending_program_change = program;
        return;
    }

    hexter_instance_select_program((hexter_instance_t *)instance, bank, program);

    pthread_mutex_unlock(&instance->patches_mutex);
}

/*
 * hexter_handle_pending_program_change
 */
static inline void
hexter_handle_pending_program_change(hexter_instance_t *instance)
{
    /* Attempt the patch mutex, return if lock fails. */
    if (pthread_mutex_trylock(&instance->patches_mutex))
        return;

    hexter_instance_select_program((hexter_instance_t *)instance, 0,
                                   instance->pending_program_change);
    instance->pending_program_change = -1;

    pthread_mutex_unlock(&instance->patches_mutex);
}

/*
 * hexter_get_midi_controller
 *
 * implements DSSI (*get_midi_controller_for_port)()
 */
int
hexter_get_midi_controller(LADSPA_Handle handle, unsigned long port)
{
    DEBUG_MESSAGE(DB_DSSI, " hexter_get_midi_controller called for port %lu\n", port);
    switch (port) {
      // case EXAMPLE_PORT_VOLUME:
      //   return DSSI_CC(7);
      default:
        break;
    }

    return DSSI_NONE;
}

/*
 * hexter_handle_event
 */
static inline void
hexter_handle_event(hexter_instance_t *instance, snd_seq_event_t *event)
{
    DEBUG_MESSAGE(DB_DSSI, " hexter_handle_event called with event type %d\n", event->type);

    switch (event->type) {
      case SND_SEQ_EVENT_NOTEOFF:
        hexter_instance_note_off(instance, event->data.note.note, event->data.note.velocity);
        break;
      case SND_SEQ_EVENT_NOTEON:
        if (event->data.note.velocity > 0)
            hexter_instance_note_on(instance, event->data.note.note, event->data.note.velocity);
        else
            hexter_instance_note_off(instance, event->data.note.note, 64); /* shouldn't happen, but... */
        break;
      case SND_SEQ_EVENT_KEYPRESS:
        hexter_instance_key_pressure(instance, event->data.note.note, event->data.note.velocity);
        break;
      case SND_SEQ_EVENT_CONTROLLER:
        hexter_instance_control_change(instance, event->data.control.param, event->data.control.value);
        break;
      case SND_SEQ_EVENT_CHANPRESS:
        hexter_instance_channel_pressure(instance, event->data.control.value);
        break;
      case SND_SEQ_EVENT_PITCHBEND:
        hexter_instance_pitch_bend(instance, event->data.control.value);
        break;
      /* SND_SEQ_EVENT_PGMCHANGE - shouldn't happen */
      /* SND_SEQ_EVENT_SYSEX - shouldn't happen */
      /* SND_SEQ_EVENT_CONTROL14? */
      /* SND_SEQ_EVENT_NONREGPARAM? */
      /* SND_SEQ_EVENT_REGPARAM? */  default:
        break;
    }
}

static void
hexter_run_synth(LADSPA_Handle handle, unsigned long sample_count,
                 snd_seq_event_t *events, unsigned long event_count)
{
    hexter_instance_t *instance = (hexter_instance_t *)handle;
    
    // Set external volume
    if (volume) {
		instance->volume = &volume;
	}
	
    unsigned long samples_done = 0;
    unsigned long event_index = 0;
    unsigned long burst_size;

    /* silence the buffer */
    memset(instance->output, 0, sizeof(LADSPA_Data) * sample_count);
#if defined(DSSP_DEBUG) && (DSSP_DEBUG & DB_AUDIO)
*instance->output += 0.10f; /* add a 'buzz' to output so there's something audible even when quiescent */
#endif /* defined(DSSP_DEBUG) && (DSSP_DEBUG & DB_AUDIO) */

    /* attempt the mutex, return only silence if lock fails. */
    if (dssp_voicelist_mutex_trylock(instance))
        return;

    if (instance->pending_program_change > -1)
        hexter_handle_pending_program_change(instance);

    while (samples_done < sample_count) {

        if (!instance->nugget_remains)
            instance->nugget_remains = HEXTER_NUGGET_SIZE;

        /* process any ready events */
        while (event_index < event_count
               && samples_done == events[event_index].time.tick) {
            hexter_handle_event(instance, &events[event_index]);
            event_index++;
        }

        /* calculate the sample count (burst_size) for the next
         * hexter_instance_render_voices() call to be the smallest of:
         * - control calculation quantization size (HEXTER_NUGGET_SIZE,
         *     in samples)
         * - the number of samples remaining in an already-begun nugget
         *     (instance->nugget_remains)
         * - the number of samples until the next event is ready
         * - the number of samples left in this run
         */
        burst_size = HEXTER_NUGGET_SIZE;
        if (instance->nugget_remains < burst_size) {
            /* we're still in the middle of a nugget, so reduce the burst size
             * to end when the nugget ends */
            burst_size = instance->nugget_remains;
        }
        if (event_index < event_count
            && events[event_index].time.tick - samples_done < burst_size) {
            /* reduce burst size to end when next event is ready */
            burst_size = events[event_index].time.tick - samples_done;
        }
        if (sample_count - samples_done < burst_size) {
            /* reduce burst size to end at end of this run */
            burst_size = sample_count - samples_done;
        }

        /* render the burst */
        hexter_instance_render_voices(instance, samples_done, burst_size,
                                      (burst_size == instance->nugget_remains));
        samples_done += burst_size;
        instance->nugget_remains -= burst_size;
    }

    dssp_voicelist_mutex_unlock(instance);
}

// optional:
//    void (*run_synth_adding)(LADSPA_Handle    Instance,
//                             unsigned long    SampleCount,
//                             snd_seq_event_t *Events,
//                             unsigned long    EventCount);
//    void (*run_multiple_synths)(unsigned long     InstanceCount,
//                                LADSPA_Handle   **Instances,
//                                unsigned long     SampleCount,
//                                snd_seq_event_t **Events,
//                                unsigned long    *EventCounts);
//    void (*run_multiple_synths_adding)(unsigned long     InstanceCount,
//                                       LADSPA_Handle   **Instances,
//                                       unsigned long     SampleCount,
//                                       snd_seq_event_t **Events,
//                                       unsigned long    *EventCounts);

/* ---- export ---- */

const LADSPA_Descriptor *ladspa_descriptor(unsigned long index)
{
    switch (index) {
      case 0:
        return hexter_LADSPA_descriptor;
      default:
        return NULL;
    }
}

const DSSI_Descriptor *dssi_descriptor(unsigned long index)
{
    switch (index) {
      case 0:
        return hexter_DSSI_descriptor;
      default:
        return NULL;
    }
}

#ifdef __GNUC__
__attribute__((constructor)) void init()
#else
void _init()
#endif
{
    char **port_names;
    LADSPA_PortDescriptor *port_descriptors;
    LADSPA_PortRangeHint *port_range_hints;

    DSSP_DEBUG_INIT("hexter.so");

    dx7_voice_init_tables();

    hexter_LADSPA_descriptor =
        (LADSPA_Descriptor *) malloc(sizeof(LADSPA_Descriptor));
    if (hexter_LADSPA_descriptor) {
        hexter_LADSPA_descriptor->UniqueID = 2183;
        hexter_LADSPA_descriptor->Label = "hexter";
        hexter_LADSPA_descriptor->Properties = 0;
        hexter_LADSPA_descriptor->Name = "hexter DX7 emulation (v" VERSION ")";
        hexter_LADSPA_descriptor->Maker = "Sean Bolton <sean AT smbolton DOT com>";
        hexter_LADSPA_descriptor->Copyright = "GNU General Public License version 2 or later";
        hexter_LADSPA_descriptor->PortCount = HEXTER_PORTS_COUNT;

        port_descriptors = (LADSPA_PortDescriptor *)
                                calloc(hexter_LADSPA_descriptor->PortCount, sizeof
                                                (LADSPA_PortDescriptor));
        hexter_LADSPA_descriptor->PortDescriptors =
            (const LADSPA_PortDescriptor *) port_descriptors;

        port_range_hints = (LADSPA_PortRangeHint *)
                                calloc(hexter_LADSPA_descriptor->PortCount, sizeof
                                                (LADSPA_PortRangeHint));
        hexter_LADSPA_descriptor->PortRangeHints =
            (const LADSPA_PortRangeHint *) port_range_hints;

        port_names = (char **) calloc(hexter_LADSPA_descriptor->PortCount, sizeof(char *));
        hexter_LADSPA_descriptor->PortNames = (const char **) port_names;

        /* Parameters for Output */
        port_descriptors[HEXTER_PORT_OUTPUT] = LADSPA_PORT_OUTPUT | LADSPA_PORT_AUDIO;
        port_names[HEXTER_PORT_OUTPUT] = "Output";
        port_range_hints[HEXTER_PORT_OUTPUT].HintDescriptor = 0;

        /* Parameters for Tuning */
        port_descriptors[HEXTER_PORT_TUNING] = LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        port_names[HEXTER_PORT_TUNING] = "Tuning";
        port_range_hints[HEXTER_PORT_TUNING].HintDescriptor =
                        LADSPA_HINT_DEFAULT_440 |
                        LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE;
        port_range_hints[HEXTER_PORT_TUNING].LowerBound = 415.3f;
        port_range_hints[HEXTER_PORT_TUNING].UpperBound = 466.2f;

        /* Parameters for Volume */
        port_descriptors[HEXTER_PORT_VOLUME] = LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        port_names[HEXTER_PORT_VOLUME] = "Volume";
        port_range_hints[HEXTER_PORT_VOLUME].HintDescriptor =
                        LADSPA_HINT_DEFAULT_0 |
                        LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE;
        port_range_hints[HEXTER_PORT_VOLUME].LowerBound = -70.0f;
        port_range_hints[HEXTER_PORT_VOLUME].UpperBound =  20.0f;

        hexter_LADSPA_descriptor->instantiate = hexter_instantiate;
        hexter_LADSPA_descriptor->connect_port = hexter_connect_port;
        hexter_LADSPA_descriptor->activate = hexter_activate;
        hexter_LADSPA_descriptor->run = hexter_ladspa_run;
        hexter_LADSPA_descriptor->run_adding = NULL;
        hexter_LADSPA_descriptor->set_run_adding_gain = NULL;
        hexter_LADSPA_descriptor->deactivate = hexter_deactivate;
        hexter_LADSPA_descriptor->cleanup = hexter_cleanup;
    }

    hexter_DSSI_descriptor = (DSSI_Descriptor *) malloc(sizeof(DSSI_Descriptor));
    if (hexter_DSSI_descriptor) {
        hexter_DSSI_descriptor->DSSI_API_Version = 1;
        hexter_DSSI_descriptor->LADSPA_Plugin = hexter_LADSPA_descriptor;
        hexter_DSSI_descriptor->configure = hexter_configure;
        hexter_DSSI_descriptor->get_program = hexter_get_program;
        hexter_DSSI_descriptor->select_program = hexter_select_program;
        hexter_DSSI_descriptor->get_midi_controller_for_port = hexter_get_midi_controller;
        hexter_DSSI_descriptor->run_synth = hexter_run_synth;
        hexter_DSSI_descriptor->run_synth_adding = NULL;
        hexter_DSSI_descriptor->run_multiple_synths = NULL;
        hexter_DSSI_descriptor->run_multiple_synths_adding = NULL;
    }
}

#ifdef __GNUC__
__attribute__((destructor)) void fini()
#else
void _fini()
#endif
{
    if (hexter_LADSPA_descriptor) {
        free((LADSPA_PortDescriptor *) hexter_LADSPA_descriptor->PortDescriptors);
        free((char **) hexter_LADSPA_descriptor->PortNames);
        free((LADSPA_PortRangeHint *) hexter_LADSPA_descriptor->PortRangeHints);
        free(hexter_LADSPA_descriptor);
    }
    if (hexter_DSSI_descriptor) {
        free(hexter_DSSI_descriptor);
    }
}

/*
 * dx7_patchbank_load
 */
int
dx7_patchbank_load_init(const char *filename, dx7_patch_t *firstpatch,
                        int maxpatches, char **errmsg)
{
    FILE *fp;
    long filelength;
    unsigned char *raw_patch_data = NULL;
    size_t filename_length;
    int count;
    int patchstart;
    int midshift;
    int datastart;
    int i;
    int op;

    /* this needs to 1) open and parse the file, 2a) if it's good, copy up
     * to maxpatches patches beginning at firstpath, and not touch errmsg,
     * 2b) if it's not good, set errmsg to a malloc'd error message that
     * the caller must free. */

    if ((fp = fopen(filename, "rb")) == NULL) {
        if (errmsg) *errmsg = dssp_error_message("could not open file '%s' for reading: %s", filename, strerror(errno));
        return 0;
    }

    if (fseek(fp, 0, SEEK_END) ||
        (filelength = ftell(fp)) == -1 ||
        fseek(fp, 0, SEEK_SET)) {
        if (errmsg) *errmsg = dssp_error_message("couldn't get length of patch file: %s", strerror(errno));
        fclose(fp);
        return 0;
    }
    if (filelength == 0) {
        if (errmsg) *errmsg = strdup("patch file has zero length");
        fclose(fp);
        return 0;
    } else if (filelength > 2097152) {
        if (errmsg) *errmsg = strdup("patch file is too large");
        fclose(fp);
        return 0;
    } else if (filelength < 128) {
        if (errmsg) *errmsg = strdup ("patch file is too small");
        fclose (fp);
        return 0;
    }

    if (!(raw_patch_data = (unsigned char *)malloc(filelength))) {
        if (errmsg) *errmsg = strdup("couldn't allocate memory for raw patch file");
        fclose(fp);
        return 0;
    }

    if (fread(raw_patch_data, 1, filelength, fp) != (size_t)filelength) {
        if (errmsg) *errmsg = dssp_error_message("short read on patch file: %s", strerror(errno));
        free(raw_patch_data);
        fclose(fp);
        return 0;
    }
    fclose(fp);
    filename_length = strlen (filename);

    /* check if the file is a standard MIDI file */
    if (raw_patch_data[0] == 0x4d &&    /* "M" */
        raw_patch_data[1] == 0x54 &&    /* "T" */
        raw_patch_data[2] == 0x68 &&    /* "h" */
        raw_patch_data[3] == 0x64)      /* "d" */
        midshift = 2;
    else
        midshift = 0;

    /* scan SysEx or MIDI file for SysEx header(s) */
    count = 0;
    datastart = 0;
    for (patchstart = 0; patchstart + midshift + 5 < filelength; patchstart++) {

        if (raw_patch_data[patchstart] == 0xf0 &&
            raw_patch_data[patchstart + 1 + midshift] == 0x43 &&
            raw_patch_data[patchstart + 2 + midshift] <= 0x0f &&
            raw_patch_data[patchstart + 3 + midshift] == 0x09 &&
            raw_patch_data[patchstart + 5 + midshift] == 0x00 &&
            patchstart + 4103 + midshift < filelength &&
            raw_patch_data[patchstart + 4103 + midshift] == 0xf7) {  /* DX7 32 voice dump */

            memmove(raw_patch_data + count * DX7_VOICE_SIZE_PACKED,
                    raw_patch_data + patchstart + 6 + midshift, 4096);
            count += 32;
            patchstart += (DX7_DUMP_SIZE_VOICE_BULK - 1);

        } else if (raw_patch_data[patchstart] == 0xf0 &&
                   raw_patch_data[patchstart + midshift + 1] == 0x43 &&
                   raw_patch_data[patchstart + midshift + 2] <= 0x0f &&
                   raw_patch_data[patchstart + midshift + 4] == 0x01 &&
                   raw_patch_data[patchstart + midshift + 5] == 0x1b &&
                   patchstart + midshift + 162 < filelength &&
                   raw_patch_data[patchstart + midshift + 162] == 0xf7) {  /* DX7 single voice (edit buffer) dump */

            unsigned char buf[DX7_VOICE_SIZE_PACKED]; /* to avoid overlap in dx7_patch_pack() */

            dx7_patch_pack(raw_patch_data + patchstart + midshift + 6,
                           (dx7_patch_t *)buf, 0);
            memcpy(raw_patch_data + count * DX7_VOICE_SIZE_PACKED,
                   buf, DX7_VOICE_SIZE_PACKED);

            count += 1;
            patchstart += (DX7_DUMP_SIZE_VOICE_SINGLE - 1);
        }
    }

    /* assume raw DX7/TX7 data if no SysEx header was found. */
    /* assume the user knows what he is doing ;-) */

    if (count == 0)
        count = filelength / DX7_VOICE_SIZE_PACKED;

    /* Dr.T and Steinberg TX7 file needs special treatment */
    if ((!strcmp(filename + filename_length - 4, ".TX7") ||
         !strcmp(filename + filename_length -4, ".SND") ||
         !strcmp(filename + filename_length -4, ".tx7") ||
         !strcmp(filename + filename_length - 4, ".snd")) && filelength == 8192) {

        count = 32;
        filelength = 4096;
    }

    /* Transform XSyn file also needs special treatment */
    if ((!strcmp(filename + filename_length - 4, ".BNK") ||
         !strcmp(filename + filename_length - 4, ".bnk")) && filelength == 8192) {

        for (i=0; i<32; i++)
        {
            memmove(raw_patch_data + 128*i, raw_patch_data + 256*i, 128);
        }
        count = 32;
        filelength = 4096;
    }

    /* Steinberg Synthworks DX7 SND */
    if ((!strcmp (filename + filename_length - 4, ".SND") ||
         !strcmp (filename + filename_length - 4, ".snd")) && filelength == 5216) {

        count = 32;
        filelength = 4096;
    }

    /* Voyetra SIDEMAN DX/TX
     * Voyetra Patchmaster DX7/TX7 */
    if ((filelength == 9816 || filelength == 5663) &&
        raw_patch_data[0] == 0xdf &&
        raw_patch_data[1] == 0x05 &&
        raw_patch_data[2] == 0x01 && raw_patch_data[3] == 0x00) {

        count = 32;
        datastart = 0x60f;
    }

    /* Yamaha DX200 editor .DX2 */
    if ((!strcmp (filename + filename_length - 4, ".DX2") ||
         !strcmp (filename + filename_length - 4, ".dx2"))
        && filelength == 326454)
      {
          memmove (raw_patch_data + 16384, raw_patch_data + 34, 128 * 381);
          for (count = 0; count < 128; count++)
            {
                for (op = 0; op < 6; op++)
                  {
                      for (i = 0; i < 8; i++)
                        {
                            raw_patch_data[17 * (5 - op) + i + 128 * count] =
                                raw_patch_data[16384 + 35 * op + 76 + i + 381 * count];
                        }
                      raw_patch_data[17 * (5 - op) + 8 + 128 * count] =
                          raw_patch_data[16384 + 35 * op + 84 + 381 * count] - 21;
                      raw_patch_data[17 * (5 - op) + 9 + 128 * count] =
                          raw_patch_data[16384 + 35 * op + 87 + 381 * count];
                      raw_patch_data[17 * (5 - op) + 10 + 128 * count] =
                          raw_patch_data[16384 + 35 * op + 88 + 381 * count];
                      raw_patch_data[17 * (5 - op) + 11 + 128 * count] =
                          raw_patch_data[16384 + 35 * op + 85 + 381 * count] +
                          raw_patch_data[16384 + 35 * op + 86 + 381 * count] * 4;
                      raw_patch_data[17 * (5 - op) + 12 + 128 * count] =
                          raw_patch_data[16384 + 35 * op + 89 + 381 * count] +
                          raw_patch_data[16384 + 35 * op + 75 + 381 * count] * 8;
                      if (raw_patch_data[16384 + 35 * op + 71 + 381 * count] > 3)
                          raw_patch_data[16384 + 35 * op + 71 + 381 * count] = 3;
                      raw_patch_data[17 * (5 - op) + 13 + 128 * count] =
                          raw_patch_data[16384 + 35 * op + 71 + 381 * count] / 2 +
                          raw_patch_data[16384 + 35 * op + 91 + 381 * count] * 4;
                      raw_patch_data[17 * (5 - op) + 14 + 128 * count] =
                          raw_patch_data[16384 + 35 * op + 90 + 381 * count];
                      raw_patch_data[17 * (5 - op) + 15 + 128 * count] =
                          raw_patch_data[16384 + 35 * op + 72 + 381 * count] +
                          raw_patch_data[16384 + 35 * op + 73 + 381 * count] * 2;
                      raw_patch_data[17 * (5 - op) + 16 + 128 * count] =
                          raw_patch_data[16384 + 35 * op + 74 + 381 * count];
                  }
                for (i = 0; i < 4; i++)
                  {
                      raw_patch_data[102 + i + 128 * count] =
                          raw_patch_data[16384 + 26 + i + 381 * count];
                  }
                for (i = 0; i < 4; i++)
                  {
                      raw_patch_data[106 + i + 128 * count] =
                          raw_patch_data[16384 + 32 + i + 381 * count];
                  }
                raw_patch_data[110 + 128 * count] =
                    raw_patch_data[16384 + 17 + 381 * count];
                raw_patch_data[111 + 128 * count] =
                    raw_patch_data[16384 + 18 + 381 * count] +
                    raw_patch_data[16384 + 38 + 381 * count] * 8;
                for (i = 0; i < 4; i++)
                  {
                      raw_patch_data[112 + i + 128 * count] =
                          raw_patch_data[16384 + 20 + i + 381 * count];
                  }
                raw_patch_data[116 + 128 * count] =
                    raw_patch_data[16384 + 24 + 381 * count] +
                    raw_patch_data[16384 + 19 + 381 * count] * 2 +
                    raw_patch_data[16384 + 25 + 381 * count] * 16;
                raw_patch_data[117 + 128 * count] =
                    raw_patch_data[16384 + 37 + 381 * count] - 36;
                for (i = 0; i < 10; i++)
                  {
                      raw_patch_data[118 + i + 128 * count] =
                          raw_patch_data[16384 + i + 381 * count];
                  }
            }

          count = 128;
          filelength = 16384;
          datastart = 0;

      }

    /* finally, copy patchdata to the right location */
    if (count > maxpatches)
        count = maxpatches;

    memcpy(firstpatch, raw_patch_data + datastart, 128 * count);
    free (raw_patch_data);
    return count;
}
