#ifndef COPILOT_AUDIO_H
#define COPILOT_AUDIO_H

#include <stdbool.h>
#include <stdint.h>

#define COPILOT_AUDIO_PATH_MAX 160

typedef struct {
    bool ready;
    bool sd_mounted;
    bool playing_file;
    uint32_t files_played;
    char current_path[COPILOT_AUDIO_PATH_MAX];
    char last_error[96];
} copilot_audio_status_t;

typedef enum {
    COPILOT_AUDIO_FILE_DONE = 0,
    COPILOT_AUDIO_FILE_FAILED,
} copilot_audio_file_event_kind_t;

typedef struct {
    copilot_audio_file_event_kind_t kind;
    uint32_t generation;
    const char *scene;
    uint16_t sequence;
    const char *path;
    const char *error;
} copilot_audio_file_event_t;

typedef void (*copilot_audio_file_event_cb_t)(const copilot_audio_file_event_t *event, void *user);

void copilot_audio_init(void);
void copilot_audio_set_file_event_callback(copilot_audio_file_event_cb_t cb, void *user);
void copilot_audio_play(const char *sound_id);
bool copilot_audio_play_scene(const char *scene_id, uint16_t sequence_id);
bool copilot_audio_play_path(const char *path);
void copilot_audio_stop(void);
bool copilot_audio_is_ready(void);
bool copilot_audio_get_status(copilot_audio_status_t *out_status);

#endif
