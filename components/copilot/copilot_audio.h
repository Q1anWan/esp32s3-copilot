#ifndef COPILOT_AUDIO_H
#define COPILOT_AUDIO_H

#include <stdbool.h>

void copilot_audio_init(void);
void copilot_audio_play(const char *sound_id);
bool copilot_audio_is_ready(void);

#endif
