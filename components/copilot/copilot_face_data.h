#ifndef COPILOT_FACE_DATA_H
#define COPILOT_FACE_DATA_H

#include <stdint.h>

typedef struct {
    int16_t eye_start;
    int16_t eye_end;
    int16_t eye_rotation;
    int16_t eye_offset_y;
    int16_t mouth_start;
    int16_t mouth_end;
    int16_t mouth_rotation;
    int16_t mouth_offset_y;
} face_arc_keyframe_t;

/*
Arc angles in degrees (0..360). 0 is to the right, 90 is down.
These keyframes drive a minimal emoticon face:
- two eye arcs
- one mouth arc
Edit the values in copilot_face_data.cpp to tune expressions.
*/

typedef enum {
    COPILOT_EXPR_NEUTRAL = 0,
    COPILOT_EXPR_HAPPY,
    COPILOT_EXPR_SAD,
    COPILOT_EXPR_ANGRY,
    COPILOT_EXPR_SURPRISED,
    COPILOT_EXPR_SLEEPY,
    COPILOT_EXPR_DIZZY,
    COPILOT_EXPR_COUNT
} copilot_expr_t;

extern const face_arc_keyframe_t kFaceKeyframes[COPILOT_EXPR_COUNT];

#endif
