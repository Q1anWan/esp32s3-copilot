#include "copilot_face_data.h"

/*
Angles are degrees in LVGL arc space.
Eye/mouth offsets are pixel offsets applied from the base positions.
*/

const face_arc_keyframe_t kFaceKeyframes[COPILOT_EXPR_COUNT] = {
    /* COPILOT_EXPR_NEUTRAL */
    {200, 340, 0, 0, 30, 150, 0, 0},
    /* COPILOT_EXPR_HAPPY */
    {210, 330, 0, -4, 10, 170, 0, -4},
    /* COPILOT_EXPR_SAD */
    {200, 340, 0, 6, 190, 350, 0, 8},
    /* COPILOT_EXPR_ANGRY */
    {220, 320, 350, 2, 70, 110, 0, 0},
    /* COPILOT_EXPR_SURPRISED */
    {0, 360, 0, 0, 0, 360, 0, 4},
    /* COPILOT_EXPR_SLEEPY */
    {240, 300, 0, 6, 60, 120, 0, 6},
    /* COPILOT_EXPR_DIZZY */
    {30, 330, 90, 0, 0, 360, 0, 2},
};
