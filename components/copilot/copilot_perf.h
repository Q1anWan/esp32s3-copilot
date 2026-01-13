#pragma once

#include "sdkconfig.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the performance profiling module
 *
 * Call this after FreeRTOS scheduler is started.
 */
void copilot_perf_init(void);

/**
 * @brief Record an animation frame for FPS calculation
 *
 * Call this from the animation timer callback.
 */
void copilot_perf_frame_tick(void);

/**
 * @brief Get the current animation FPS
 *
 * @return Current frames per second (0-255)
 */
uint8_t copilot_perf_get_fps(void);

/**
 * @brief Force an immediate performance report output
 */
void copilot_perf_report_now(void);

#ifdef __cplusplus
}
#endif
