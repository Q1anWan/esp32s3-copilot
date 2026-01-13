#include "copilot_perf.h"

#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if CONFIG_COPILOT_PERF_HEAP
#include "esp_heap_caps.h"
#endif

static const char *TAG = "copilot_perf";

#if CONFIG_COPILOT_PERF_ENABLE

// FPS calculation state
static volatile uint32_t s_frame_count = 0;
static volatile uint32_t s_last_fps_time = 0;
static volatile uint8_t s_current_fps = 0;
static volatile uint32_t s_fps_sample_count = 0;
static volatile uint32_t s_fps_sum = 0;
static volatile uint8_t s_fps_min = 255;
static volatile uint8_t s_fps_max = 0;

// Report timing
static uint32_t s_last_report_time = 0;
static esp_timer_handle_t s_report_timer = nullptr;

#if CONFIG_COPILOT_PERF_RTOS_STATS
// Task info buffer
#define MAX_TASKS 16
static TaskStatus_t s_task_status[MAX_TASKS];
#endif

static void copilot_perf_update_fps(void) {
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);  // ms
    uint32_t elapsed = now - s_last_fps_time;

    if (elapsed >= 1000) {
        uint8_t fps = (uint8_t)((s_frame_count * 1000) / elapsed);
        s_current_fps = fps;

        // Update statistics
        s_fps_sum += fps;
        s_fps_sample_count++;
        if (fps < s_fps_min) s_fps_min = fps;
        if (fps > s_fps_max) s_fps_max = fps;

        s_frame_count = 0;
        s_last_fps_time = now;
    }
}

#if CONFIG_COPILOT_PERF_RTOS_STATS
static void copilot_perf_report_rtos(void) {
    UBaseType_t task_count = uxTaskGetNumberOfTasks();
    if (task_count > MAX_TASKS) {
        task_count = MAX_TASKS;
    }

    uint32_t total_runtime;
    UBaseType_t filled = uxTaskGetSystemState(s_task_status, task_count, &total_runtime);

    if (filled == 0) {
        ESP_LOGW(TAG, "RTOS stats not available");
        return;
    }

    ESP_LOGI(TAG, "=== RTOS Task Statistics ===");
    ESP_LOGI(TAG, "%-16s %6s %8s %6s", "Task", "State", "FreeStk", "CPU%");

    for (UBaseType_t i = 0; i < filled; i++) {
        TaskStatus_t *t = &s_task_status[i];

        // Calculate CPU percentage (runtime * 100 / total)
        uint32_t cpu_percent = 0;
        if (total_runtime > 0) {
            cpu_percent = (t->ulRunTimeCounter * 100UL) / total_runtime;
        }

        // Stack high watermark = minimum free stack ever (in words -> bytes)
        uint32_t free_stack = t->usStackHighWaterMark * sizeof(StackType_t);

        // Task state string
        const char* state_str;
        switch (t->eCurrentState) {
            case eRunning:   state_str = "Run"; break;
            case eReady:     state_str = "Ready"; break;
            case eBlocked:   state_str = "Block"; break;
            case eSuspended: state_str = "Susp"; break;
            case eDeleted:   state_str = "Del"; break;
            default:         state_str = "?"; break;
        }

        ESP_LOGI(TAG, "%-16s %6s %8lu %5lu%%",
                 t->pcTaskName,
                 state_str,
                 (unsigned long)free_stack,
                 (unsigned long)cpu_percent);
    }

    // Show total runtime info
    if (total_runtime > 0) {
        ESP_LOGI(TAG, "Total runtime ticks: %lu", (unsigned long)total_runtime);
    } else {
        ESP_LOGW(TAG, "Note: Enable configGENERATE_RUN_TIME_STATS for CPU%% stats");
    }
}
#endif

#if CONFIG_COPILOT_PERF_HEAP
static void copilot_perf_report_heap(void) {
    ESP_LOGI(TAG, "=== Heap Memory Statistics ===");

    // Internal RAM
    size_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t internal_total = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
    size_t internal_min = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
    size_t internal_largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);

    ESP_LOGI(TAG, "Internal RAM:");
    ESP_LOGI(TAG, "  Free: %u / %u bytes (%.1f%%)",
             internal_free, internal_total,
             internal_total > 0 ? (float)internal_free * 100.0f / internal_total : 0);
    ESP_LOGI(TAG, "  Min free: %u bytes", internal_min);
    ESP_LOGI(TAG, "  Largest block: %u bytes", internal_largest);

    // PSRAM (SPIRAM)
    size_t spiram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    size_t spiram_total = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    size_t spiram_min = heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM);
    size_t spiram_largest = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);

    if (spiram_total > 0) {
        ESP_LOGI(TAG, "SPIRAM (PSRAM):");
        ESP_LOGI(TAG, "  Free: %u / %u bytes (%.1f%%)",
                 spiram_free, spiram_total,
                 spiram_total > 0 ? (float)spiram_free * 100.0f / spiram_total : 0);
        ESP_LOGI(TAG, "  Min free: %u bytes", spiram_min);
        ESP_LOGI(TAG, "  Largest block: %u bytes", spiram_largest);
    }

    // DMA-capable memory
    size_t dma_free = heap_caps_get_free_size(MALLOC_CAP_DMA);
    size_t dma_largest = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);
    ESP_LOGI(TAG, "DMA-capable:");
    ESP_LOGI(TAG, "  Free: %u bytes, Largest: %u bytes", dma_free, dma_largest);
}
#endif

#if CONFIG_COPILOT_PERF_FPS
static void copilot_perf_report_fps(void) {
    ESP_LOGI(TAG, "=== Animation FPS Statistics ===");

    uint8_t avg_fps = 0;
    if (s_fps_sample_count > 0) {
        avg_fps = (uint8_t)(s_fps_sum / s_fps_sample_count);
    }

    ESP_LOGI(TAG, "Current FPS: %u", s_current_fps);
    ESP_LOGI(TAG, "Average FPS: %u (min: %u, max: %u, samples: %lu)",
             avg_fps,
             s_fps_min == 255 ? 0 : s_fps_min,
             s_fps_max,
             (unsigned long)s_fps_sample_count);

    // Reset statistics after report
    s_fps_sum = 0;
    s_fps_sample_count = 0;
    s_fps_min = 255;
    s_fps_max = 0;
}
#endif

static void copilot_perf_report_cb(void *arg) {
    (void)arg;
    copilot_perf_report_now();
}

void copilot_perf_init(void) {
    ESP_LOGI(TAG, "Performance profiling enabled");
    ESP_LOGI(TAG, "  Report interval: %d ms", CONFIG_COPILOT_PERF_INTERVAL_MS);
#if CONFIG_COPILOT_PERF_FPS
    ESP_LOGI(TAG, "  FPS monitoring: enabled");
#endif
#if CONFIG_COPILOT_PERF_RTOS_STATS
    ESP_LOGI(TAG, "  RTOS stats: enabled");
#endif
#if CONFIG_COPILOT_PERF_HEAP
    ESP_LOGI(TAG, "  Heap monitoring: enabled");
#endif

    s_last_fps_time = (uint32_t)(esp_timer_get_time() / 1000);
    s_last_report_time = s_last_fps_time;

    // Create periodic timer for reports
    esp_timer_create_args_t timer_args = {
        .callback = copilot_perf_report_cb,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "perf_report",
        .skip_unhandled_events = true,
    };

    esp_err_t err = esp_timer_create(&timer_args, &s_report_timer);
    if (err == ESP_OK) {
        esp_timer_start_periodic(s_report_timer, CONFIG_COPILOT_PERF_INTERVAL_MS * 1000);
    } else {
        ESP_LOGE(TAG, "Failed to create perf timer: %d", err);
    }
}

void copilot_perf_frame_tick(void) {
    s_frame_count++;
    copilot_perf_update_fps();
}

uint8_t copilot_perf_get_fps(void) {
    return s_current_fps;
}

void copilot_perf_report_now(void) {
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "       PERFORMANCE PROFILING REPORT");
    ESP_LOGI(TAG, "========================================");

#if CONFIG_COPILOT_PERF_FPS
    copilot_perf_report_fps();
#endif

#if CONFIG_COPILOT_PERF_RTOS_STATS
    copilot_perf_report_rtos();
#endif

#if CONFIG_COPILOT_PERF_HEAP
    copilot_perf_report_heap();
#endif

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "");
}

#else  // !CONFIG_COPILOT_PERF_ENABLE

// Stub implementations when profiling is disabled
void copilot_perf_init(void) {
    // No-op
}

void copilot_perf_frame_tick(void) {
    // No-op
}

uint8_t copilot_perf_get_fps(void) {
    return 0;
}

void copilot_perf_report_now(void) {
    // No-op
}

#endif  // CONFIG_COPILOT_PERF_ENABLE
