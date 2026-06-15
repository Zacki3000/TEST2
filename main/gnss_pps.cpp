#include "sdkconfig.h"

#include <stdint.h>
#include <sys/time.h>

#include "driver/gpio.h"
#include "driver/mcpwm_cap.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "gnss_pps.h"

constexpr char TAG[] = "GNSS_PPS";

constexpr gpio_num_t PPS_IO = GPIO_NUM_4;

constexpr uint64_t RESOLUTION_HZ = 160'000'000ull;
constexpr uint64_t INTERVAL_UPPER_BOUND = RESOLUTION_HZ + RESOLUTION_HZ/100;
constexpr uint64_t INTERVAL_LOWER_BOUND = RESOLUTION_HZ - RESOLUTION_HZ/100;

// seconds to wait before assuming stable clock
constexpr uint64_t SYNC_WAIT_TIME = 10;

static mcpwm_cap_timer_handle_t cap_timer;
static mcpwm_cap_channel_handle_t pps_cap_chan;
static mcpwm_cap_channel_handle_t software_cap_chan;

static volatile uint64_t avg_second_duration;
static volatile uint32_t last_cap;
static volatile int64_t projected_gnss_time_unix_seconds;

static volatile uint32_t gnss_time_good_counter;
static int64_t integral_error;

static bool mcpwm_capture_event_handler(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *)
{
    // shouldn't happen?
    if (cap_chan != pps_cap_chan)
        return false;

    uint32_t val = edata->cap_value;

    // handles overflow correctly
    uint32_t time_delta = val - last_cap;
    last_cap = val;

    // reject second durations that are more than 1 % off (1 % is already a huge error)
    if (time_delta < INTERVAL_LOWER_BOUND || time_delta > INTERVAL_UPPER_BOUND)
    {
        ESP_EARLY_LOGD(TAG, "GNSS time delta was out of range (%u, should be %u)", time_delta, RESOLUTION_HZ);
        return false;
    }

    if (avg_second_duration == 0)
        avg_second_duration = (uint64_t)time_delta << 8;
    else
        // exponential smoothing with alpha = 1/256
        avg_second_duration = avg_second_duration - (avg_second_duration >> 8) + time_delta;

    projected_gnss_time_unix_seconds += 1;
    ESP_EARLY_LOGD(TAG, "sec now %lld", projected_gnss_time_unix_seconds);

    // high prio task woken
    return false;
}

void gnss_pps_adjust_system_time(void *)
{
    if (gnss_time_good_counter < SYNC_WAIT_TIME)
    {
        ESP_LOGD(TAG, "GNSS time has been in sync for less than 10 seconds. Not adjusting system clock.");
        return;
    }

    uint32_t last_cap_local;
    uint64_t avg_second_duration_local;
    uint64_t projected_gnss_time_unix_seconds_local;
    uint32_t time_since_hw_cap;
    do {
        last_cap_local = last_cap;
        avg_second_duration_local = avg_second_duration;
        projected_gnss_time_unix_seconds_local = projected_gnss_time_unix_seconds;

        ESP_ERROR_CHECK(mcpwm_capture_channel_trigger_soft_catch(software_cap_chan));
        uint32_t cap;
        ESP_ERROR_CHECK(mcpwm_capture_get_latched_value(software_cap_chan, &cap));

        time_since_hw_cap = cap - last_cap_local;

        // We may have been interrupted by the capture IRQ. Just try again in this case.
        // Luckily, the ESP32-C5 only has one core, which means we cannot run concurrently with the IRQ handler.
    } while (last_cap_local != last_cap);


    // We did not benchmark exactly how long the call takes, and the duration may vary due to interrupt handling etc.
    // thus, we will incur an unknown offset to the actual time. There is probably no easy solution to this.
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);

    int64_t systemtime_usec = tv_now.tv_sec * 1'000'000ll + tv_now.tv_usec;
    int64_t gnsstime_usec = projected_gnss_time_unix_seconds_local * 1'000'000ll + (int64_t)time_since_hw_cap * 1'000'000ll / (avg_second_duration_local >> 8);
    int64_t diff = gnsstime_usec - systemtime_usec;

    ESP_LOGD(TAG, "stu %lld", systemtime_usec);
    ESP_LOGD(TAG, "gtu %lld", gnsstime_usec);
    ESP_LOGD(TAG, "diff %lld", diff);

    if (diff > 3'600'000'000ll || diff < -3'600'000'000)
    {
        struct timeval tv_gnss = {
            .tv_sec = gnsstime_usec / 1'000'000ll,
            .tv_usec = (int32_t)(gnsstime_usec % 1'000'000ll)
        };
        settimeofday(&tv_gnss, NULL);

        integral_error = 0;
    }
    else
    {
        uint64_t old_integral_error = integral_error >> 5;
        if (diff >= -100ll && diff <= 100ll && gnss_time_good_counter != SYNC_WAIT_TIME)
            integral_error += diff;

        //ESP_LOGE(TAG, "newie %lld", integral_error);

        diff += old_integral_error;

        struct timeval tv_delta = {
            .tv_sec = diff / 1'000'000ll,
            .tv_usec = (int32_t)(diff % 1'000'000ll)
        };
        adjtime(&tv_delta, NULL);
    }
}

esp_timer_handle_t systo_timer_handle;

void gnss_pps_push_gnss_time(int64_t gnss_time_unix_seconds)
{
    uint32_t last_cap_local = last_cap;

    if (projected_gnss_time_unix_seconds != gnss_time_unix_seconds)
    {
        if (projected_gnss_time_unix_seconds != 0)
            ESP_LOGW(TAG, "GNSS time got out of sync, %lld != %lld", projected_gnss_time_unix_seconds, gnss_time_unix_seconds);
        projected_gnss_time_unix_seconds = gnss_time_unix_seconds;
        gnss_time_good_counter = 0;
    }
    else
    {
        if (gnss_time_good_counter < SYNC_WAIT_TIME + 1)
            gnss_time_good_counter += 1;

        if (gnss_time_good_counter == SYNC_WAIT_TIME)
            ESP_LOGI(TAG, "GNSS time good");
    }

    if (last_cap_local != last_cap)
    {
        // This should not happen
        gnss_time_good_counter = 0;
        ESP_LOGW(TAG, "gnss_pps_push_gnss_time was interrupted by PPS IRQ");
    }
}

extern "C" void gnss_pps_init()
{
    gpio_config_t pps_io_config{
        .pin_bit_mask = (1ull << (int)PPS_IO),
        .mode = GPIO_MODE_INPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&pps_io_config));

    // timer should overflow every 26 seconds (2^32/(160 MHz))
    mcpwm_capture_timer_config_t timer_conf{
        .group_id = 0,
        .resolution_hz = RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&timer_conf, &cap_timer));

    mcpwm_capture_channel_config_t chan_conf{
        .gpio_num = PPS_IO,
        .prescale = 1,
        .flags = { .pos_edge = 1 },
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &chan_conf, &pps_cap_chan));

    chan_conf.gpio_num = 0;
    chan_conf.flags.pos_edge = 0;
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &chan_conf, &software_cap_chan));

    mcpwm_capture_event_callbacks_t cb{
        .on_cap = mcpwm_capture_event_handler
    };
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(pps_cap_chan, &cb, NULL));

    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(pps_cap_chan));
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(software_cap_chan));

    esp_timer_create_args_t create_args = {
        .callback = gnss_pps_adjust_system_time,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "gnss_systo",
        .skip_unhandled_events = true
    };
    ESP_ERROR_CHECK(esp_timer_create(&create_args, &systo_timer_handle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(systo_timer_handle, 1'000'000ull));
}
