#include "sdkconfig.h"

#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "esp_timer.h"

#include "RustApi.h"

#include "gnss.h"
#include "tx_custom.h"

#include "cam_tx.h"

static esp_timer_handle_t tx_timer;

ItsSState *its_s_state;

static const uint8_t mac[6] = { 0x02, 0xca, 0xfe, 0x12, 0x34, 0x56 };

static void test_tx_cam(void *)
{
    // As long as the time is synced via GNSS, the system time should be accurate within tens of microseconds.
    // This is by far good enough for C-ITS, which has millisecond timestamps.
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    int64_t time_ms = tv_now.tv_sec * 1000ll + tv_now.tv_usec / 1000;

    if (time_ms < 1781430642000ll)
        return;

    size_t cam_size;
    uint8_t *cam = build_cam(its_s_state, get_gnss_state(), time_ms, &cam_size);

    if (cam == NULL)
    {
        return;
    }

    uint8_t wlan_header[] = {
        0x88, 0x00,
        0x00, 0x00,
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
        0x00, 0x00,
        0x00, 0x25,
        0xaa, 0xaa, 0x03,
        0x00, 0x00, 0x00,
        0x89, 0x47
    };
    cam = realloc(cam, cam_size + sizeof(wlan_header));
    memmove(cam + sizeof(wlan_header), cam, cam_size);
    memcpy(cam, wlan_header, sizeof(wlan_header));

    wifi_tx_rate_config_t config = {
        .phymode = WIFI_PHY_MODE_11A,
        .rate = WIFI_PHY_RATE_12M,
        .ersu = false,
        .dcm = false
    };
    esp_wifi_80211_tx_custom(WIFI_IF_STA, cam, cam_size + sizeof(wlan_header), true, &config, WIFI_BAND_5G, WIFI_BW20);

    free(cam);
}

void cam_tx_init()
{
    its_s_state = create_its_s(&mac, 1, true);

    esp_timer_create_args_t create_args = {
        .callback = test_tx_cam,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "tx_cam",
        .skip_unhandled_events = true
    };
    ESP_ERROR_CHECK(esp_timer_create(&create_args, &tx_timer));
    esp_timer_start_periodic(tx_timer, 1000000ull);
}
