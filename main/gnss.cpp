#include "sdkconfig.h"

#if CONFIG_ENABLE_GNSS && ENABLE_RUST_LIB

#include <string>

#include "driver/uart.h"
#include "soc/gpio_num.h"
#include "esp_timer.h"

#include "RustApi.h"

#include "gnss.h"
#include "gnss_pps.h"

constexpr uart_port_t UART = UART_NUM_1;

constexpr gpio_num_t TX_IO = (gpio_num_t)CONFIG_GNSS_PIN_TX;
constexpr gpio_num_t RX_IO = (gpio_num_t)CONFIG_GNSS_PIN_RX;

static QueueHandle_t uart_queue;
static esp_timer_handle_t timer;

static std::string buffer;

static GnssState *gnss_state;

static void update_gnss(void *)
{
    static bool last_was_cr;

    size_t available;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART, &available));
    if (available == 0)
    {
        //ESP_LOGI("GNSS", "got nothing");
        return;
    }
    size_t old_size = buffer.size();
    buffer.resize(old_size + available);
    uart_read_bytes(UART, buffer.data() + old_size, available, 0);

    while (true)
    {
        size_t initial_pos = buffer.find_first_of("\r\n");
        size_t pos = initial_pos;
        if (pos == std::string::npos)
            break;

        if (buffer[pos] == '\n' && last_was_cr)
        {
            buffer.erase(0, 1);
            last_was_cr = false;
            continue;
        }

        if (buffer[pos] == '\r')
        {
            if (buffer.size() > pos && buffer[pos + 1] == '\n')
                pos++;
            else
                last_was_cr = true;
        }

        //if (buffer.starts_with("$GNZDA"))
        //    ESP_LOGI("GNSS", "got %.*s", (int)(pos), buffer.data());
        int res = gnss_handle_nmea(gnss_state, reinterpret_cast<const uint8_t *>(buffer.data()), initial_pos);
        ESP_LOGV("GNSS", "gnss_handle_nmea returned %d", res);

        buffer.erase(0, pos + 1);
    }
}

// Callback from rust code when the time is received
extern "C" void gnss_handle_time(int64_t gnss_time_unix_seconds)
{
    gnss_pps_push_gnss_time(gnss_time_unix_seconds);
}

extern "C" void gnss_init()
{
    ESP_ERROR_CHECK(uart_driver_install(UART, 1024, 256, 10, &uart_queue, 0));

    uart_config_t uart_config{
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(UART, &uart_config));

    //ESP_ERROR_CHECK(uart_set_pin(UART, TX_IO, RX_IO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    uart_set_pin(UART, TX_IO, RX_IO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Switch baud rate to 115200, otherwise we are too slow to reliably keep sync with PPS pin
    uart_tx_chars(UART, "$PCAS01,5*19\r\n", sizeof("$PCAS01,5*19\r\n") - 1);

    ESP_ERROR_CHECK(uart_wait_tx_done(UART, portMAX_DELAY));

    uart_config.baud_rate = 115200;
    ESP_ERROR_CHECK(uart_param_config(UART, &uart_config));

    esp_timer_create_args_t create_args{
        .callback = update_gnss,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "gnss",
        .skip_unhandled_events = true
    };
    ESP_ERROR_CHECK(esp_timer_create(&create_args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, 100000ull));

    gnss_state = gnss_new();
}

extern "C" GnssState *get_gnss_state()
{
    return gnss_state;
}

#endif
