set(ENABLE_RUST_LIB false CACHE BOOL "Enable building Rust library. Required for GNSS and ITS.")

idf_component_register(
    SRCS
        "main.c"
        "cam_tx.c"
        "cmd_sniffer.c"
        "cmd_pcap.c"
        "config.c"
        "ethernet.c"
        "events.c"
        "gnss.cpp"
        "gnss_pps.cpp"
        "led.c"
        "mqtt.cpp"
        "ota.c"
        "sdcard.c"
        "spi.c"
        "temperature.c"
        "tx_custom.c"
        "tx_test.c"
    PRIV_REQUIRES
        app_trace
        app_update
        console
        esp_driver_mcpwm
        esp_eth
        esp_https_ota
        esp_netif
        esp_wifi
        fatfs
        nvs_flash
        c-cits
    INCLUDE_DIRS
        "."
)

target_compile_options(${COMPONENT_TARGET}
    PRIVATE
        -fstack-reuse=all
        -Wno-unused-function
        -Wno-deprecated-declarations
        -Wno-missing-field-initializers
        -Wno-parentheses
        -Werror=write-strings
)

if(${ENABLE_RUST_LIB})
    target_compile_definitions(${COMPONENT_TARGET}
        PRIVATE
            -DENABLE_RUST_LIB=1
    )
endif()

