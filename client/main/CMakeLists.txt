# Set up basic source files
set(srcs 
        "sensors.c"
        "screen.c"
        "decode_jpeg.c"
        "decode_png.c"
        "pngle.c"
        "main.c")

# Get the target for conditional dependencies
idf_build_get_property(target IDF_TARGET)

# Set conditional sources and requirements based on the target and configuration
set(requires "esp_event") # Default required components

list(APPEND requires "protocol_examples_common" "nvs_flash" "esp_netif" "vfs" "spiffs" "driver")

if("${CONFIG_EXAMPLE_IPV4}" STREQUAL y)
    list(APPEND srcs "tcp_client_v4.c")
else()
    list(APPEND srcs "tcp_client_v6.c")
endif()

# Register component with merged configuration
idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "."
                       REQUIRES ${requires}
                       )
