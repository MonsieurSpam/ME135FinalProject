# Create custom Dynamixel SDK implementation for ESP32
set(DYNAMIXEL_SDK_SRCS
    "src/dynamixel_sdk/port_handler_esp32.c"
)

# Register component
idf_component_register(SRCS "uart_echo_example_main.c" "dynamixel.c"
                    INCLUDE_DIRS ".")

