/**
 * @file uart_echo_example_main.c
 * @brief Example application using the Dynamixel library
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <sys/time.h>
#include <sys/select.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "driver/uart_vfs.h"
#include "linenoise/linenoise.h"
#include "dynamixel.h"

#define TAG "DXL_EXAMPLE"

// UART and pin configuration
#define DXL_UART_NUM   UART_NUM_2
#define DXL_TX_PIN     GPIO_NUM_14
#define DXL_RX_PIN     GPIO_NUM_7
#define DXL_DIR_PIN    GPIO_NUM_21

// Baudrate for Dynamixel communication
#define DXL_BAUDRATE   1000000

// Maximum number of servos to control
#define MAX_SERVOS     6

// Function to read character input from console in non-blocking mode
static int read_key_from_uart(void)
{
    // Check if input is available
    fd_set rfds;
    struct timeval tv = {
        .tv_sec = 0,
        .tv_usec = 0,
    };
    
    FD_ZERO(&rfds);
    FD_SET(fileno(stdin), &rfds);
    
    int ret = select(fileno(stdin) + 1, &rfds, NULL, NULL, &tv);
    if (ret > 0) {
        return fgetc(stdin);
    }
    
    return -1; // No data available
}

// Initialize console for input
static void init_console(void)
{
    // Initialize VFS & UART
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    
    // Install UART driver for console I/O
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // Use UART0 (console UART)
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    
    // Redirect stdin/stdout to UART
    uart_vfs_dev_use_driver(UART_NUM_0);

    // Initialize the console
    esp_console_config_t console_config = {
        .max_cmdline_length = 256,
        .max_cmdline_args = 8,
        .hint_color = 36  // Cyan color (ANSI color code)
    };
    ESP_ERROR_CHECK(esp_console_init(&console_config));

    // Configure linenoise
    linenoiseSetMultiLine(1);
    linenoiseSetCompletionCallback(NULL);
    linenoiseSetHintsCallback(NULL);
    linenoiseHistorySetMaxLen(100);
}

// Dynamixel control task
static void dxl_control_task(void *pvParameters)
{
    // Initialize the Dynamixel interface
    if (!dxl_init(DXL_UART_NUM, DXL_TX_PIN, DXL_RX_PIN, DXL_DIR_PIN, DXL_BAUDRATE)) {
        ESP_LOGE(TAG, "Failed to initialize Dynamixel interface");
        vTaskDelete(NULL);
        return;
    }
    
    // Scan for connected servos
    uint8_t servo_ids[MAX_SERVOS] = {0};
    uint16_t model_numbers[MAX_SERVOS] = {0};
    int found_count = dxl_scan(servo_ids, model_numbers, MAX_SERVOS, MAX_SERVOS);
    
    if (found_count > 0) {
        ESP_LOGI(TAG, "Found %d servos", found_count);
        
        // Initialize each servo
        for (int i = 0; i < found_count; i++) {
            uint8_t id = servo_ids[i];
            
            // Set to position control mode
            if (dxl_set_operating_mode(id, DXL_OPERATING_MODE_POSITION)) {
                // Enable torque
                if (dxl_enable_torque(id)) {
                    ESP_LOGI(TAG, "Servo ID %d initialized successfully", id);
                } else {
                    ESP_LOGE(TAG, "Failed to enable torque for servo ID %d", id);
                }
            } else {
                ESP_LOGE(TAG, "Failed to set operating mode for servo ID %d", id);
            }
        }
        
        // Move all servos to center position
        for (int i = 0; i < found_count; i++) {
            dxl_set_position(servo_ids[i], DXL_CENTER_POSITION);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        
        ESP_LOGI(TAG, "All servos moved to center position");
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // Small movement test
        const uint32_t positions[] = {
            DXL_CENTER_POSITION - 50,
            DXL_CENTER_POSITION + 50,
            DXL_CENTER_POSITION
        };
        
        ESP_LOGI(TAG, "Testing small movements");
        for (int pos_idx = 0; pos_idx < 3; pos_idx++) {
            ESP_LOGI(TAG, "Movement step %d: Position %d", pos_idx + 1, (int)positions[pos_idx]);
            
            for (int i = 0; i < found_count; i++) {
                dxl_set_position(servo_ids[i], positions[pos_idx]);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        
        ESP_LOGI(TAG, "Movement test complete");
        
        // Start interactive control
        printf("\n\n===== Dynamixel Servo Control =====\n");
        printf("Use the following keys to control servos:\n");
        printf("W: Move all servos slightly up/left (-50)\n");
        printf("S: Move all servos slightly down/right (+50)\n");
        printf("C: Center all servos\n");
        printf("1-%d: Select a servo (0 for all servos)\n", MAX_SERVOS);
        printf("Q: Quit\n\n");
        
        int selected_servo = 0; // 0 means all servos
        
        // Main interactive control loop
        while (1) {
            int ch = read_key_from_uart();
            if (ch != -1) {
                if (ch == 'q' || ch == 'Q') {
                    printf("Returning all servos to center and exiting\n");
                    for (int i = 0; i < found_count; i++) {
                        dxl_set_position(servo_ids[i], DXL_CENTER_POSITION);
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }
                    break;
                } else if (ch == 'w' || ch == 'W') {
                    uint32_t new_pos = DXL_CENTER_POSITION - 50;
                    printf("Moving %s to position: %" PRIu32 "\n",
                           selected_servo == 0 ? "all servos" : "selected servo",
                           new_pos);
                    
                    if (selected_servo == 0) {
                        // Move all servos
                        for (int i = 0; i < found_count; i++) {
                            dxl_set_position(servo_ids[i], new_pos);
                            vTaskDelay(pdMS_TO_TICKS(100));
                        }
                    } else {
                        // Find the selected servo
                        for (int i = 0; i < found_count; i++) {
                            if (servo_ids[i] == selected_servo) {
                                dxl_set_position(servo_ids[i], new_pos);
                                break;
                            }
                        }
                    }
                } else if (ch == 's' || ch == 'S') {
                    uint32_t new_pos = DXL_CENTER_POSITION + 50;
                    printf("Moving %s to position: %" PRIu32 "\n",
                           selected_servo == 0 ? "all servos" : "selected servo",
                           new_pos);
                    
                    if (selected_servo == 0) {
                        // Move all servos
                        for (int i = 0; i < found_count; i++) {
                            dxl_set_position(servo_ids[i], new_pos);
                            vTaskDelay(pdMS_TO_TICKS(100));
                        }
                    } else {
                        // Find the selected servo
                        for (int i = 0; i < found_count; i++) {
                            if (servo_ids[i] == selected_servo) {
                                dxl_set_position(servo_ids[i], new_pos);
                                break;
                            }
                        }
                    }
                } else if (ch == 'c' || ch == 'C') {
                    printf("Centering %s\n", selected_servo == 0 ? "all servos" : "selected servo");
                    
                    if (selected_servo == 0) {
                        // Center all servos
                        for (int i = 0; i < found_count; i++) {
                            dxl_set_position(servo_ids[i], DXL_CENTER_POSITION);
                            vTaskDelay(pdMS_TO_TICKS(100));
                        }
                    } else {
                        // Find the selected servo
                        for (int i = 0; i < found_count; i++) {
                            if (servo_ids[i] == selected_servo) {
                                dxl_set_position(servo_ids[i], DXL_CENTER_POSITION);
                                break;
                            }
                        }
                    }
                } else if (ch >= '0' && ch <= '0' + MAX_SERVOS) {
                    selected_servo = ch - '0';
                    if (selected_servo == 0) {
                        printf("Selected all servos\n");
                    } else {
                        // Check if this servo was found
                        bool servo_found = false;
                        for (int i = 0; i < found_count; i++) {
                            if (servo_ids[i] == selected_servo) {
                                servo_found = true;
                                break;
                            }
                        }
                        
                        if (servo_found) {
                            printf("Selected servo ID %d\n", selected_servo);
                        } else {
                            printf("Servo ID %d was not found during scan\n", selected_servo);
                            selected_servo = 0; // Reset to all servos
                        }
                    }
                }
            }
            
            // Small delay to prevent CPU hogging
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    } else {
        ESP_LOGE(TAG, "No servos found!");
        printf("\nNo Dynamixel servos found. Please check:\n");
        printf("1. Servo power\n");
        printf("2. Data connections\n");
        printf("3. Baudrate settings\n");
    }
    
    vTaskDelete(NULL);
}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize console for input
    init_console();
    
    ESP_LOGI(TAG, "ESP32 Dynamixel Library Example");
    ESP_LOGI(TAG, "Using UART%d: TX=%d, RX=%d, DIR=%d, Baudrate=%d", 
             DXL_UART_NUM, DXL_TX_PIN, DXL_RX_PIN, DXL_DIR_PIN, (int)DXL_BAUDRATE);
    
    // Create the Dynamixel control task
    xTaskCreate(dxl_control_task, "dxl_task", 8192, NULL, 5, NULL);
}

