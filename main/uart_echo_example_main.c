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
#include <ctype.h>
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
// Removing this function as we're focusing only on serial commands
/*
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
        int ch = fgetc(stdin);
        
        // Filter out control characters and non-printable ASCII
        if (ch < 0 || (ch < 32 && ch != '\n' && ch != '\r')) {
            return -1;
        }
        
        // Only accept valid command characters
        if (ch == 'w' || ch == 'W' || 
            ch == 's' || ch == 'S' || 
            ch == 'c' || ch == 'C' || 
            ch == 'b' || ch == 'B' || 
            ch == 'q' || ch == 'Q' || 
            (ch >= '0' && ch <= '0' + MAX_SERVOS)) {
            return ch;
        }
        
        // Ignore other characters
        return -1;
    }
    
    return -1; // No data available
}
*/

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

// Print the main menu
static void print_main_menu()
{
    printf("\n\n===== Dynamixel Servo Control =====\n");
    printf("Available commands:\n");
    printf("SxPyyy - Set servo x to position yyy (e.g., S1P2048)\n");
    printf("M2048,2048,2048,2048,2048,2048 - Move all servos to specified positions\n");
    printf("C - Center all servos\n");
    printf("R - Read all servo positions\n");
    printf("\nWaiting for commands...\n");
}

// Process a command to set a single servo position
static void process_servo_command(const char* cmd) {
    int servo_id = 0;
    int position = 0;
    
    // Format: SxPyyy where x is servo ID (1-6) and yyy is position
    if (sscanf(cmd, "S%dP%d", &servo_id, &position) == 2) {
        if (servo_id >= 1 && servo_id <= 6 && position >= 0 && position <= 4095) {
            // Set the servo position
            if (dxl_set_position(servo_id, position)) {
                printf("OK\n");
            } else {
                printf("ERROR: Failed to set servo position\n");
            }
        } else {
            printf("ERROR: Invalid servo ID or position value\n");
        }
    } else {
        printf("ERROR: Invalid command format\n");
    }
}

// Process a command to set all servo positions at once
static void process_move_all_command(const char* cmd) {
    int positions[6];
    char temp[64];
    strncpy(temp, cmd + 1, sizeof(temp) - 1); // Skip the 'M' prefix
    
    // Format: M2048,2048,2048,2048,2048,2048
    char* token = strtok(temp, ",");
    int count = 0;
    
    while (token != NULL && count < 6) {
        positions[count++] = atoi(token);
        token = strtok(NULL, ",");
    }
    
    if (count == 6) {
        bool success = true;
        for (int i = 0; i < 6; i++) {
            if (!dxl_set_position(i + 1, positions[i])) {
                success = false;
                break;
            }
        }
        
        if (success) {
            printf("OK\n");
        } else {
            printf("ERROR: Failed to set all servo positions\n");
        }
    } else {
        printf("ERROR: Expected 6 position values\n");
    }
}

// Process a command to center all servos
static void process_center_command(void) {
    bool success = true;
    
    // Set all servos to center position (2048)
    for (int i = 1; i <= 6; i++) {
        if (!dxl_set_position(i, 2048)) {
            success = false;
            break;
        }
    }
    
    if (success) {
        printf("OK\n");
    } else {
        printf("ERROR: Failed to center all servos\n");
    }
}

// Process a command to read all servo positions
static void process_read_command(void) {
    printf("Positions: ");
    
    for (int i = 1; i <= 6; i++) {
        uint32_t position = 2048; // Default/fallback value
        
        // Try to read the current position
        if (dxl_read_position(i, &position)) {
            printf("%d", (int)position);
        } else {
            printf("2048"); // Use default if read fails
        }
        
        // Add comma except for the last item
        if (i < 6) {
            printf(",");
        }
    }
    
    printf("\nOK\n");
}

// Process incoming serial commands
static void process_command(const char* cmd) {
    // Trim any whitespace
    while (isspace((unsigned char)*cmd)) {
        cmd++;
    }
    
    // Ignore empty commands
    if (strlen(cmd) == 0) {
        return;
    }
    
    // Process command based on first character
    switch (cmd[0]) {
        case 'S': // Set a single servo: S1P2048
            process_servo_command(cmd);
            break;
            
        case 'M': // Move all servos: M2048,2048,2048,2048,2048,2048
            process_move_all_command(cmd);
            break;
            
        case 'C': // Center all servos
            process_center_command();
            break;
            
        case 'R': // Read all servo positions
            process_read_command();
            break;
            
        default:
            printf("ERROR: Unknown command '%c'\n", cmd[0]);
            break;
    }
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
        vTaskDelay(pdMS_TO_TICKS(300)); // Reduced delay for stability
        
        // Flush stdin to clear any buffered input
        fflush(stdin);
        
        // Clear the screen with ANSI escape sequence
        printf("\033[2J\033[H");  // Clear screen and position cursor at top
        printf("\n\n=== SYSTEM READY ===\n");
        printf("Please wait a moment before entering commands...\n");
        vTaskDelay(pdMS_TO_TICKS(500));  // Reduced waiting time
        
        // Start command processing
        print_main_menu();
        
        // Buffer for storing incoming commands
        char cmd_buffer[128];
        int cmd_pos = 0;
        
        // Main command processing loop
        while (1) {
            // Check for serial commands
            int ch = fgetc(stdin);
            if (ch != EOF) {
                // Echo the character for better UX
                printf("%c", ch);
                
                if (ch == '\n' || ch == '\r') {
                    // End of command, process it
                    if (cmd_pos > 0) {
                        cmd_buffer[cmd_pos] = '\0';
                        printf("\nProcessing command: %s\n", cmd_buffer);
                        process_command(cmd_buffer);
                        cmd_pos = 0;
                        printf("\nEnter command: ");
                    }
                } else if (cmd_pos < sizeof(cmd_buffer) - 1) {
                    // Add character to command buffer
                    cmd_buffer[cmd_pos++] = (char)ch;
                }
            }
            
            // Delay to prevent CPU hogging - increased for monitor mode
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

