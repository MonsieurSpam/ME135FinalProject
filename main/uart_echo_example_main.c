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
#include <math.h>
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
#include "freertos/queue.h"
#include "state_machine.h"
#include "motor_parameters.h"

#define TAG "DXL_EXAMPLE"

// UART and pin configuration
#define DXL_UART_NUM   UART_NUM_2
#define DXL_TX_PIN     GPIO_NUM_14
#define DXL_RX_PIN     GPIO_NUM_7
#define DXL_DIR_PIN    GPIO_NUM_21

// Baudrate for Dynamixel communication
#define DXL_BAUDRATE   1000000

// Maximum number of servos to control
#define MAX_SERVOS     4

// Add after other defines
#define POSITION_CONTROL_STACK_SIZE 4096
#define POSITION_CONTROL_PRIORITY 5
#define POSITION_CONTROL_PERIOD_MS 20

// Add these constants near the top with other constants
#define DEMO_POSITION_DELAY_MS 2000  // Time to hold each position

// Global variables for servo control
static uint8_t servo_ids[MAX_SERVOS] = {0};
static int found_count = 0;
static pid_controller_t pid_controllers[MAX_SERVOS];
static QueueHandle_t position_command_queue;
static dxl_servo_control_t servo_controls[MAX_SERVOS];

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
    printf("D - Run demo sequence\n");
    printf("d - Run arm demo sequence\n");
    printf("\nWaiting for commands...\n");
}

// Process a command to set a single servo target position
static void process_servo_command(const char* cmd) {
    int servo_id = 0;
    int position = 0;
    
    // Format: SxPyyy where x is servo ID (1-6) and yyy is position
    if (sscanf(cmd, "S%dP%d", &servo_id, &position) == 2) {
        if (servo_id >= 1 && servo_id <= 6 && position >= 0 && position <= 4095) {
            // Set the target position using state machine
            if (state_machine_set_target(servo_id, position)) {
                printf("OK\n");
            } else {
                printf("ERROR: Failed to set position\n");
            }
        } else {
            printf("ERROR: Invalid servo ID or position value\n");
        }
    } else {
        printf("ERROR: Invalid command format\n");
    }
}

// Process a command to set all servo target positions at once
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
        for (int i = 0; i < found_count && i < MAX_SERVOS; i++) {
            if (!state_machine_set_target(servo_ids[i], positions[i])) {
                success = false;
            }
        }
        printf(success ? "OK\n" : "ERROR: Failed to set some positions\n");
    } else {
        printf("ERROR: Expected 6 position values\n");
    }
}

// Process a command to center all servos
static void process_center_command(void) {
    bool success = true;
    for (int i = 0; i < found_count && i < MAX_SERVOS; i++) {
        uint8_t id = servo_ids[i];
        int32_t position = (id == 1) ? 2048 : 2000; // Center position
        if (!state_machine_set_target(id, position)) {
            success = false;
        }
    }
    printf(success ? "OK\n" : "ERROR: Failed to center some servos\n");
}

// Process a command to read all servo positions
static void process_read_command(void) {
    printf("Positions: ");
    
    for (int i = 1; i <= 6; i++) {
        int32_t position;
        if (state_machine_get_position(i, &position)) {
            printf("%" PRId32, position);
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

// Function declarations
static void perform_demo_movements(void);

// Function to perform demo movements
static void perform_demo_movements(void) {
    ESP_LOGI(TAG, "Starting demo sequence...");
    
    // Wait for initial positions to stabilize
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Test each joint individually with smaller movements
    for (int i = 0; i < found_count; i++) {
        uint8_t id = servo_ids[i];
        if (id > 4) continue; // Skip gripper motors
        
        ESP_LOGI(TAG, "Testing joint %d", id);
        
        // Get current position
        int32_t current_pos;
        if (!dxl_read_position(id, &current_pos)) {
            ESP_LOGE(TAG, "Failed to read position for joint %d", id);
            continue;
        }
        
        // Move slightly in positive direction (smaller movement)
        pid_controllers[i].target = current_pos + 50;
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        // Move slightly in negative direction (smaller movement)
        pid_controllers[i].target = current_pos - 50;
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        // Return to original position
        pid_controllers[i].target = current_pos;
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    
    // Coordinated arm movement with smaller ranges
    ESP_LOGI(TAG, "Performing coordinated arm movement");
    
    // Move up and to the right (smaller movements)
    for (int i = 0; i < found_count; i++) {
        uint8_t id = servo_ids[i];
        if (id > 4) continue;
        
        int32_t current_pos;
        if (!dxl_read_position(id, &current_pos)) continue;
        
        // Different offsets for each joint (reduced from previous values)
        switch (id) {
            case 1: // Base rotation
                pid_controllers[i].target = current_pos + 100;
                break;
            case 2: // Shoulder
                pid_controllers[i].target = current_pos - 75;
                break;
            case 3: // Elbow
                pid_controllers[i].target = current_pos - 50;
                break;
            case 4: // Wrist
                pid_controllers[i].target = current_pos + 25;
                break;
        }
    }
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Move up and to the left (smaller movements)
    for (int i = 0; i < found_count; i++) {
        uint8_t id = servo_ids[i];
        if (id > 4) continue;
        
        int32_t current_pos;
        if (!dxl_read_position(id, &current_pos)) continue;
        
        // Different offsets for each joint (reduced from previous values)
        switch (id) {
            case 1: // Base rotation
                pid_controllers[i].target = current_pos - 100;
                break;
            case 2: // Shoulder
                pid_controllers[i].target = current_pos - 75;
                break;
            case 3: // Elbow
                pid_controllers[i].target = current_pos - 50;
                break;
            case 4: // Wrist
                pid_controllers[i].target = current_pos + 25;
                break;
        }
    }
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Return to initial positions
    for (int i = 0; i < found_count; i++) {
        uint8_t id = servo_ids[i];
        if (id > 4) continue;
        
        switch (id) {
            case 1:
                pid_controllers[i].target = 2048;
                break;
            case 2:
            case 3:
            case 4:
                pid_controllers[i].target = 2000;
                break;
        }
    }
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    ESP_LOGI(TAG, "Demo sequence completed");
}

// Add this function before the command processing section
void run_arm_demo(void) {
    ESP_LOGI(TAG, "Starting arm demo sequence");
    
    // Position 1: Initial/Home position
    ESP_LOGI(TAG, "Moving to home position");
    state_machine_set_target(1, DXL_CENTER_POSITION);  // Base centered
    state_machine_set_target(2, MOTOR2_INITIAL_POSITION); // Shoulder back
    state_machine_set_target(3, MOTOR3_INITIAL_POSITION); // Elbow centered
    state_machine_set_target(4, MOTOR4_INITIAL_POSITION); // Wrist down
    vTaskDelay(pdMS_TO_TICKS(DEMO_POSITION_DELAY_MS));
    
    // Position 2: Arm extended forward
    ESP_LOGI(TAG, "Moving to extended position");
    state_machine_set_target(2, 2200);  // Shoulder forward
    state_machine_set_target(3, 1800);  // Elbow slightly up
    vTaskDelay(pdMS_TO_TICKS(DEMO_POSITION_DELAY_MS));
    
    // Position 3: Arm up and ready
    ESP_LOGI(TAG, "Moving to ready position");
    state_machine_set_target(2, 2000);  // Shoulder centered
    state_machine_set_target(3, 2200);  // Elbow up
    state_machine_set_target(4, 2000);  // Wrist level
    vTaskDelay(pdMS_TO_TICKS(DEMO_POSITION_DELAY_MS));
    
    // Position 4: Arm reaching down
    ESP_LOGI(TAG, "Moving to reach down position");
    state_machine_set_target(2, 2300);  // Shoulder forward
    state_machine_set_target(3, 1500);  // Elbow down
    state_machine_set_target(4, 2500);  // Wrist down
    vTaskDelay(pdMS_TO_TICKS(DEMO_POSITION_DELAY_MS));
    
    // Return to home position
    ESP_LOGI(TAG, "Returning to home position");
    state_machine_set_target(1, DXL_CENTER_POSITION);
    state_machine_set_target(2, MOTOR2_INITIAL_POSITION);
    state_machine_set_target(3, MOTOR3_INITIAL_POSITION);
    state_machine_set_target(4, MOTOR4_INITIAL_POSITION);
    ESP_LOGI(TAG, "Demo sequence complete");
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
            
        case 'D': // Run demo sequence
            printf("Starting demo sequence...\n");
            perform_demo_movements();
            printf("Demo sequence completed\n");
            break;
            
        case 'd': // Run arm demo sequence
            run_arm_demo();
            break;
            
        default:
            printf("ERROR: Unknown command '%c'\n", cmd[0]);
            break;
    }
}

// User input task
static void user_input_task(void *pvParameters)
{
    // Clear screen and show ready message
    printf("\033[2J\033[H");
    printf("\n\n=== SYSTEM READY ===\n");
    print_main_menu();
    
    // Buffer for commands
    char cmd_buffer[128];
    int cmd_pos = 0;
    
    while (1) {
        // Check for serial commands
        int ch = fgetc(stdin);
        if (ch != EOF) {
            printf("%c", ch);
            
            if (ch == '\n' || ch == '\r') {
                if (cmd_pos > 0) {
                    cmd_buffer[cmd_pos] = '\0';
                    process_command(cmd_buffer);
                    cmd_pos = 0;
                    printf("\nEnter command: ");
                }
            } else if (cmd_pos < sizeof(cmd_buffer) - 1) {
                cmd_buffer[cmd_pos++] = (char)ch;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// Dynamixel control task (combined with state machine)
static void dxl_control_task(void *pvParameters)
{
    // Initialize the Dynamixel interface
    if (!dxl_init(DXL_UART_NUM, DXL_TX_PIN, DXL_RX_PIN, DXL_DIR_PIN, DXL_BAUDRATE)) {
        ESP_LOGE(TAG, "Failed to initialize Dynamixel interface");
        vTaskDelete(NULL);
        return;
    }
    
    // Initialize the state machine
    state_machine_init();
    
    // Scan for connected servos
    uint16_t model_numbers[MAX_SERVOS] = {0};
    found_count = dxl_scan(servo_ids, model_numbers, MAX_SERVOS, MAX_SERVOS);
    
    if (found_count > 0) {
        ESP_LOGI(TAG, "Found %d servos", found_count);
        
        // Create position command queue
        position_command_queue = xQueueCreate(10, sizeof(dxl_position_command_t));
        if (position_command_queue == NULL) {
            ESP_LOGE(TAG, "Failed to create position command queue");
            vTaskDelete(NULL);
            return;
        }
        
        // Initialize each servo and its control structure
        for (int i = 0; i < found_count; i++) {
            uint8_t id = servo_ids[i];
            
            // Skip if this is a gripper motor (IDs 5-6)
            if (id > 4) continue;
            
            ESP_LOGI(TAG, "Initializing servo ID %d", id);
            
            // First disable torque
            if (!dxl_disable_torque(id)) {
                ESP_LOGE(TAG, "Failed to disable torque for servo ID %d", id);
                continue;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            
            // Set to PWM mode
            if (!dxl_set_operating_mode(id, DXL_OPERATING_MODE_PWM)) {
                ESP_LOGE(TAG, "Failed to set PWM mode for servo ID %d", id);
                continue;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            
            // Re-enable torque
            if (!dxl_enable_torque(id)) {
                ESP_LOGE(TAG, "Failed to enable torque for servo ID %d", id);
                continue;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            
            // Initialize servo control structure
            dxl_servo_control_t *control = &servo_controls[i];
            control->id = id;
            control->state = DXL_STATE_MOVING;
            
            // Set initial position target
            switch (id) {
                case 1: control->target_position = 2048; break;  // Center position
                case 2: control->target_position = 2000; break;  // Slightly forward
                case 3: control->target_position = 2000; break;  // Slightly forward
                case 4: control->target_position = 2000; break;  // Slightly forward
                default: control->target_position = 2048; break;
            }
            
            // Set PWM limits based on servo ID
            switch (id) {
                case 1: // Base rotation
                    control->max_moving_pwm = 200;
                    control->max_holding_pwm = 50;
                    break;
                case 2: // Shoulder
                    control->max_moving_pwm = -400;
                    control->max_holding_pwm = -250;
                    break;
                case 3: // Elbow
                    control->max_moving_pwm = -300;
                    control->max_holding_pwm = -200;
                    break;
                case 4: // Wrist
                    control->max_moving_pwm = 200;
                    control->max_holding_pwm = 50;
                    break;
            }
            
            // Initialize PID controller
            init_pid_controller(&pid_controllers[i], 0.2f, 0.01f, 0.005f, control->max_moving_pwm);
            pid_controllers[i].target = control->target_position;
            
            ESP_LOGI(TAG, "Servo ID %d initialized with moving PWM %d and holding PWM %d",
                     id, control->max_moving_pwm, control->max_holding_pwm);
        }
        
        // Main control loop
        while (1) {
            // Update the state machine
            state_machine_update();
            
            vTaskDelay(pdMS_TO_TICKS(20));
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
    
    // Create the user input task
    xTaskCreate(user_input_task, "user_input", 4096, NULL, 4, NULL);
}

