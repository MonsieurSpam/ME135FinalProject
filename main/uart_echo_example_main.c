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
#include "freertos/queue.h"
#include "freertos/portmacro.h"
#include "freertos/portable.h"
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
#include "state_machine.h"
#include "motor_parameters.h"
#include "esp_timer.h"
#include "esp_freertos_hooks.h"  // Add this for core affinity functions

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

// Add after other defines
#define POSITION_CONTROL_STACK_SIZE 8192
#define POSITION_CONTROL_PRIORITY 3  // Lower priority, not realtime
#define POSITION_CONTROL_PERIOD_MS 100
#define USER_INPUT_STACK_SIZE 8192
#define USER_INPUT_PRIORITY 2  // Lower priority, not realtime
#define USER_INPUT_PERIOD_MS 10
#define POSITION_REPORT_STACK_SIZE 4096
#define POSITION_REPORT_PRIORITY 1  // Lower priority, not realtime
#define POSITION_REPORT_PERIOD_US 100000  // 100ms in microseconds
#define POSITION_REPORT_QUEUE_SIZE 10

// Add these constants near the top with other constants
#define DEMO_POSITION_DELAY_MS 4000  // Time to hold each position (4 seconds)
#define GRIPPER_OPEN_POSITION  2500
#define GRIPPER_CLOSED_POSITION 2048
#define GRIPPER_MAX_TORQUE     5000   // Maximum torque limit for gripper (adjust based on testing)
#define GRIPPER_CLOSING_TORQUE 4500   // Lower torque limit for closing to prevent overload
#define GRIPPER_TORQUE_CHECK_MS 50   // How often to check torque (50ms)
#define DXL_ADDR_PRESENT_CURRENT 126  // Address for present current in control table
#define DXL_ADDR_HARDWARE_ERROR_STATUS 70  // Address for hardware error status
#define DXL_ADDR_OPERATING_MODE 11    // Address for operating mode
#define DXL_CURRENT_CONTROL_MODE 0    // Current control mode
#define DXL_POSITION_CONTROL_MODE 3   // Position control mode
#define DXL_PWM_CONTROL_MODE 16      // PWM control mode
#define DXL_ADDR_GOAL_PWM 100        // Address for goal PWM
#define GRIPPER_PWM_LIMIT 885        // Maximum PWM value (about 85% of max)
#define GRIPPER_OPEN_PWM 300         // Positive PWM to open
#define GRIPPER_CLOSE_PWM -300       // Negative PWM to close
#define GRIPPER_ERROR_THRESHOLD 10    // Position error threshold for considering movement complete
#define GRIPPER_PWM_RAMP_STEPS 10    // Number of steps for PWM ramping
#define GRIPPER_PWM_RAMP_DELAY_MS 100 // Delay between PWM steps

// Add after other defines
#define PID_CONTROL_PERIOD_US 10000  // 10ms = 10000μs
#define PID_CONTROL_STACK_SIZE 4096
#define PID_CONTROL_PRIORITY (configMAX_PRIORITIES - 1)  // Highest priority

// Global variables for servo control
static uint8_t servo_ids[MAX_SERVOS] = {0};
static int found_count = 0;
static QueueHandle_t position_command_queue;
static dxl_servo_control_t servo_controls[MAX_SERVOS];

// Set logging levels
#define DYNAMIXEL_LOG_LEVEL ESP_LOG_ERROR  // Only show errors
#define STATE_MACHINE_LOG_LEVEL ESP_LOG_ERROR  // Only show errors

// Task handle for PID control
static TaskHandle_t pid_control_task_handle = NULL;

// Timer handle for PID control
static esp_timer_handle_t pid_timer_handle = NULL;

// Buffer for current positions
static int32_t current_positions[MAX_SERVOS];

// Shared position buffer with mutex protection
static struct {
    int32_t positions[MAX_SERVOS];
    portMUX_TYPE mutex;
} shared_positions = {
    .mutex = portMUX_INITIALIZER_UNLOCKED
};

// Add after other global variables
static QueueHandle_t position_report_queue = NULL;
static TaskHandle_t position_report_task_handle = NULL;

// Timer ISR - minimal, just notifies task
static void IRAM_ATTR pid_timer_isr(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(pid_control_task_handle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// PID Control Task
static void pid_control_task(void* arg) {
    // Set task priority to maximum
    vTaskPrioritySet(NULL, PID_CONTROL_PRIORITY);
    
    while (1) {
        // Wait for notification from timer ISR
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        // 1. Read positions
        for (int i = 0; i < found_count; i++) {
            if (!state_machine_get_position(servo_ids[i], &current_positions[i])) {
                ESP_LOGE(TAG, "Failed to read position for servo %d", servo_ids[i]);
                continue;
            }
        }
        
        // Update shared position buffer
        portENTER_CRITICAL(&shared_positions.mutex);
        for (int i = 0; i < found_count; i++) {
            shared_positions.positions[i] = current_positions[i];
        }
        portEXIT_CRITICAL(&shared_positions.mutex);
        
        // 2. Update state machine for PID control
        state_machine_update();
    }
}

static void init_pid_control(void) {
    // Create PID control task pinned to Core 0
    BaseType_t ret = xTaskCreatePinnedToCore(
        pid_control_task,
        "pid_control",
        PID_CONTROL_STACK_SIZE,
        NULL,
        PID_CONTROL_PRIORITY,
        &pid_control_task_handle,
        0  // Pin to Core 0
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create PID control task");
        return;
    }
    
    // Create timer for PID control
    esp_timer_create_args_t timer_args = {
        .callback = pid_timer_isr,
        .name = "pid_timer",
        .dispatch_method = ESP_TIMER_TASK
    };
    
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &pid_timer_handle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(pid_timer_handle, PID_CONTROL_PERIOD_US));
    
    ESP_LOGI(TAG, "PID control initialized with %dμs period", PID_CONTROL_PERIOD_US);
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

// Print the main menu
static void print_main_menu()
{
    // Empty - no menu printing
}

// Process a command to set a single servo target position
static void process_servo_command(const char* cmd) {
    int servo_id = 0;
    int position = 0;
    
    // Format: SxPyyy where x is servo ID (1-6) and yyy is position
    if (sscanf(cmd, "S%dP%d", &servo_id, &position) == 2) {
        if (servo_id >= 1 && servo_id <= 6 && position >= 0 && position <= 4095) {
            // Set the target position using state machine
            state_machine_set_target(servo_id, position);
        }
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
        for (int i = 0; i < found_count && i < MAX_SERVOS; i++) {
            state_machine_set_target(servo_ids[i], positions[i]);
        }
    }
}

// Process a command to center all servos
static void process_center_command(void) {
    for (int i = 0; i < found_count && i < MAX_SERVOS; i++) {
        uint8_t id = servo_ids[i];
        int32_t position;
        
        // Use initial positions from motor_parameters.h
        switch(id) {
            case 1:
                position = MOTOR1_CENTER_POSITION;  // Base
                break;
            case 2:
                position = MOTOR2_INITIAL_POSITION;  // Shoulder
                break;
            case 3:
                position = MOTOR3_INITIAL_POSITION;  // Elbow
                break;
            case 4:
                position = MOTOR4_INITIAL_POSITION;  // Wrist
                break;
            case 5:
                position = MOTOR5_INITIAL_POSITION;  // Fixed wrist
                break;
            case 6:
                position = MOTOR6_INITIAL_POSITION;  // Gripper
                break;
            default:
                position = 2048;  // Default center position for any other servos
                break;
        }
        
        state_machine_set_target(id, position);
    }
}

// Process a command to read all servo positions
static void process_read_command(void) {
    // Empty - no position reading printing
}

// Add this function before the command processing section
void run_arm_demo(void) {
    // Position 1: Initial/Home position
    state_machine_set_target(1, DXL_CENTER_POSITION);  // Base centered
    state_machine_set_target(2, MOTOR2_INITIAL_POSITION); // Shoulder back
    state_machine_set_target(3, MOTOR3_INITIAL_POSITION); // Elbow centered
    state_machine_set_target(4, MOTOR4_INITIAL_POSITION); // Wrist down
    vTaskDelay(pdMS_TO_TICKS(DEMO_POSITION_DELAY_MS));
    
    // Position 2: Arm extended forward
    state_machine_set_target(2, 2200);  // Shoulder forward
    state_machine_set_target(3, 1800);  // Elbow slightly up
    state_machine_set_target(4, 1450);  // Wrist up
    vTaskDelay(pdMS_TO_TICKS(DEMO_POSITION_DELAY_MS));
    
    // Position 3: Arm up and ready
    state_machine_set_target(2, 2000);  // Shoulder centered
    state_machine_set_target(3, 2200);  // Elbow up
    state_machine_set_target(4, 2000);  // Wrist level
    vTaskDelay(pdMS_TO_TICKS(DEMO_POSITION_DELAY_MS));
    
    // Position 4: Arm reaching down
    state_machine_set_target(2, 2300);  // Shoulder forward
    state_machine_set_target(3, 1500);  // Elbow down
    state_machine_set_target(4, 2500);  // Wrist down
    vTaskDelay(pdMS_TO_TICKS(DEMO_POSITION_DELAY_MS));
    
    // Return to home position
    state_machine_set_target(1, DXL_CENTER_POSITION);
    state_machine_set_target(2, MOTOR2_INITIAL_POSITION);
    state_machine_set_target(3, MOTOR3_INITIAL_POSITION);
    state_machine_set_target(4, MOTOR4_INITIAL_POSITION);
}

// Process IK command
static void process_ik_command(const char* cmd) {
    // Skip the 'I' prefix
    cmd++;
    
    // Split the command into individual angle sets
    char* angle_sets[20];  // Maximum 20 sets of angles
    int num_sets = 0;
    char* token = strtok((char*)cmd, ";");
    
    while (token != NULL && num_sets < 20) {
        angle_sets[num_sets++] = token;
        token = strtok(NULL, ";");
    }
    
    if (num_sets == 0) {
        return;
    }
    
    // Process each set of angles
    for (int set_idx = 0; set_idx < num_sets; set_idx++) {
        float angles[4];  // Array to store joint angles (only 4 joints)
        int num_angles = sscanf(angle_sets[set_idx], "%f,%f,%f,%f",
                              &angles[0], &angles[1], &angles[2], &angles[3]);
        
        if (num_angles != 4) {  // We need exactly 4 angles
            continue;
        }
        
        // Convert angles to positions and move servos
        int positions[4];
        
        // Standard Dynamixel mapping for all joints:
        // 180° -> 2048 (center)
        // 90° -> 1024 (90° clockwise)
        // 270° -> 3072 (90° counterclockwise)
        for (int i = 0; i < 4; i++) {
            positions[i] = 2048 + (int)((angles[i] - 180.0f) * (1024.0f / 90.0f));
            
            // Send command to servo
            char servo_cmd[16];
            snprintf(servo_cmd, sizeof(servo_cmd), "S%dP%d", i+1, positions[i]);
            process_servo_command(servo_cmd);
        }
        
        // Wait for servos to reach position
        vTaskDelay(pdMS_TO_TICKS(1000));  // 1 second delay between sets
    }
}

// Add this function before the process_gripper_open_command
static bool recover_from_hardware_error(uint8_t servo_id) {
    ESP_LOGW(TAG, "Attempting to recover from hardware error for servo %d", servo_id);
    
    // Disable torque
    if (!dxl_disable_torque(servo_id)) {
        ESP_LOGE(TAG, "Failed to disable torque during recovery");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Clear hardware error
    if (!dxl_write_register(servo_id, DXL_ADDR_HARDWARE_ERROR_STATUS, 1, 0)) {
        ESP_LOGE(TAG, "Failed to clear hardware error");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Set PWM to 0
    if (!dxl_write_register(servo_id, DXL_ADDR_GOAL_PWM, 2, 0)) {
        ESP_LOGE(TAG, "Failed to set PWM to 0 during recovery");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Re-enable torque
    if (!dxl_enable_torque(servo_id)) {
        ESP_LOGE(TAG, "Failed to re-enable torque during recovery");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(200));
    
    ESP_LOGI(TAG, "Recovery completed for servo %d", servo_id);
    return true;
}

// Update set_pwm_gradually function
static bool set_pwm_gradually(int8_t servo_id, int32_t target_pwm) {
    int32_t current_pwm = 0;
    int32_t step = target_pwm / GRIPPER_PWM_RAMP_STEPS;
    
    // First ensure we start from 0
    if (!dxl_write_register(servo_id, DXL_ADDR_GOAL_PWM, 2, 0)) {
        ESP_LOGE(TAG, "Failed to set initial PWM to 0");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(200));
    
    for (int i = 0; i < GRIPPER_PWM_RAMP_STEPS; i++) {
        current_pwm += step;
        
        // Check for hardware errors before setting PWM
        uint32_t error_status;
        if (dxl_read_register(servo_id, DXL_ADDR_HARDWARE_ERROR_STATUS, 1, &error_status)) {
            if (error_status != 0) {
                ESP_LOGW(TAG, "Hardware error detected during PWM ramp: 0x%02lx", error_status);
                if (!recover_from_hardware_error(servo_id)) {
                    return false;
                }
            }
        }
        
        if (!dxl_write_register(servo_id, DXL_ADDR_GOAL_PWM, 2, current_pwm)) {
            ESP_LOGE(TAG, "Failed to set PWM during ramp");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(GRIPPER_PWM_RAMP_DELAY_MS));
    }
    
    // Ensure we reach exactly the target PWM
    if (!dxl_write_register(servo_id, DXL_ADDR_GOAL_PWM, 2, target_pwm)) {
        ESP_LOGE(TAG, "Failed to set final PWM");
        return false;
    }
    
    return true;
}

// Process a command to open the gripper
static void process_gripper_open_command(void) {
    // Set PWM to open gripper
    dxl_write_register(6, DXL_ADDR_GOAL_PWM, 2, GRIPPER_OPEN_PWM);
}

// Process a command to close the gripper
static void process_gripper_close_command(void) {
    // Set PWM to close gripper
    dxl_write_register(6, DXL_ADDR_GOAL_PWM, 2, GRIPPER_CLOSE_PWM);
}

// Update the process_command function
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
            
        case 'd': // Run arm demo sequence
            run_arm_demo();
            break;
            
        case 'I': // Move to joint angles
            process_ik_command(cmd);
            break;
            
        case 'O': // Open gripper
            process_gripper_open_command();
            break;
            
        case 'G': // Close gripper
            process_gripper_close_command();
            break;
            
        default:
            printf("ERROR: Unknown command '%c'\n", cmd[0]);
            break;
    }
}

// User input task
static void user_input_task(void *pvParameters)
{
    // Buffer for commands
    char cmd_buffer[128];
    int cmd_pos = 0;
    
    while (1) {
        // Check for serial commands
        int ch = fgetc(stdin);
        if (ch != EOF) {
            if (ch == '\n' || ch == '\r') {
                if (cmd_pos > 0) {
                    cmd_buffer[cmd_pos] = '\0';
                    process_command(cmd_buffer);
                    cmd_pos = 0;
                }
            } else if (cmd_pos < sizeof(cmd_buffer) - 1) {
                cmd_buffer[cmd_pos++] = (char)ch;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(USER_INPUT_PERIOD_MS));
    }
}

// Add this function declaration before dxl_control_task
static void initialize_gripper(void);

// Add this function to initialize the gripper
static void initialize_gripper(void) {
    // First disable torque
    if (!dxl_disable_torque(6)) {
        ESP_LOGE(TAG, "Failed to disable torque during initialization");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // Set PWM to 0
    if (!dxl_write_register(6, DXL_ADDR_GOAL_PWM, 2, 0)) {
        ESP_LOGE(TAG, "Failed to set PWM to 0 during initialization");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // Enable torque
    if (!dxl_enable_torque(6)) {
        ESP_LOGE(TAG, "Failed to enable torque during initialization");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // Set gripper to closed position
    if (!dxl_write_register(6, DXL_ADDR_GOAL_PWM, 2, GRIPPER_CLOSE_PWM)) {
        ESP_LOGE(TAG, "Failed to set initial PWM for gripper");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(500)); // Give it time to close

    ESP_LOGI(TAG, "Gripper initialized to closed position");
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
        
        // Initialize gripper first
        initialize_gripper();
        
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
            
            ESP_LOGI(TAG, "Initializing servo ID %d", id);
            
            // First disable torque
            if (!dxl_disable_torque(id)) {
                ESP_LOGE(TAG, "Failed to disable torque for servo ID %d", id);
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
                case 1: control->target_position = MOTOR1_CENTER_POSITION; break;  // Center position
                case 2: control->target_position = MOTOR2_INITIAL_POSITION; break;  // Slightly forward
                case 3: control->target_position = MOTOR3_INITIAL_POSITION; break;  // Slightly forward
                case 4: control->target_position = MOTOR4_INITIAL_POSITION; break;  // Slightly forward
                case 5: control->target_position = MOTOR5_INITIAL_POSITION; break;  // Fixed wrist position
                case 6: control->target_position = MOTOR6_INITIAL_POSITION; break;  // Open gripper
                default: control->target_position = 2048; break;
            }
            
            ESP_LOGI(TAG, "Servo ID %d initialized", id);
        }
        
        // Main control loop
        while (1) {
            // Update the state machine
            state_machine_update();
            
            vTaskDelay(pdMS_TO_TICKS(POSITION_CONTROL_PERIOD_MS));
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

// Position reporting task
static void position_report_task(void* arg) {
    int32_t positions[MAX_SERVOS];
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(100); // 100ms period
    
    while (1) {
        // Copy positions from shared buffer
        portENTER_CRITICAL(&shared_positions.mutex);
        for (int i = 0; i < found_count; i++) {
            positions[i] = shared_positions.positions[i];
        }
        portEXIT_CRITICAL(&shared_positions.mutex);
        
        // Print positions regardless of queue state
        printf("J");  // 'J' prefix for joint positions
        for (int i = 0; i < found_count; i++) {
            printf("%" PRId32, positions[i]);
            if (i < found_count - 1) {
                printf(",");
            }
        }
        printf("\n");
        
        // Ensure consistent timing
        vTaskDelayUntil(&last_wake_time, frequency);
    }
}

// Update position reporting callback to be simpler
static void position_report_callback(void* arg) {
    // Just notify the task - actual position reading is done in the task
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(position_report_task_handle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

static void init_position_reporting(void)
{
    // Create position report task (no core pinning)
    BaseType_t ret = xTaskCreate(
        position_report_task,
        "pos_report",
        POSITION_REPORT_STACK_SIZE,
        NULL,
        POSITION_REPORT_PRIORITY,
        &position_report_task_handle  // Store the handle
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create position report task");
        return;
    }
    
    // Create timer for position reporting
    esp_timer_handle_t timer_handle;
    esp_timer_create_args_t timer_args = {
        .callback = position_report_callback,
        .name = "position_report",
        .dispatch_method = ESP_TIMER_TASK
    };
    
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_handle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handle, POSITION_REPORT_PERIOD_US));
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
    
    // Set logging levels
    esp_log_level_set("DYNAMIXEL", DYNAMIXEL_LOG_LEVEL);
    esp_log_level_set("STATE_MACHINE", STATE_MACHINE_LOG_LEVEL);
    
    ESP_LOGI(TAG, "ESP32 Dynamixel Library Example");
    ESP_LOGI(TAG, "Using UART%d: TX=%d, RX=%d, DIR=%d, Baudrate=%d", 
             DXL_UART_NUM, DXL_TX_PIN, DXL_RX_PIN, DXL_DIR_PIN, (int)DXL_BAUDRATE);
    
    // Create tasks with adjusted priorities (no core pinning for UART tasks)
    xTaskCreate(user_input_task, "user_input", USER_INPUT_STACK_SIZE, NULL, USER_INPUT_PRIORITY, NULL);
    xTaskCreate(dxl_control_task, "dxl_control", POSITION_CONTROL_STACK_SIZE, NULL, POSITION_CONTROL_PRIORITY, NULL);
    
    // Initialize position reporting using esp_timer
    init_position_reporting();

    // Initialize PID control (this remains realtime)
    init_pid_control();
}

