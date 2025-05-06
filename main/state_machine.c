#include <string.h>
#include <math.h>
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "state_machine.h"
#include "dynamixel.h"
#include "freertos/queue.h"
#include "motor_parameters.h"

#define TAG "STATE_MACHINE"

// Constants
#define UPDATE_PERIOD_MS 50
#define POSITION_ERROR_THRESHOLD 20
#define HOLDING_ERROR_THRESHOLD 100
#define MAX_SERVOS 6  // Updated to support 6 motors
#define DIRECTION_CHANGE_THRESHOLD 50
#define DEADBAND_THRESHOLD 10
#define HOLDING_STATE_THRESHOLD 15
#define MAX_VELOCITY 265  // Maximum velocity value for Dynamixel
#define MIN_VELOCITY -265 // Minimum velocity value
#define HOLDING_VELOCITY 0 // Velocity when holding position
#define MOTOR2_FORWARD_HOLDING_PWM 600  // Higher holding PWM for forward positions
#define MOTOR2_FORWARD_THRESHOLD 2100   // Position threshold where we consider it "forward"
#define MOTOR3_FORWARD_HOLDING_PWM 600  // Higher holding PWM for forward positions
#define MOTOR3_FORWARD_THRESHOLD 1700   // Position threshold where we consider it "forward"
#define MOTOR2_ELBOW_UP_HOLDING_PWM 800  // Even higher holding PWM when elbow is up
#define MOTOR3_ELBOW_UP_THRESHOLD 1900   // Position where elbow is considered "up"
#define DXL_ADDR_OPERATING_MODE 11    // Address for operating mode
#define DXL_PWM_CONTROL_MODE 16      // PWM control mode
#define DXL_ADDR_GOAL_PWM 100        // Address for goal PWM
#define GRIPPER_PWM_LIMIT 885        // Maximum PWM value (about 85% of max)
#define GRIPPER_ERROR_THRESHOLD 10
#define GRIPPER_OPEN_PWM 300         // Positive PWM to open
#define GRIPPER_CLOSE_PWM -300       // Negative PWM to close
#define GRIPPER_HOLD_OPEN_PWM 30     // Small positive PWM to hold open position
#define GRIPPER_HOLD_CLOSE_PWM -30   // Small negative PWM to hold closed position

// Global variables
static dxl_servo_control_t servo_controls[MAX_SERVOS];
static uint8_t found_count = 0;
static pid_controller_t pid_controllers[MAX_SERVOS];
static QueueHandle_t position_command_queue;

// Initialize PID controller
void init_pid_controller(pid_controller_t* pid, float kp, float ki, float kd, float max_current) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->target = 0.0f;
    pid->current = 0.0f;
    pid->max_current = max_current;
    pid->integral_limit = max_current / ki; // Limit integral term to prevent windup
}

// PID controller with anti-windup
static float pid_controller(pid_controller_t *pid, float error)
{
    // Proportional term
    float p_term = pid->kp * error;
    
    // Integral term with anti-windup
    pid->integral += error;
    
    // Limit integral term to prevent windup
    float max_integral = 100.0f; // Adjust this value based on your needs
    if (pid->integral > max_integral) {
        pid->integral = max_integral;
    } else if (pid->integral < -max_integral) {
        pid->integral = -max_integral;
    }
    
    // Reset integral if error changes sign (prevents windup during direction changes)
    if (error * pid->prev_error < 0) {
        pid->integral = 0;
    }
    
    float i_term = pid->ki * pid->integral;
    
    // Derivative term
    float d_term = pid->kd * (error - pid->prev_error);
    pid->prev_error = error;
    
    // Calculate total output
    float output = p_term + i_term + d_term;
    
    // Limit output to maximum velocity
    if (output > MAX_VELOCITY) {
        output = MAX_VELOCITY;
    } else if (output < MIN_VELOCITY) {
        output = MIN_VELOCITY;
    }
    
    return output;
}

// Helper function to find servo control by ID
static dxl_servo_control_t* find_servo_control(uint8_t id) {
    for (int i = 0; i < found_count; i++) {
        if (servo_controls[i].id == id) {
            return &servo_controls[i];
        }
    }
    return NULL;
}

// Initialize the state machine
void state_machine_init(void) {
    // Get list of connected servos
    uint8_t servo_ids[MAX_SERVOS];
    uint16_t model_numbers[MAX_SERVOS];
    found_count = dxl_scan(servo_ids, model_numbers, MAX_SERVOS, MAX_SERVOS);
    
    if (found_count == 0) {
        ESP_LOGE(TAG, "No servos found!");
        return;
    }
    
    ESP_LOGI(TAG, "Found %d servos", found_count);
    
    // Log all found servo IDs
    for (int i = 0; i < found_count; i++) {
        ESP_LOGI(TAG, "Found servo ID: %d", servo_ids[i]);
    }
    
    // Initialize all motors (1-6)
    for (int i = 0; i < found_count; i++) {
        uint8_t id = servo_ids[i];
        
        // Only process motors 1-6
        if (id < 1 || id > 6) continue;
        
        dxl_servo_control_t *control = &servo_controls[i];
        memset(control, 0, sizeof(dxl_servo_control_t));
        
        control->id = id;
        control->state = DXL_STATE_MOVING;
        
        // Set initial position and velocity limits based on motor ID
        switch (id) {
            case 1: // Base rotation
                control->target_position = MOTOR1_CENTER_POSITION;
                init_pid_controller(&pid_controllers[i], MOTOR1_KP, MOTOR1_KI, MOTOR1_KD, MOTOR1_MAX_VELOCITY);
                break;
            case 2: // Shoulder
                control->target_position = MOTOR2_INITIAL_POSITION;
                init_pid_controller(&pid_controllers[i], MOTOR2_KP, MOTOR2_KI, MOTOR2_KD, MOTOR2_MAX_VELOCITY);
                break;
            case 3: // Elbow
                control->target_position = MOTOR3_INITIAL_POSITION;
                init_pid_controller(&pid_controllers[i], MOTOR3_KP, MOTOR3_KI, MOTOR3_KD, MOTOR3_MAX_VELOCITY);
                break;
            case 4: // Wrist
                control->target_position = MOTOR4_INITIAL_POSITION;
                init_pid_controller(&pid_controllers[i], MOTOR4_KP, MOTOR4_KI, MOTOR4_KD, MOTOR4_MAX_VELOCITY);
                break;
            case 5: // Fixed wrist
                control->target_position = MOTOR5_INITIAL_POSITION;
                init_pid_controller(&pid_controllers[i], MOTOR5_KP, MOTOR5_KI, MOTOR5_KD, MOTOR5_MAX_VELOCITY);
                break;
            case 6: // Gripper
                control->target_position = MOTOR6_INITIAL_POSITION;
                // For gripper, switch to PWM control mode
                if (!dxl_write_register(6, DXL_ADDR_OPERATING_MODE, 1, DXL_PWM_CONTROL_MODE)) {
                    ESP_LOGE(TAG, "Failed to switch gripper to PWM control mode");
                    continue;
                }
                vTaskDelay(pdMS_TO_TICKS(100));
                // Set initial PWM to 0
                if (!dxl_write_register(6, DXL_ADDR_GOAL_PWM, 2, 0)) {
                    ESP_LOGE(TAG, "Failed to set initial PWM for gripper");
                    continue;
                }
                break;
        }
        
        // Only set PID target for non-gripper motors
        if (id != 6) {
            pid_controllers[i].target = control->target_position;
        }
        
        ESP_LOGI(TAG, "Motor %d initialized with max velocity %" PRId32,
                 id, (int32_t)pid_controllers[i].max_current);
    }

    // Initialize position command queue
    position_command_queue = xQueueCreate(10, sizeof(dxl_position_command_t));
}

// Update the state machine
void state_machine_update(void) {
    for (int i = 0; i < found_count; i++) {
        if (servo_controls[i].id == 0) continue; // Skip unused slots
        
        // Read current position
        int32_t current_pos;
        if (!dxl_read_position(servo_controls[i].id, &current_pos)) {
            ESP_LOGE(TAG, "Failed to read position for servo ID %d", servo_controls[i].id);
            continue;
        }
        servo_controls[i].current_position = current_pos;
        
        // Special handling for gripper (motor 6) using PWM control
        if (servo_controls[i].id == 6) {
            // Skip motor 6 in state machine - it's controlled directly by PWM commands
            continue;
        }
        
        // For other motors, use PID control
        // Calculate error
        int32_t target_pos = servo_controls[i].target_position;
        float error = target_pos - current_pos;
        
        // Calculate PID output
        float pid_output = pid_controller(&pid_controllers[i], error);
        
        // Convert PID output to velocity
        int32_t velocity = (int32_t)pid_output;
        
        // Limit velocity based on motor parameters
        switch (servo_controls[i].id) {
            case 1: velocity = (velocity > MOTOR1_MAX_VELOCITY) ? MOTOR1_MAX_VELOCITY : 
                             (velocity < -MOTOR1_MAX_VELOCITY) ? -MOTOR1_MAX_VELOCITY : velocity; break;
            case 2: velocity = (velocity > MOTOR2_MAX_VELOCITY) ? MOTOR2_MAX_VELOCITY : 
                             (velocity < -MOTOR2_MAX_VELOCITY) ? -MOTOR2_MAX_VELOCITY : velocity; break;
            case 3: velocity = (velocity > MOTOR3_MAX_VELOCITY) ? MOTOR3_MAX_VELOCITY : 
                             (velocity < -MOTOR3_MAX_VELOCITY) ? -MOTOR3_MAX_VELOCITY : velocity; break;
            case 4: velocity = (velocity > MOTOR4_MAX_VELOCITY) ? MOTOR4_MAX_VELOCITY : 
                             (velocity < -MOTOR4_MAX_VELOCITY) ? -MOTOR4_MAX_VELOCITY : velocity; break;
            case 5: velocity = (velocity > MOTOR5_MAX_VELOCITY) ? MOTOR5_MAX_VELOCITY : 
                             (velocity < -MOTOR5_MAX_VELOCITY) ? -MOTOR5_MAX_VELOCITY : velocity; break;
        }
        
        // Set velocity based on state
        if (servo_controls[i].state == DXL_STATE_HOLDING) {
            velocity = 0;  // Stop when holding position
        }
        
        // Set the velocity
        if (!dxl_set_velocity(servo_controls[i].id, velocity)) {
            ESP_LOGE(TAG, "Failed to set velocity for servo ID %d", servo_controls[i].id);
        }
        
        servo_controls[i].current_velocity = velocity;
        ESP_LOGI(TAG, "Servo %d: Set velocity to %" PRId32, servo_controls[i].id, velocity);
        
        // Update state based on error
        if (fabsf(error) < 15) {  // Use fabsf for float comparison
            if (servo_controls[i].state == DXL_STATE_MOVING) {
                servo_controls[i].state = DXL_STATE_HOLDING;
                ESP_LOGI(TAG, "Servo %d: Transitioned to HOLDING state", servo_controls[i].id);
            }
        } else {
            if (servo_controls[i].state == DXL_STATE_HOLDING) {
                servo_controls[i].state = DXL_STATE_MOVING;
                ESP_LOGI(TAG, "Servo %d: Transitioned to MOVING state", servo_controls[i].id);
            }
        }
    }
}

// Set target position for a servo
bool state_machine_set_target(uint8_t id, int32_t position) {
    dxl_servo_control_t *control = find_servo_control(id);
    if (!control) {
        ESP_LOGE(TAG, "Invalid servo ID: %d", id);
        return false;
    }
    
    control->target_position = position;
    pid_controllers[id - 1].target = position;
    control->state = DXL_STATE_MOVING;
    
    ESP_LOGI(TAG, "Set target position for servo ID %d to %" PRId32, id, position);
    return true;
}

// Get current state of a servo
dxl_state_t state_machine_get_state(uint8_t id) {
    dxl_servo_control_t *control = find_servo_control(id);
    return control ? control->state : DXL_STATE_IDLE;
}

// Get current position of a servo
bool state_machine_get_position(uint8_t id, int32_t *position) {
    dxl_servo_control_t *control = find_servo_control(id);
    if (!control || !position) return false;
    
    *position = control->current_position;
    return true;
}

// Get current velocity of a servo
bool state_machine_get_velocity(uint8_t id, int32_t *velocity) {
    dxl_servo_control_t *control = find_servo_control(id);
    if (!control || !velocity) return false;
    
    *velocity = control->current_velocity;
    return true;
}

// Emergency stop all servos
void state_machine_emergency_stop(void) {
    for (int i = 0; i < found_count; i++) {
        if (servo_controls[i].id == 0) continue; // Skip unused slots
        
        // Special handling for gripper (motor 6)
        if (servo_controls[i].id == 6) {
            // Set PWM to 0 to stop
            if (!dxl_write_register(6, DXL_ADDR_GOAL_PWM, 2, 0)) {
                ESP_LOGE(TAG, "Failed to stop PWM for servo ID 6");
            }
            // Keep in PWM control mode
            if (!dxl_write_register(6, DXL_ADDR_OPERATING_MODE, 1, DXL_PWM_CONTROL_MODE)) {
                ESP_LOGE(TAG, "Failed to maintain PWM control mode for servo ID 6");
            }
            servo_controls[i].state = DXL_STATE_HOLDING;
            continue;
        }
        
        // For other motors, set velocity to 0
        if (!dxl_set_velocity(servo_controls[i].id, 0)) {
            ESP_LOGE(TAG, "Failed to stop servo ID %d", servo_controls[i].id);
        }
        servo_controls[i].state = DXL_STATE_HOLDING;
    }
}

// Update position control task
void dxl_position_control_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Position control task started");
    
    while (1) {
        // Process any pending position commands
        dxl_position_command_t cmd;
        while (xQueueReceive(position_command_queue, &cmd, 0) == pdTRUE) {
            ESP_LOGI(TAG, "Received position command: servo %d -> %" PRId32, cmd.servo_id, cmd.position);
            for (int i = 0; i < MAX_SERVOS; i++) {
                if (servo_controls[i].id == cmd.servo_id) {
                    servo_controls[i].target_position = cmd.position;
                    pid_controllers[i].target = cmd.position;
                    ESP_LOGI(TAG, "Updated target for servo %d to %" PRId32, cmd.servo_id, cmd.position);
                    break;
                }
            }
        }
        
        // Update each servo
        for (int i = 0; i < MAX_SERVOS; i++) {
            if (servo_controls[i].id == 0) continue; // Skip unused slots
            
            // Read current position
            int32_t current_position;
            if (!dxl_read_position(servo_controls[i].id, &current_position)) {
                ESP_LOGW(TAG, "Failed to read position for servo %d", servo_controls[i].id);
                continue;
            }
            
            // Calculate error
            int32_t error = servo_controls[i].target_position - current_position;
            ESP_LOGI(TAG, "Servo %d: Current=%" PRId32 ", Target=%" PRId32 ", Error=%" PRId32,
                    servo_controls[i].id, current_position, servo_controls[i].target_position, error);
            
            // Update PID controller
            float velocity = pid_controller(&pid_controllers[i], error);
            ESP_LOGI(TAG, "Servo %d: PID output = %.2f", servo_controls[i].id, velocity);
            
            // Limit velocity based on motor parameters
            int32_t limited_velocity;
            switch (servo_controls[i].id) {
                case 1: limited_velocity = (int32_t)fmaxf(fminf(velocity, MOTOR1_MAX_VELOCITY), -MOTOR1_MAX_VELOCITY); break;
                case 2: limited_velocity = (int32_t)fmaxf(fminf(velocity, MOTOR2_MAX_VELOCITY), -MOTOR2_MAX_VELOCITY); break;
                case 3: limited_velocity = (int32_t)fmaxf(fminf(velocity, MOTOR3_MAX_VELOCITY), -MOTOR3_MAX_VELOCITY); break;
                case 4: limited_velocity = (int32_t)fmaxf(fminf(velocity, MOTOR4_MAX_VELOCITY), -MOTOR4_MAX_VELOCITY); break;
                case 5: limited_velocity = (int32_t)fmaxf(fminf(velocity, MOTOR5_MAX_VELOCITY), -MOTOR5_MAX_VELOCITY); break;
                case 6: continue; // Skip PID control for gripper
                default: limited_velocity = 0; break;
            }
            
            // Set velocity based on state
            if (servo_controls[i].state == DXL_STATE_HOLDING) {
                limited_velocity = 0;  // Stop when holding position
            }
            
            // Send velocity command
            if (!dxl_set_velocity(servo_controls[i].id, limited_velocity)) {
                ESP_LOGW(TAG, "Failed to set velocity for servo %d", servo_controls[i].id);
                continue;
            }
            ESP_LOGI(TAG, "Servo %d: Set velocity to %" PRId32, servo_controls[i].id, limited_velocity);
            
            // Update state based on error
            if (abs(error) < 15) {  // Use abs for integer comparison
                if (servo_controls[i].state == DXL_STATE_MOVING) {
                    servo_controls[i].state = DXL_STATE_HOLDING;
                    ESP_LOGI(TAG, "Servo %d: Transitioned to HOLDING state", servo_controls[i].id);
                }
            } else {
                if (servo_controls[i].state == DXL_STATE_HOLDING) {
                    servo_controls[i].state = DXL_STATE_MOVING;
                    ESP_LOGI(TAG, "Servo %d: Transitioned to MOVING state", servo_controls[i].id);
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz control loop
    }
}

bool send_position_command(uint8_t servo_id, int32_t position) {
    if (servo_id >= MAX_SERVOS) return false;
    
    dxl_position_command_t cmd = {
        .servo_id = servo_id,
        .position = position
    };
    
    return xQueueSend(position_command_queue, &cmd, portMAX_DELAY) == pdTRUE;
} 