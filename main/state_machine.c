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
#define UPDATE_PERIOD_MS 50  // Increased from 20ms to slow down control rate
#define POSITION_ERROR_THRESHOLD 20
#define HOLDING_ERROR_THRESHOLD 100
#define MAX_SERVOS 4
#define DIRECTION_CHANGE_THRESHOLD 50  // New threshold for detecting direction changes
#define DEADBAND_COMPENSATION 50  // Added deadband compensation
#define MOTOR2_FORWARD_HOLDING_PWM 600  // Higher holding PWM for forward positions
#define MOTOR2_FORWARD_THRESHOLD 2100   // Position threshold where we consider it "forward"
#define MOTOR3_FORWARD_HOLDING_PWM 600  // Higher holding PWM for forward positions
#define MOTOR3_FORWARD_THRESHOLD 1700   // Position threshold where we consider it "forward"
#define MOTOR2_ELBOW_UP_HOLDING_PWM 800  // Even higher holding PWM when elbow is up
#define MOTOR3_ELBOW_UP_THRESHOLD 1900   // Position where elbow is considered "up"

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

// Update PID controller and return control output
float update_pid(pid_controller_t* pid, float current_position) {
    // Calculate error (target - current)
    float error = pid->target - current_position;
    
    // Calculate proportional term with increased gain for larger errors
    float p_term = pid->kp * error;
    if (fabsf(error) > 100) {
        p_term *= 1.5f; // Increased from 0.8f for more aggressive response
    }
    
    // Add deadband compensation for large errors
    if (fabsf(error) > 200) {
        p_term += (error > 0 ? DEADBAND_COMPENSATION : -DEADBAND_COMPENSATION);
    }
    
    // Update integral term with anti-windup and direction change detection
    float i_term = pid->ki * pid->integral;
    
    // Check for direction change
    if (fabsf(error) > DIRECTION_CHANGE_THRESHOLD) {
        // If error sign changed significantly, reset integral
        if ((error * pid->prev_error) < 0) {
            pid->integral = 0.0f;
            ESP_LOGI(TAG, "Direction change detected, resetting integral");
        }
    }
    
    if (fabsf(i_term) < pid->integral_limit) {
        pid->integral += error;
    } else {
        // Reset integral if we're at the limit
        pid->integral = 0.0f;
    }
    
    // Calculate derivative term with smoothing and direction change handling
    float d_term = pid->kd * (error - pid->prev_error);
    pid->prev_error = error;
    
    // Calculate total output
    float output = p_term + i_term + d_term;
    
    // Limit output to max current while preserving sign
    if (output > 0) {
        output = fminf(output, pid->max_current);
    } else {
        output = fmaxf(output, -pid->max_current);
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
    
    // Initialize all motors (1-4)
    for (int i = 0; i < found_count; i++) {
        uint8_t id = servo_ids[i];
        
        // Only process motors 1-4
        if (id < 1 || id > 4) continue;
        
        dxl_servo_control_t *control = &servo_controls[i];
        memset(control, 0, sizeof(dxl_servo_control_t));
        
        control->id = id;
        control->state = DXL_STATE_MOVING;
        
        // Set initial position and PWM limits based on motor ID
        switch (id) {
            case 1: // Base rotation
                control->target_position = MOTOR1_CENTER_POSITION;
                control->max_moving_pwm = MOTOR1_MOVING_PWM;
                control->max_holding_pwm = MOTOR1_HOLDING_PWM;
                init_pid_controller(&pid_controllers[i], MOTOR1_KP, MOTOR1_KI, MOTOR1_KD, control->max_moving_pwm);
                break;
            case 2: // Shoulder
                control->target_position = MOTOR2_INITIAL_POSITION;
                control->max_moving_pwm = MOTOR2_MOVING_PWM;
                control->max_holding_pwm = MOTOR2_HOLDING_PWM;
                init_pid_controller(&pid_controllers[i], MOTOR2_KP, MOTOR2_KI, MOTOR2_KD, control->max_moving_pwm);
                ESP_LOGI(TAG, "Motor 2 (Shoulder) initialized with target position %" PRId32, control->target_position);
                break;
            case 3: // Elbow
                control->target_position = MOTOR3_INITIAL_POSITION;
                control->max_moving_pwm = MOTOR3_MOVING_PWM;
                control->max_holding_pwm = MOTOR3_HOLDING_PWM;
                init_pid_controller(&pid_controllers[i], MOTOR3_KP, MOTOR3_KI, MOTOR3_KD, control->max_moving_pwm);
                break;
            case 4: // Wrist
                control->target_position = MOTOR4_INITIAL_POSITION;
                control->max_moving_pwm = MOTOR4_MOVING_PWM;
                control->max_holding_pwm = MOTOR4_HOLDING_PWM;
                init_pid_controller(&pid_controllers[i], MOTOR4_KP, MOTOR4_KI, MOTOR4_KD, control->max_moving_pwm);
                break;
        }
        
        pid_controllers[i].target = control->target_position;
        
        ESP_LOGI(TAG, "Motor %d initialized with moving PWM %d and holding PWM %d",
                 id, control->max_moving_pwm, control->max_holding_pwm);
    }

    // Initialize position command queue
    position_command_queue = xQueueCreate(10, sizeof(dxl_position_command_t));
}

// Update the state machine
void state_machine_update(void)
{
    for (int i = 0; i < found_count; i++) {
        uint8_t id = servo_controls[i].id;
        if (id > 4) continue; // Skip gripper motors
        
        // Read current position
        int32_t current_pos;
        if (!dxl_read_position(id, &current_pos)) {
            ESP_LOGE(TAG, "Failed to read position for servo ID %d", id);
            continue;
        }
        
        // Get target position from PID controller
        int32_t target_pos = pid_controllers[i].target;
        
        // Calculate error
        float error = target_pos - current_pos;
        
        // Remove special handling for motor 2
        // Ensure position limits for all motors
        if (current_pos < MOTOR2_MIN_POSITION) {
            current_pos = MOTOR2_MIN_POSITION;
            error = 0;
        } else if (current_pos > MOTOR2_MAX_POSITION) {
            current_pos = MOTOR2_MAX_POSITION;
            error = 0;
        }
        
        // Update PID controller
        float output = update_pid(&pid_controllers[i], current_pos);
        
        // Apply PWM limits based on servo ID
        int16_t pwm = 0;
        if (fabsf(error) > 5.0f) { // Moving state
            switch (id) {
                case 1: pwm = (int16_t)fminf(fmaxf(output, -MOTOR1_MOVING_PWM), MOTOR1_MOVING_PWM); break;
                case 2: pwm = (int16_t)fminf(fmaxf(output, -MOTOR2_MOVING_PWM), MOTOR2_MOVING_PWM); break;
                case 3: pwm = (int16_t)fminf(fmaxf(output, -MOTOR3_MOVING_PWM), MOTOR3_MOVING_PWM); break;
                case 4: pwm = (int16_t)fminf(fmaxf(output, -MOTOR4_MOVING_PWM), MOTOR4_MOVING_PWM); break;
            }
        } else { // Holding state
            switch (id) {
                case 1: pwm = (int16_t)fminf(fmaxf(output, -MOTOR1_HOLDING_PWM), MOTOR1_HOLDING_PWM); break;
                case 2: pwm = (int16_t)fminf(fmaxf(output, -MOTOR2_HOLDING_PWM), MOTOR2_HOLDING_PWM); break;
                case 3: pwm = (int16_t)fminf(fmaxf(output, -MOTOR3_HOLDING_PWM), MOTOR3_HOLDING_PWM); break;
                case 4: pwm = (int16_t)fminf(fmaxf(output, -MOTOR4_HOLDING_PWM), MOTOR4_HOLDING_PWM); break;
            }
        }
        
        // Set PWM for the servo
        if (!dxl_set_pwm(id, pwm)) {
            ESP_LOGE(TAG, "Failed to set PWM for servo ID %d", id);
        }
        
        ESP_LOGI(TAG, "Servo %d: Current=%ld, Target=%ld, Error=%.1f, PWM=%d", 
                 id, current_pos, target_pos, error, pwm);
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

// Get current PWM of a servo
bool state_machine_get_pwm(uint8_t id, int16_t *pwm) {
    dxl_servo_control_t *control = find_servo_control(id);
    if (!control || !pwm) return false;
    
    *pwm = control->current_pwm;
    return true;
}

// Emergency stop all servos
void state_machine_emergency_stop(void) {
    for (int i = 0; i < found_count; i++) {
        dxl_servo_control_t *control = &servo_controls[i];
        control->state = DXL_STATE_IDLE;
        control->current_pwm = 0;
        dxl_set_pwm(control->id, 0);
    }
    ESP_LOGI(TAG, "Emergency stop activated");
}

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
            
            // Get current positions of both motors 2 and 3
            int32_t motor2_pos = servo_controls[1].current_position;  // Motor 2 is index 1
            int32_t motor3_pos = servo_controls[2].current_position;  // Motor 3 is index 2
            
            // For motor 2, determine holding PWM based on both its position and motor 3's position
            int32_t holding_pwm = servo_controls[i].max_holding_pwm;
            if (servo_controls[i].id == 2) {
                if (current_position > MOTOR2_FORWARD_THRESHOLD) {
                    if (motor3_pos > MOTOR3_ELBOW_UP_THRESHOLD) {
                        holding_pwm = MOTOR2_ELBOW_UP_HOLDING_PWM;  // Extra high holding when elbow is up
                    } else {
                        holding_pwm = MOTOR2_FORWARD_HOLDING_PWM;
                    }
                }
            } else if (servo_controls[i].id == 3) {
                if (current_position > MOTOR3_FORWARD_THRESHOLD) {
                    holding_pwm = MOTOR3_FORWARD_HOLDING_PWM;
                }
            }
            
            // Update PID controller
            float pwm = update_pid(&pid_controllers[i], current_position);
            ESP_LOGI(TAG, "Servo %d: PID output = %.2f", servo_controls[i].id, pwm);
            
            // Ensure PWM is in the correct direction based on error
            if (error > 0) {
                // Need to move in positive direction
                pwm = fmaxf(pwm, 0);
            } else {
                // Need to move in negative direction
                pwm = fminf(pwm, 0);
            }
            
            // Limit PWM based on state
            int16_t limited_pwm;
            if (servo_controls[i].state == DXL_STATE_MOVING) {
                limited_pwm = (int16_t)fmaxf(fminf(pwm, servo_controls[i].max_moving_pwm), -servo_controls[i].max_moving_pwm);
            } else {
                limited_pwm = (int16_t)fmaxf(fminf(pwm, holding_pwm), -holding_pwm);
            }
            
            // Send PWM command
            if (!dxl_set_pwm(servo_controls[i].id, limited_pwm)) {
                ESP_LOGW(TAG, "Failed to set PWM for servo %d", servo_controls[i].id);
                continue;
            }
            ESP_LOGI(TAG, "Servo %d: Set PWM to %d", servo_controls[i].id, limited_pwm);
            
            // Update state based on error
            if (abs(error) < 5) {
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