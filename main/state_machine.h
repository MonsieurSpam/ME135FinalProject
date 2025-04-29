#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>
#include <stdbool.h>
#include "dynamixel.h"

// PID controller structure
typedef struct {
    float kp;           // Proportional gain
    float ki;           // Integral gain
    float kd;           // Derivative gain
    float integral;     // Integral term
    float prev_error;   // Previous error for derivative term
    float target;       // Target position
    float current;      // Current position
    float max_current;  // Maximum allowed current
    float integral_limit; // Limit for integral term to prevent windup
} pid_controller_t;

// Position command structure
typedef struct {
    uint8_t servo_id;
    int32_t position;
} dxl_position_command_t;

// State definitions
typedef enum {
    DXL_STATE_IDLE,     // Initial state, no movement
    DXL_STATE_MOVING,   // Actively moving to target
    DXL_STATE_HOLDING   // Holding position with PID
} dxl_state_t;

// Control structure for each servo
typedef struct {
    uint8_t id;                     // Servo ID
    dxl_state_t state;              // Current state
    int32_t target_position;        // Target position
    int32_t current_position;       // Current position
    int16_t current_pwm;            // Current PWM value
    int16_t max_moving_pwm;         // Maximum PWM for movement
    int16_t max_holding_pwm;        // Maximum PWM for holding
    pid_controller_t pid;           // PID controller
    uint32_t last_update_time;      // Last update timestamp
} dxl_servo_control_t;

// Initialize PID controller
void init_pid_controller(pid_controller_t* pid, float kp, float ki, float kd, float max_current);

// Update PID controller and return control output
float update_pid(pid_controller_t* pid, float current_position);

// Initialize the state machine
void state_machine_init(void);

// Update the state machine (call periodically)
void state_machine_update(void);

// Set target position for a servo
bool state_machine_set_target(uint8_t id, int32_t position);

// Get current state of a servo
dxl_state_t state_machine_get_state(uint8_t id);

// Get current position of a servo
bool state_machine_get_position(uint8_t id, int32_t *position);

// Get current PWM of a servo
bool state_machine_get_pwm(uint8_t id, int16_t *pwm);

// Emergency stop all servos
void state_machine_emergency_stop(void);

// Position control task
void dxl_position_control_task(void *pvParameters);

// Send position command to a servo
bool send_position_command(uint8_t servo_id, int32_t position);

#endif // STATE_MACHINE_H 