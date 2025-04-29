/**
 * @file pid_constants.h
 * @brief PID control constants for Dynamixel servos
 */

#ifndef PID_CONSTANTS_H
#define PID_CONSTANTS_H

// PID control constants
#define PID_KP 1.0f    // Proportional gain
#define PID_KI 0.1f    // Integral gain
#define PID_KD 0.01f   // Derivative gain

// Maximum current limit (0-2047)
#define PID_MAX_CURRENT 500.0f

// Control loop period in milliseconds
#define PID_CONTROL_PERIOD_MS 10

#endif /* PID_CONSTANTS_H */ 