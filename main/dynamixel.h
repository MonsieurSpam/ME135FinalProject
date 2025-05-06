/**
 * @file dynamixel.h
 * @brief Dynamixel motor control library for ESP32
 * 
 * This library provides generalized functions for controlling Dynamixel servos
 * using the Dynamixel Protocol 2.0 via UART communication.
 */

#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

#include <stdint.h>
#include <stdbool.h>

// Communication results
#define DXL_COMM_SUCCESS 0
#define DXL_COMM_ERROR -1
#define DXL_COMM_TX_ERROR -2
#define DXL_COMM_RX_ERROR -3

// Operating modes
#define DXL_OPERATING_MODE_CURRENT 0
#define DXL_OPERATING_MODE_VELOCITY 1
#define DXL_OPERATING_MODE_POSITION 3
#define DXL_OPERATING_MODE_EXTENDED_POSITION 4
#define DXL_OPERATING_MODE_CURRENT_BASED_POSITION 5
#define DXL_OPERATING_MODE_PWM 16

// Velocity control parameters
#define DXL_MAX_VELOCITY    1023    // Maximum velocity (about 114 RPM)
#define DXL_MIN_VELOCITY    -1023   // Minimum velocity (about -114 RPM)
#define DXL_VELOCITY_UNIT   0.229   // RPM per unit

// Common positions
#define DXL_MIN_POSITION 0
#define DXL_MAX_POSITION 4095
#define DXL_CENTER_POSITION 2048

// Dynamixel register addresses
#define DXL_ADDR_TORQUE_ENABLE 64
#define DXL_ADDR_LED 65
#define DXL_ADDR_OPERATING_MODE 11
#define DXL_ADDR_GOAL_POSITION 116
#define DXL_ADDR_PRESENT_POSITION 132
#define DXL_ADDR_MOVING 122
#define DXL_ADDR_HARDWARE_ERROR 70
#define DXL_ADDR_PROFILE_VELOCITY 112
#define DXL_ADDR_PROFILE_ACCELERATION 108
#define DXL_ADDR_GOAL_CURRENT 102
#define DXL_ADDR_GOAL_PWM 100
#define DXL_ADDR_GOAL_VELOCITY 104
#define DXL_ADDR_PRESENT_LOAD 126
#define DXL_ADDR_PWM_LIMIT 36

// XL330-M288 specific addresses
#define DXL330_ADDR_GOAL_PWM 100
#define DXL330_ADDR_PRESENT_LOAD 126
#define DXL330_ADDR_PWM_LIMIT 36

// PWM control
#define DXL_PWM_LIMIT 885  // Maximum PWM value (0.113% per unit)
#define DXL_MIN_PWM (-DXL_PWM_LIMIT)
#define DXL_MAX_PWM DXL_PWM_LIMIT
#define DXL330_MAX_PWM 885  // 100% duty cycle for XL330
#define DXL330_MIN_PWM -885 // -100% duty cycle for XL330
#define DXL_MAX_LOAD 1000  // 100% load
#define DXL_MIN_LOAD -1000 // -100% load

/**
 * @brief Initialize the Dynamixel communication interface
 * 
 * @param uart_num UART port number to use (e.g., UART_NUM_1, UART_NUM_2)
 * @param tx_pin GPIO pin for TX
 * @param rx_pin GPIO pin for RX
 * @param dir_pin GPIO pin for direction control
 * @param baudrate Baudrate for communication (common values: 57600, 1000000)
 * @return true if initialization successful, false otherwise
 */
bool dxl_init(uint8_t uart_num, int tx_pin, int rx_pin, int dir_pin, uint32_t baudrate);

/**
 * @brief Scan for Dynamixel servos and identify which IDs are present
 * 
 * @param id_list Array to store the IDs of found servos
 * @param model_numbers Array to store the model numbers of found servos
 * @param max_id Maximum ID to scan for (typically 1-253)
 * @param max_servos Maximum number of servos to find (size of id_list and model_numbers arrays)
 * @return Number of servos found
 */
int dxl_scan(uint8_t *id_list, uint16_t *model_numbers, uint8_t max_id, int max_servos);

/**
 * @brief Ping a specific Dynamixel servo
 * 
 * @param id ID of the servo to ping
 * @param model_number Pointer to store the model number if found
 * @return true if ping successful, false otherwise
 */
bool dxl_ping(uint8_t id, uint16_t *model_number);

/**
 * @brief Set the operating mode of a Dynamixel servo
 * 
 * @param id ID of the servo
 * @param mode Operating mode (use DXL_OPERATING_MODE_* constants)
 * @return true if successful, false otherwise
 */
bool dxl_set_operating_mode(uint8_t id, uint8_t mode);

/**
 * @brief Enable torque on a Dynamixel servo
 * 
 * @param id ID of the servo
 * @return true if successful, false otherwise
 */
bool dxl_enable_torque(uint8_t id);

/**
 * @brief Disable torque on a Dynamixel servo
 * 
 * @param id ID of the servo
 * @return true if successful, false otherwise
 */
bool dxl_disable_torque(uint8_t id);

/**
 * @brief Set the position of a Dynamixel servo
 * 
 * @param id ID of the servo
 * @param position Target position (0-4095 for most servos)
 * @return true if successful, false otherwise
 */
bool dxl_set_position(uint8_t id, uint32_t position);

/**
 * @brief Read the current position of a Dynamixel servo
 * 
 * @param id ID of the servo
 * @param position Pointer to store the current position
 * @return true if successful, false otherwise
 */
bool dxl_read_position(uint8_t id, int32_t *position);

/**
 * @brief Set the velocity profile of a Dynamixel servo
 * 
 * @param id ID of the servo
 * @param velocity Velocity value (0-1023)
 * @return true if successful, false otherwise
 */
bool dxl_set_velocity(uint8_t id, int32_t velocity);

/**
 * @brief Set the acceleration profile of a Dynamixel servo
 * 
 * @param id ID of the servo
 * @param acceleration Acceleration value (0-1023)
 * @return true if successful, false otherwise
 */
bool dxl_set_acceleration(uint8_t id, uint32_t acceleration);

/**
 * @brief Read an error status from a Dynamixel servo
 * 
 * @param id ID of the servo
 * @param error_code Pointer to store the error code
 * @return true if successful, false otherwise
 */
bool dxl_read_error(uint8_t id, uint8_t *error_code);

/**
 * @brief Read a value from a Dynamixel servo register
 * 
 * @param id ID of the servo
 * @param address Register address to read from
 * @param size Number of bytes to read (1, 2, or 4)
 * @param value Pointer to store the read value
 * @return true if successful, false otherwise
 */
bool dxl_read_register(uint8_t id, uint16_t address, uint8_t size, uint32_t *value);

/**
 * @brief Write a value to a Dynamixel servo register
 * 
 * @param id ID of the servo
 * @param address Register address to write to
 * @param size Number of bytes to write (1, 2, or 4)
 * @param value Value to write
 * @return true if successful, false otherwise
 */
bool dxl_write_register(uint8_t id, uint16_t address, uint8_t size, uint32_t value);

/**
 * @brief Set the torque (current) of a Dynamixel servo
 * 
 * @param id ID of the servo
 * @param current Target current value (0-2047, where 2047 is max current)
 * @return true if successful, false otherwise
 */
bool dxl_set_current(uint8_t id, uint32_t current);

/**
 * @brief Set the PWM value of a Dynamixel servo
 * 
 * @param id ID of the servo
 * @param pwm Target PWM value (-885 to 885, where 885 is 100% power)
 * @return true if successful, false otherwise
 */
bool dxl_set_pwm(uint8_t id, int16_t pwm);

/**
 * @brief Read the current load/torque of a Dynamixel servo
 * 
 * @param id ID of the servo
 * @param load Pointer to store the load value (-1000 to 1000, in 0.1% units)
 * @return true if successful, false otherwise
 */
bool dxl_read_load(uint8_t id, int16_t *load);

#endif /* DYNAMIXEL_H */ 