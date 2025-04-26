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

// Common positions
#define DXL_MIN_POSITION 0
#define DXL_MAX_POSITION 4095
#define DXL_CENTER_POSITION 2048

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
bool dxl_read_position(uint8_t id, uint32_t *position);

/**
 * @brief Set the velocity profile of a Dynamixel servo
 * 
 * @param id ID of the servo
 * @param velocity Velocity value (0-1023)
 * @return true if successful, false otherwise
 */
bool dxl_set_velocity(uint8_t id, uint32_t velocity);

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

#endif /* DYNAMIXEL_H */ 