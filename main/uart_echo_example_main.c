/* 
 * Simplified ESP32 implementation for controlling Dynamixel XL430-W250 servo
 * using direct UART communication via Waveshare Serial Bus Servo Driver Board
 * Compatible with ESP32 Feather V2
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "driver/uart_vfs.h"
#include "linenoise/linenoise.h"

// Constant definitions
#define PACKET_MAX_LENGTH 256  // Maximum length of Dynamixel packet
#define TAG "DYNAMIXEL"

// UART settings
#define UART_PORT_NUM   UART_NUM_2
#define DXL_TXD_PIN     GPIO_NUM_14
#define DXL_RXD_PIN     GPIO_NUM_7
#define DXL_DIR_PIN     GPIO_NUM_21  // Direction control pin

// Direction control
#define TX_MODE 1
#define RX_MODE 0

// Dynamixel settings
#define BAUDRATE        1000000  // Using standard baudrate for better compatibility
#define NUM_SERVOS      6     // Number of servos to control
#define PROTOCOL_VERSION 2.0
#define DXL_ID          1      // Default ID for backward compatibility with original functions

// Position limits
#define MIN_SAFE_POSITION 1900
#define MAX_SAFE_POSITION 2100
#define CENTER_POSITION  2048

// Dynamixel memory addresses
#define ADDR_TORQUE_ENABLE      64
#define ADDR_LED                65
#define ADDR_OPERATING_MODE     11
#define ADDR_GOAL_POSITION      116
#define ADDR_PRESENT_POSITION   132
#define ADDR_MOVING             122
#define ADDR_HARDWARE_ERROR     70
#define ADDR_PROFILE_VELOCITY   112
#define ADDR_PROFILE_ACCELERATION 108

// Circular buffer size for UART
#define BUF_SIZE 512

// Dynamixel Protocol 2.0 parameters
#define DXL_LOBYTE(w) ((uint8_t)((w) & 0xFF))
#define DXL_HIBYTE(w) ((uint8_t)(((w) >> 8) & 0xFF))
#define DXL_LOWORD(l) ((uint16_t)((l) & 0xFFFF))
#define DXL_HIWORD(l) ((uint16_t)(((l) >> 16) & 0xFFFF))
#define DXL_MAKEWORD(a, b) ((uint16_t)(((uint8_t)(a)) | ((uint16_t)((uint8_t)(b))) << 8))

// Dynamixel instruction values
#define INST_PING 0x01
#define INST_READ 0x02
#define INST_WRITE 0x03

// Default settings
#define POSITION_CONTROL_MODE 3
#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0
#define DXL_MOVING_STATUS_THRESHOLD 20  // Dynamixel moving status threshold

// Communication result
#define COMM_SUCCESS 0
#define COMM_ERROR -1
#define COMM_TX_ERROR -2
#define COMM_RX_ERROR -3

// Forward declarations of functions
static void init_uart(uint32_t baudrate);
static void set_direction_pin(uint8_t direction);
static void clear_uart_buffer(void);
static void send_packet(uint8_t id, uint8_t instruction, uint8_t *parameters, uint16_t parameter_length);
static int read_status_packet(uint8_t *error, uint8_t *param_buffer, uint16_t *param_length);
static bool ping_dynamixel(uint16_t *model_number);
static bool set_position(uint32_t position);
static bool read_position(uint32_t *position);
static bool enable_torque(void);
static bool set_operating_mode(uint8_t mode);
static void dynamixel_task(void *pvParameters);

// CRC Table for Protocol 2.0
static const uint16_t crc_table[256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
};

// Calculate CRC for Protocol 2.0
static uint16_t update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size)
{
    uint16_t i, j;

    for (j = 0; j < data_blk_size; j++)
    {
        i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}

// Initialize UART for Dynamixel communication
static void init_uart(uint32_t baudrate)
{
    ESP_LOGI(TAG, "Initializing UART with baudrate: %" PRIu32, baudrate);
    
    // Configure GPIO for direction control pin
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << DXL_DIR_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);
    
    // Set direction pin to RX mode
    set_direction_pin(RX_MODE);
    
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0
    };

    // Uninstall previous UART driver if needed
    uart_driver_delete(UART_PORT_NUM);

    // Install UART driver
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, DXL_TXD_PIN, DXL_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Clear buffer
    clear_uart_buffer();
    
    ESP_LOGI(TAG, "UART initialized successfully with baudrate: %" PRIu32, baudrate);
}

// Set the direction control pin (TX/RX mode)
static void set_direction_pin(uint8_t direction)
{
    gpio_set_level(DXL_DIR_PIN, direction);
    if (direction == TX_MODE) {
        // Small delay to ensure direction pin has settled
        vTaskDelay(1); 
    }
}

// Clear UART buffer
static void clear_uart_buffer()
{
    uart_flush(UART_PORT_NUM);
}

// Send packet to Dynamixel
static void send_packet(uint8_t id, uint8_t instruction, uint8_t *parameters, uint16_t parameter_length)
{
    uint16_t packet_length = parameter_length + 3;    // 3 = Instruction(1) + CRC(2)
    uint16_t total_packet_length = packet_length + 7; // 7 = Header(3) + Reserved(1) + ID(1) + Length(2)

    uint8_t *txpacket = (uint8_t *)malloc(total_packet_length * sizeof(uint8_t));
    if (txpacket == NULL) {
        ESP_LOGE(TAG, "Memory allocation failed for TX packet");
        return;
    }

    // Header
    txpacket[0] = 0xFF;
    txpacket[1] = 0xFF;
    txpacket[2] = 0xFD;
    txpacket[3] = 0x00; // Reserved
    txpacket[4] = id;
    txpacket[5] = DXL_LOBYTE(packet_length);
    txpacket[6] = DXL_HIBYTE(packet_length);
    txpacket[7] = instruction;

    // Parameters
    for (uint16_t i = 0; i < parameter_length; i++)
    {
        txpacket[8 + i] = parameters[i];
    }

    // Calculate CRC for all bytes except CRC itself
    uint16_t crc = update_crc(0, txpacket, total_packet_length - 2);
    txpacket[total_packet_length - 2] = DXL_LOBYTE(crc);
    txpacket[total_packet_length - 1] = DXL_HIBYTE(crc);

    // Debug output
    ESP_LOGI(TAG, "TX Packet:");
    ESP_LOG_BUFFER_HEX(TAG, txpacket, total_packet_length);

    // Set to TX mode
    set_direction_pin(TX_MODE);
    
    // Send packet
    uart_write_bytes(UART_PORT_NUM, (const char*)txpacket, total_packet_length);
    uart_wait_tx_done(UART_PORT_NUM, portMAX_DELAY);
    
    // Set back to RX mode
    set_direction_pin(RX_MODE);
    
    free(txpacket);

    // Small delay to allow the servo to process the command
    vTaskDelay(pdMS_TO_TICKS(5));
}

// Read status packet from Dynamixel
static int read_status_packet(uint8_t *error, uint8_t *param_buffer, uint16_t *param_length)
{
    if (error == NULL || param_buffer == NULL || param_length == NULL) {
        ESP_LOGE(TAG, "NULL pointer provided to read_status_packet");
        return COMM_ERROR;
    }

    // Buffer for received data
    uint8_t rxpacket[PACKET_MAX_LENGTH] = {0};
    uint16_t rx_index = 0;
    
    // Wait for data with timeout
    const int MAX_RETRIES = 100; // 1 second timeout (10ms checks)
    int retry_count = 0;
    
    while (1) {
        // Check for data
        size_t available_bytes;
        uart_get_buffered_data_len(UART_PORT_NUM, &available_bytes);
        
        if (available_bytes <= 0) {
            retry_count++;
            if (retry_count > MAX_RETRIES) {
                ESP_LOGE(TAG, "No data received within timeout period (1000 ms)");
                return COMM_RX_ERROR;
            }
            vTaskDelay(pdMS_TO_TICKS(10)); // 10ms delay between checks
            continue;
        }
        
        // Read data
        int bytes_read = uart_read_bytes(UART_PORT_NUM, rxpacket, available_bytes, 100 / portTICK_PERIOD_MS);
        if (bytes_read > 0) {
            rx_index = bytes_read;
            ESP_LOGI(TAG, "Received %d bytes:", rx_index);
            ESP_LOG_BUFFER_HEX(TAG, rxpacket, rx_index);
            break;
        }
    }
    
    // Minimum length for a valid status packet
    if (rx_index < 10) {
        ESP_LOGE(TAG, "Packet too short to be valid (%d bytes)", rx_index);
        return COMM_RX_ERROR;
    }
    
    // Verify header
    if (rxpacket[0] != 0xFF || rxpacket[1] != 0xFF || rxpacket[2] != 0xFD || rxpacket[3] != 0x00) {
        ESP_LOGE(TAG, "Invalid packet header");
        return COMM_RX_ERROR;
    }
    
    // Extract parameters
    // uint8_t id = rxpacket[4]; - unused, commenting out to fix warning
    uint16_t length = DXL_MAKEWORD(rxpacket[5], rxpacket[6]);
    uint8_t instruction = rxpacket[7];
    *error = rxpacket[8];
    
    // Check if this is a status packet (0x55)
    if (instruction != 0x55) {
        ESP_LOGE(TAG, "Not a status packet (instruction = 0x%02X)", instruction);
        return COMM_RX_ERROR;
    }
    
    // Calculate parameter length (total length - instruction(1) - error(1) - CRC(2))
    uint16_t param_len = length - 4;
    
    // Check packet length
    if (rx_index < 10 + param_len) { // header(7) + instruction(1) + error(1) + params(?) + crc(2)
        ESP_LOGE(TAG, "Incomplete packet received");
        return COMM_RX_ERROR;
    }
    
    // Copy parameters
    if (param_len > 0) {
        for (uint16_t i = 0; i < param_len && i < 16; i++) { // Limit to 16 bytes for safety
            param_buffer[i] = rxpacket[9 + i];
        }
    }
    
    *param_length = param_len;
    
    // Optional: Log error if present
    if (*error != 0) {
        ESP_LOGW(TAG, "Dynamixel error status: 0x%02X", *error);
    }
    
    return COMM_SUCCESS;
}

// Ping the Dynamixel servo to test communication
static bool ping_dynamixel(uint16_t *model_number)
{
    send_packet(DXL_ID, INST_PING, NULL, 0);
    
    uint8_t error = 0;
    uint8_t params[16] = {0};
    uint16_t param_length = 0;
    
    int result = read_status_packet(&error, params, &param_length);
    if (result == COMM_SUCCESS) {
        if (param_length >= 2) {
            *model_number = DXL_MAKEWORD(params[0], params[1]);
            ESP_LOGI(TAG, "Ping success! Model number: 0x%04X (%d)", *model_number, *model_number);
            return true;
        } else {
            ESP_LOGI(TAG, "Ping response received but no model number");
            *model_number = 0;
            return true; // Still count as success
        }
    }
    
    return false;
}

// Write data to a register
static bool write_register(uint16_t address, uint8_t *data, uint16_t data_length)
{
    uint8_t *parameters = (uint8_t *)malloc((data_length + 2) * sizeof(uint8_t));
    if (parameters == NULL) {
        ESP_LOGE(TAG, "Memory allocation failed for write parameters");
        return false;
    }
    
    parameters[0] = DXL_LOBYTE(address);
    parameters[1] = DXL_HIBYTE(address);
    
    for (uint16_t i = 0; i < data_length; i++) {
        parameters[2 + i] = data[i];
    }
    
    send_packet(DXL_ID, INST_WRITE, parameters, data_length + 2);
    free(parameters);
    
    uint8_t error = 0;
    uint8_t params[16] = {0};
    uint16_t param_length = 0;
    
    int result = read_status_packet(&error, params, &param_length);
    return (result == COMM_SUCCESS && error == 0);
}

// Read data from a register
static bool read_register(uint16_t address, uint16_t data_length, uint8_t *data)
{
    uint8_t parameters[4];
    parameters[0] = DXL_LOBYTE(address);
    parameters[1] = DXL_HIBYTE(address);
    parameters[2] = DXL_LOBYTE(data_length);
    parameters[3] = DXL_HIBYTE(data_length);
    
    send_packet(DXL_ID, INST_READ, parameters, 4);
    
    uint8_t error = 0;
    uint8_t params[16] = {0}; // Buffer for receiving data, adjust size as needed
    uint16_t param_length = 0;
    
    int result = read_status_packet(&error, params, &param_length);
    if (result == COMM_SUCCESS && error == 0) {
        if (param_length == data_length) {
            memcpy(data, params, data_length);
            return true;
        }
    }
    
    return false;
}

// Write a single byte
static bool write_1byte(uint16_t address, uint8_t data)
{
    return write_register(address, &data, 1);
}

// Write 4 bytes (for position, etc.)
static bool write_4bytes(uint16_t address, uint32_t data)
{
    uint8_t data_array[4];
    data_array[0] = DXL_LOBYTE(DXL_LOWORD(data));
    data_array[1] = DXL_HIBYTE(DXL_LOWORD(data));
    data_array[2] = DXL_LOBYTE(DXL_HIWORD(data));
    data_array[3] = DXL_HIBYTE(DXL_HIWORD(data));
    
    return write_register(address, data_array, 4);
}

// Read 4 bytes (for position, etc.)
static bool read_4bytes(uint16_t address, uint32_t *data)
{
    uint8_t data_array[4];
    bool result = read_register(address, 4, data_array);
    
    if (result) {
        *data = DXL_MAKEWORD(data_array[0], data_array[1]) | 
               (DXL_MAKEWORD(data_array[2], data_array[3]) << 16);
    }
    
    return result;
}

// Enable torque
static bool enable_torque(void)
{
    return write_1byte(ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
}

// Disable torque
static bool disable_torque(void)
{
    return write_1byte(ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
}

// Set operating mode
static bool set_operating_mode(uint8_t mode)
{
    // Disable torque to change operating mode
    if (!disable_torque()) {
        return false;
    }
    
    // Set operating mode
    if (!write_1byte(ADDR_OPERATING_MODE, mode)) {
        return false;
    }
    
    return true;
}

// Set position
static bool set_position(uint32_t position)
{
    ESP_LOGI(TAG, "Moving to position: %" PRIu32, position);
    return write_4bytes(ADDR_GOAL_POSITION, position);
}

// Read position
static bool read_position(uint32_t *position)
{
    bool result = read_4bytes(ADDR_PRESENT_POSITION, position);
    if (result) {
        ESP_LOGI(TAG, "Current position: %" PRIu32, *position);
    }
    return result;
}

// Initialize console for input
static void init_console(void)
{
    // Initialize VFS & UART so we can use std::cout/cin
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    
    // Install UART driver for console I/O
    const uart_config_t uart_config = {
        .baud_rate = 115200,  // Default console baud rate
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

    /* Initialize the console */
    esp_console_config_t console_config = {
        .max_cmdline_length = 256,
        .max_cmdline_args = 8,
        .hint_color = 36  // Cyan color (ANSI color code)
    };
    ESP_ERROR_CHECK(esp_console_init(&console_config));

    /* Configure linenoise line completion library */
    /* Enable multiline editing. */
    linenoiseSetMultiLine(1);

    /* Tell linenoise where to get command completions and hints */
    linenoiseSetCompletionCallback(NULL);
    linenoiseSetHintsCallback(NULL);

    /* Set command history size */
    linenoiseHistorySetMaxLen(100);
}

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

// Function to scan for multiple servos
static int scan_servos(uint8_t *found_ids, uint16_t *model_numbers, int max_servos) {
    int found_count = 0;
    
    ESP_LOGI(TAG, "Starting servo scan (IDs 1-%d)...", max_servos);
    
    // Loop through potential IDs
    for (uint8_t id = 1; id <= max_servos; id++) {
        ESP_LOGI(TAG, "Pinging servo ID %d", id);
        
        // Send ping packet
        uint8_t *parameters = NULL; // No parameters for ping
        send_packet(id, INST_PING, parameters, 0);
        
        // Process response
        uint8_t error = 0;
        uint8_t params[16] = {0};
        uint16_t param_length = 0;
        
        int result = read_status_packet(&error, params, &param_length);
        if (result == COMM_SUCCESS) {
            uint16_t model_number = 0;
            if (param_length >= 2) {
                model_number = DXL_MAKEWORD(params[0], params[1]);
            }
            
            ESP_LOGI(TAG, "Found servo ID %d, model number: %d", id, model_number);
            
            if (found_count < max_servos) {
                found_ids[found_count] = id;
                model_numbers[found_count] = model_number;
                found_count++;
            }
        } else {
            ESP_LOGW(TAG, "No response from servo ID %d", id);
        }
        
        // Small delay between pings
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    ESP_LOGI(TAG, "Scan complete. Found %d servos.", found_count);
    return found_count;
}

// Set position for a specific servo
static bool set_servo_position(uint8_t id, uint32_t position) {
    ESP_LOGI(TAG, "Moving servo ID %d to position: %" PRIu32, id, position);
    
    uint8_t parameters[4 + 2];  // Address(2) + Position(4)
    parameters[0] = DXL_LOBYTE(ADDR_GOAL_POSITION);
    parameters[1] = DXL_HIBYTE(ADDR_GOAL_POSITION);
    parameters[2] = DXL_LOBYTE(DXL_LOWORD(position));
    parameters[3] = DXL_HIBYTE(DXL_LOWORD(position));
    parameters[4] = DXL_LOBYTE(DXL_HIWORD(position));
    parameters[5] = DXL_HIBYTE(DXL_HIWORD(position));
    
    send_packet(id, INST_WRITE, parameters, 6);
    
    uint8_t error = 0;
    uint8_t recv_params[16] = {0};
    uint16_t recv_length = 0;
    
    int result = read_status_packet(&error, recv_params, &recv_length);
    
    if (result == COMM_SUCCESS && error == 0) {
        ESP_LOGI(TAG, "Servo ID %d moved successfully", id);
        return true;
    } else {
        ESP_LOGE(TAG, "Failed to move servo ID %d, error: %d, comm result: %d", id, error, result);
        return false;
    }
}

// Enable torque for a specific servo
static bool enable_servo_torque(uint8_t id) {
    ESP_LOGI(TAG, "Enabling torque for servo ID %d", id);
    
    uint8_t parameters[3];  // Address(2) + Data(1)
    parameters[0] = DXL_LOBYTE(ADDR_TORQUE_ENABLE);
    parameters[1] = DXL_HIBYTE(ADDR_TORQUE_ENABLE);
    parameters[2] = TORQUE_ENABLE;
    
    send_packet(id, INST_WRITE, parameters, 3);
    
    uint8_t error = 0;
    uint8_t recv_params[16] = {0};
    uint16_t recv_length = 0;
    
    int result = read_status_packet(&error, recv_params, &recv_length);
    
    if (result == COMM_SUCCESS && error == 0) {
        ESP_LOGI(TAG, "Torque enabled for servo ID %d", id);
        return true;
    } else {
        ESP_LOGE(TAG, "Failed to enable torque for servo ID %d, error: %d", id, error);
        return false;
    }
}

// Set operating mode for a specific servo
static bool set_servo_operating_mode(uint8_t id, uint8_t mode) {
    ESP_LOGI(TAG, "Setting servo ID %d to mode %d", id, mode);
    
    // First disable torque
    uint8_t parameters[3];
    parameters[0] = DXL_LOBYTE(ADDR_TORQUE_ENABLE);
    parameters[1] = DXL_HIBYTE(ADDR_TORQUE_ENABLE);
    parameters[2] = TORQUE_DISABLE;
    
    send_packet(id, INST_WRITE, parameters, 3);
    
    uint8_t error = 0;
    uint8_t recv_params[16] = {0};
    uint16_t recv_length = 0;
    
    int result = read_status_packet(&error, recv_params, &recv_length);
    
    if (result != COMM_SUCCESS || error != 0) {
        ESP_LOGE(TAG, "Failed to disable torque for servo ID %d", id);
        return false;
    }
    
    // Now set operating mode
    parameters[0] = DXL_LOBYTE(ADDR_OPERATING_MODE);
    parameters[1] = DXL_HIBYTE(ADDR_OPERATING_MODE);
    parameters[2] = mode;
    
    send_packet(id, INST_WRITE, parameters, 3);
    
    error = 0;
    recv_length = 0;
    
    result = read_status_packet(&error, recv_params, &recv_length);
    
    if (result == COMM_SUCCESS && error == 0) {
        ESP_LOGI(TAG, "Operating mode set for servo ID %d", id);
        return true;
    } else {
        ESP_LOGE(TAG, "Failed to set operating mode for servo ID %d", id);
        return false;
    }
}

// Main Dynamixel control task
static void dynamixel_task(void *pvParameters)
{
    uint16_t model_numbers[NUM_SERVOS] = {0};
    uint8_t found_ids[NUM_SERVOS] = {0};
    int found_count = 0;
    
    // Initial setup with baudrate
    init_uart(BAUDRATE);
    
    ESP_LOGI(TAG, "Searching for servos...");
    
    // Scan for available servos
    found_count = scan_servos(found_ids, model_numbers, NUM_SERVOS);
    
    if (found_count > 0) {
        ESP_LOGI(TAG, "Found %d servos", found_count);
        
        // Initialize each servo found
        for (int i = 0; i < found_count; i++) {
            uint8_t id = found_ids[i];
            
            // Set to position control mode
            if (set_servo_operating_mode(id, POSITION_CONTROL_MODE)) {
                // Enable torque
                if (enable_servo_torque(id)) {
                    ESP_LOGI(TAG, "Servo ID %d initialized successfully", id);
                } else {
                    ESP_LOGE(TAG, "Failed to enable torque for servo ID %d", id);
                }
            } else {
                ESP_LOGE(TAG, "Failed to set operating mode for servo ID %d", id);
            }
        }
        
        // Move each servo to center position
        for (int i = 0; i < found_count; i++) {
            set_servo_position(found_ids[i], CENTER_POSITION);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        
        ESP_LOGI(TAG, "All servos moved to center position");
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // Small movement to validate servo control
        const uint32_t positions[] = {
            CENTER_POSITION - 50,  // Slightly left
            CENTER_POSITION + 50,  // Slightly right
            CENTER_POSITION        // Back to center
        };
        
        ESP_LOGI(TAG, "Starting small movements to verify communication");
        
        // Perform small movements for each servo
        for (int pos_idx = 0; pos_idx < 3; pos_idx++) {
            ESP_LOGI(TAG, "Movement step %d: Position %" PRIu32, pos_idx + 1, positions[pos_idx]);
            
            for (int i = 0; i < found_count; i++) {
                set_servo_position(found_ids[i], positions[pos_idx]);
                vTaskDelay(pdMS_TO_TICKS(100)); // Small delay between servos
            }
            
            vTaskDelay(pdMS_TO_TICKS(1000)); // Wait between position changes
        }
        
        ESP_LOGI(TAG, "Movement test complete");
        
        // Start interactive control
        printf("\n\n===== Dynamixel Servo Control =====\n");
        printf("Use the following keys to control all servos:\n");
        printf("W: Move all servos slightly up/left (-50)\n");
        printf("S: Move all servos slightly down/right (+50)\n");
        printf("C: Center all servos (position 2048)\n");
        printf("1-6: Select a servo (0 for all servos)\n");
        printf("Q: Quit\n");
        
        int selected_servo = 0; // 0 means all servos
        
        // Main control loop for interactive control
        while (1) {
            int ch = read_key_from_uart();
            if (ch != -1) {
                if (ch == 'q' || ch == 'Q') {
                    printf("Returning all servos to center and exiting\n");
                    for (int i = 0; i < found_count; i++) {
                        set_servo_position(found_ids[i], CENTER_POSITION);
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }
                    break;
                } else if (ch == 'w' || ch == 'W') {
                    uint32_t new_pos = CENTER_POSITION - 50;
                    printf("Moving %s to position: %" PRIu32 "\n", 
                           selected_servo == 0 ? "all servos" : "selected servo", 
                           new_pos);
                    
                    if (selected_servo == 0) {
                        // Move all servos
                        for (int i = 0; i < found_count; i++) {
                            set_servo_position(found_ids[i], new_pos);
                            vTaskDelay(pdMS_TO_TICKS(100));
                        }
                    } else {
                        // Find the servo with matching ID
                        for (int i = 0; i < found_count; i++) {
                            if (found_ids[i] == selected_servo) {
                                set_servo_position(found_ids[i], new_pos);
                                break;
                            }
                        }
                    }
                } else if (ch == 's' || ch == 'S') {
                    uint32_t new_pos = CENTER_POSITION + 50;
                    printf("Moving %s to position: %" PRIu32 "\n", 
                           selected_servo == 0 ? "all servos" : "selected servo", 
                           new_pos);
                    
                    if (selected_servo == 0) {
                        // Move all servos
                        for (int i = 0; i < found_count; i++) {
                            set_servo_position(found_ids[i], new_pos);
                            vTaskDelay(pdMS_TO_TICKS(100));
                        }
                    } else {
                        // Find the servo with matching ID
                        for (int i = 0; i < found_count; i++) {
                            if (found_ids[i] == selected_servo) {
                                set_servo_position(found_ids[i], new_pos);
                                break;
                            }
                        }
                    }
                } else if (ch == 'c' || ch == 'C') {
                    printf("Centering %s\n", selected_servo == 0 ? "all servos" : "selected servo");
                    
                    if (selected_servo == 0) {
                        // Center all servos
                        for (int i = 0; i < found_count; i++) {
                            set_servo_position(found_ids[i], CENTER_POSITION);
                            vTaskDelay(pdMS_TO_TICKS(100));
                        }
                    } else {
                        // Find the servo with matching ID
                        for (int i = 0; i < found_count; i++) {
                            if (found_ids[i] == selected_servo) {
                                set_servo_position(found_ids[i], CENTER_POSITION);
                                break;
                            }
                        }
                    }
                } else if (ch >= '0' && ch <= '6') {
                    selected_servo = ch - '0';
                    if (selected_servo == 0) {
                        printf("Selected all servos\n");
                    } else {
                        // Check if this servo was found
                        bool servo_found = false;
                        for (int i = 0; i < found_count; i++) {
                            if (found_ids[i] == selected_servo) {
                                servo_found = true;
                                break;
                            }
                        }
                        
                        if (servo_found) {
                            printf("Selected servo ID %d\n", selected_servo);
                        } else {
                            printf("Servo ID %d was not found during scan\n", selected_servo);
                        }
                    }
                }
            }
            
            // Small delay to prevent CPU hogging
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    } else {
        ESP_LOGE(TAG, "No servos found. Check connections and power.");
        printf("\nNo Dynamixel servos found. Please check:\n");
        printf("1. Servo power (usually 12V)\n");
        printf("2. Data line connections\n");
        printf("3. That servos are set to correct baudrate (%d)\n", BAUDRATE);
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
    
    ESP_LOGI(TAG, "ESP32 Dynamixel Controller");
    ESP_LOGI(TAG, "Multiple servo control version");
    ESP_LOGI(TAG, "TX Pin: %d, RX Pin: %d, DIR Pin: %d", DXL_TXD_PIN, DXL_RXD_PIN, DXL_DIR_PIN);
    
    // Create task for Dynamixel control with increased stack size for multiple servos
    xTaskCreate(dynamixel_task, "dynamixel_task", 8192, NULL, 5, NULL);
}

