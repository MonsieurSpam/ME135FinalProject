/**
 * @file dynamixel.c
 * @brief Implementation of Dynamixel motor control library
 */

#include "dynamixel.h"
#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "DYNAMIXEL"

// Buffer sizes
#define DXL_PACKET_MAX_LENGTH 256
#define DXL_BUF_SIZE 512

// Direction control
#define DXL_TX_MODE 1
#define DXL_RX_MODE 0

// Dynamixel Protocol 2.0 parameters
#define DXL_LOBYTE(w) ((uint8_t)((w) & 0xFF))
#define DXL_HIBYTE(w) ((uint8_t)(((w) >> 8) & 0xFF))
#define DXL_LOWORD(l) ((uint16_t)((l) & 0xFFFF))
#define DXL_HIWORD(l) ((uint16_t)(((l) >> 16) & 0xFFFF))
#define DXL_MAKEWORD(a, b) ((uint16_t)(((uint8_t)(a)) | ((uint16_t)((uint8_t)(b))) << 8))

// Dynamixel instruction values
#define DXL_INST_PING 0x01
#define DXL_INST_READ 0x02
#define DXL_INST_WRITE 0x03

// Dynamixel register addresses
#define DXL_ADDR_TORQUE_ENABLE      64
#define DXL_ADDR_LED                65
#define DXL_ADDR_OPERATING_MODE     11
#define DXL_ADDR_GOAL_POSITION      116
#define DXL_ADDR_PRESENT_POSITION   132
#define DXL_ADDR_MOVING             122
#define DXL_ADDR_HARDWARE_ERROR     70
#define DXL_ADDR_PROFILE_VELOCITY   112
#define DXL_ADDR_PROFILE_ACCELERATION 108

// Static variables for UART configuration
static uint8_t dxl_uart_num = 0;
static int dxl_tx_pin = -1;
static int dxl_rx_pin = -1;
static int dxl_dir_pin = -1;
static bool dxl_initialized = false;

// CRC Table for Protocol 2.0
static const uint16_t dxl_crc_table[256] = {
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
static uint16_t dxl_update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size)
{
    uint16_t i, j;

    for (j = 0; j < data_blk_size; j++)
    {
        i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ dxl_crc_table[i];
    }

    return crc_accum;
}

// Set the direction control pin (TX/RX mode)
static void dxl_set_direction(uint8_t direction)
{
    if (!dxl_initialized || dxl_dir_pin < 0) {
        return;
    }
    
    gpio_set_level(dxl_dir_pin, direction);
    if (direction == DXL_TX_MODE) {
        // Small delay to ensure direction pin has settled
        vTaskDelay(1); 
    }
}

// Clear UART buffer
static void dxl_clear_buffer(void)
{
    if (!dxl_initialized) {
        return;
    }
    
    uart_flush(dxl_uart_num);
}

// Send packet to Dynamixel
static void dxl_send_packet(uint8_t id, uint8_t instruction, uint8_t *parameters, uint16_t parameter_length)
{
    if (!dxl_initialized) {
        ESP_LOGE(TAG, "Dynamixel not initialized");
        return;
    }

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
    uint16_t crc = dxl_update_crc(0, txpacket, total_packet_length - 2);
    txpacket[total_packet_length - 2] = DXL_LOBYTE(crc);
    txpacket[total_packet_length - 1] = DXL_HIBYTE(crc);

    // Debug output
    ESP_LOGI(TAG, "TX Packet:");
    ESP_LOG_BUFFER_HEX(TAG, txpacket, total_packet_length);

    // Set to TX mode
    dxl_set_direction(DXL_TX_MODE);
    
    // Send packet
    uart_write_bytes(dxl_uart_num, (const char*)txpacket, total_packet_length);
    uart_wait_tx_done(dxl_uart_num, portMAX_DELAY);
    
    // Set back to RX mode
    dxl_set_direction(DXL_RX_MODE);
    
    free(txpacket);

    // Small delay to allow the servo to process the command
    vTaskDelay(pdMS_TO_TICKS(5));
}

// Read status packet from Dynamixel
static int dxl_read_status_packet(uint8_t *error, uint8_t *param_buffer, uint16_t *param_length)
{
    if (!dxl_initialized) {
        ESP_LOGE(TAG, "Dynamixel not initialized");
        return DXL_COMM_ERROR;
    }

    if (error == NULL || param_buffer == NULL || param_length == NULL) {
        ESP_LOGE(TAG, "NULL pointer provided to dxl_read_status_packet");
        return DXL_COMM_ERROR;
    }

    // Buffer for received data
    uint8_t rxpacket[DXL_PACKET_MAX_LENGTH] = {0};
    uint16_t rx_index = 0;
    
    // Wait for data with timeout
    const int MAX_RETRIES = 100; // 1 second timeout (10ms checks)
    int retry_count = 0;
    
    while (1) {
        // Check for data
        size_t available_bytes;
        uart_get_buffered_data_len(dxl_uart_num, &available_bytes);
        
        if (available_bytes <= 0) {
            retry_count++;
            if (retry_count > MAX_RETRIES) {
                ESP_LOGE(TAG, "No data received within timeout period (1000 ms)");
                return DXL_COMM_RX_ERROR;
            }
            vTaskDelay(pdMS_TO_TICKS(10)); // 10ms delay between checks
            continue;
        }
        
        // Read data
        int bytes_to_read = (available_bytes > DXL_PACKET_MAX_LENGTH) ? DXL_PACKET_MAX_LENGTH : available_bytes;
        int bytes_read = uart_read_bytes(dxl_uart_num, rxpacket, bytes_to_read, 100 / portTICK_PERIOD_MS);
        if (bytes_read > 0) {
            rx_index = bytes_read;
            ESP_LOGI(TAG, "Received %d bytes:", rx_index);
            ESP_LOG_BUFFER_HEX(TAG, rxpacket, rx_index);
            break;
        }
    }
    
    // Clear any remaining bytes in buffer
    uart_flush(dxl_uart_num);
    
    // Minimum length for a valid status packet
    if (rx_index < 10) {
        ESP_LOGE(TAG, "Packet too short to be valid (%d bytes)", rx_index);
        return DXL_COMM_RX_ERROR;
    }
    
    // Verify header with more defensive checks
    if (rx_index < 4 || rxpacket[0] != 0xFF || rxpacket[1] != 0xFF || rxpacket[2] != 0xFD || rxpacket[3] != 0x00) {
        ESP_LOGE(TAG, "Invalid packet header");
        return DXL_COMM_RX_ERROR;
    }
    
    // Safety check for accessing array elements
    if (rx_index < 9) {
        ESP_LOGE(TAG, "Packet too short for required fields");
        return DXL_COMM_RX_ERROR;
    }
    
    // Extract parameters
    uint16_t length = DXL_MAKEWORD(rxpacket[5], rxpacket[6]);
    uint8_t instruction = rxpacket[7];
    *error = rxpacket[8];
    
    // Check if this is a status packet (0x55)
    if (instruction != 0x55) {
        ESP_LOGE(TAG, "Not a status packet (instruction = 0x%02X)", instruction);
        return DXL_COMM_RX_ERROR;
    }
    
    // Calculate parameter length (total length - instruction(1) - error(1) - CRC(2))
    uint16_t param_len = (length > 4) ? length - 4 : 0;
    
    // Check packet length and bounds
    if (rx_index < 10 + param_len) { // header(7) + instruction(1) + error(1) + params(?) + crc(2)
        ESP_LOGE(TAG, "Incomplete packet received");
        return DXL_COMM_RX_ERROR;
    }
    
    // Limit parameter length to prevent buffer overflow
    uint16_t max_param_size = 16; // Maximum parameter size allowed
    param_len = (param_len > max_param_size) ? max_param_size : param_len;
    
    // Copy parameters with bounds checking
    if (param_len > 0) {
        memset(param_buffer, 0, param_len); // Clear buffer first
        for (uint16_t i = 0; i < param_len; i++) {
            if (9 + i < rx_index) { // Ensure we don't read past the received data
                param_buffer[i] = rxpacket[9 + i];
            }
        }
    }
    
    *param_length = param_len;
    
    // Optional: Log error if present
    if (*error != 0) {
        ESP_LOGW(TAG, "Dynamixel error status: 0x%02X", *error);
    }
    
    return DXL_COMM_SUCCESS;
}

// Implement public API functions

bool dxl_init(uint8_t uart_num, int tx_pin, int rx_pin, int dir_pin, uint32_t baudrate)
{
    ESP_LOGI(TAG, "Initializing Dynamixel with UART%d (TX:%d, RX:%d, DIR:%d) at %d baud", 
             uart_num, tx_pin, rx_pin, dir_pin, (int)baudrate);
    
    // Store configuration
    dxl_uart_num = uart_num;
    dxl_tx_pin = tx_pin;
    dxl_rx_pin = rx_pin;
    dxl_dir_pin = dir_pin;
    
    // Configure GPIO for direction control pin
    if (dir_pin >= 0) {
        gpio_config_t io_conf = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = (1ULL << dir_pin),
            .pull_down_en = 0,
            .pull_up_en = 0
        };
        gpio_config(&io_conf);
        
        // Set direction pin to RX mode
        dxl_set_direction(DXL_RX_MODE);
    }
    
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
    uart_driver_delete(uart_num);

    // Install UART driver
    esp_err_t err = uart_driver_install(uart_num, DXL_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %d", err);
        return false;
    }
    
    err = uart_param_config(uart_num, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART: %d", err);
        return false;
    }
    
    err = uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %d", err);
        return false;
    }

    // Clear buffer
    dxl_clear_buffer();
    
    dxl_initialized = true;
    ESP_LOGI(TAG, "Dynamixel initialized successfully");
    return true;
}

int dxl_scan(uint8_t *id_list, uint16_t *model_numbers, uint8_t max_id, int max_servos)
{
    if (!dxl_initialized) {
        ESP_LOGE(TAG, "Dynamixel not initialized");
        return 0;
    }
    
    if (id_list == NULL || model_numbers == NULL || max_servos <= 0) {
        ESP_LOGE(TAG, "Invalid parameters to dxl_scan");
        return 0;
    }
    
    int found_count = 0;
    
    ESP_LOGI(TAG, "Starting servo scan (IDs 1-%d)...", max_id);
    
    // Loop through potential IDs
    for (uint8_t id = 1; id <= max_id && found_count < max_servos; id++) {
        ESP_LOGI(TAG, "Pinging servo ID %d", id);
        
        uint16_t model_number = 0;
        if (dxl_ping(id, &model_number)) {
            ESP_LOGI(TAG, "Found servo ID %d, model number: %d", id, model_number);
            
            id_list[found_count] = id;
            model_numbers[found_count] = model_number;
            found_count++;
        } else {
            ESP_LOGW(TAG, "No response from servo ID %d", id);
        }
        
        // Small delay between pings
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    ESP_LOGI(TAG, "Scan complete. Found %d servos.", found_count);
    return found_count;
}

bool dxl_ping(uint8_t id, uint16_t *model_number)
{
    if (!dxl_initialized) {
        ESP_LOGE(TAG, "Dynamixel not initialized");
        return false;
    }
    
    if (model_number == NULL) {
        ESP_LOGE(TAG, "NULL pointer provided to dxl_ping");
        return false;
    }

    // Clear buffer before sending
    dxl_clear_buffer();

    // Send ping packet
    dxl_send_packet(id, DXL_INST_PING, NULL, 0);
    
    uint8_t error = 0;
    uint8_t params[16] = {0};
    uint16_t param_length = 0;
    
    int result = dxl_read_status_packet(&error, params, &param_length);
    if (result == DXL_COMM_SUCCESS) {
        if (param_length >= 2) {
            *model_number = DXL_MAKEWORD(params[0], params[1]);
            ESP_LOGI(TAG, "Ping success! Model number: 0x%04X (%d)", *model_number, *model_number);
            return true;
        } else {
            ESP_LOGI(TAG, "Ping response received but no model number");
            *model_number = 0;
            return true; // Still count as success
        }
    } else {
        ESP_LOGE(TAG, "Communication error during ping: %d", result);
        *model_number = 0;
    }
    
    return false;
}

bool dxl_set_operating_mode(uint8_t id, uint8_t mode)
{
    if (!dxl_initialized) {
        ESP_LOGE(TAG, "Dynamixel not initialized");
        return false;
    }
    
    ESP_LOGI(TAG, "Setting servo ID %d to mode %d", id, mode);
    
    // First disable torque
    if (!dxl_disable_torque(id)) {
        ESP_LOGE(TAG, "Failed to disable torque for servo ID %d", id);
        return false;
    }
    
    // Now set operating mode
    uint8_t parameters[3];
    parameters[0] = DXL_LOBYTE(DXL_ADDR_OPERATING_MODE);
    parameters[1] = DXL_HIBYTE(DXL_ADDR_OPERATING_MODE);
    parameters[2] = mode;
    
    dxl_send_packet(id, DXL_INST_WRITE, parameters, 3);
    
    uint8_t error = 0;
    uint8_t recv_params[16] = {0};
    uint16_t recv_length = 0;
    
    int result = dxl_read_status_packet(&error, recv_params, &recv_length);
    
    if (result == DXL_COMM_SUCCESS && error == 0) {
        ESP_LOGI(TAG, "Operating mode set for servo ID %d", id);
        return true;
    } else {
        ESP_LOGE(TAG, "Failed to set operating mode for servo ID %d", id);
        return false;
    }
}

bool dxl_enable_torque(uint8_t id)
{
    if (!dxl_initialized) {
        ESP_LOGE(TAG, "Dynamixel not initialized");
        return false;
    }
    
    ESP_LOGI(TAG, "Enabling torque for servo ID %d", id);
    
    uint8_t parameters[3];
    parameters[0] = DXL_LOBYTE(DXL_ADDR_TORQUE_ENABLE);
    parameters[1] = DXL_HIBYTE(DXL_ADDR_TORQUE_ENABLE);
    parameters[2] = 1; // Enable
    
    dxl_send_packet(id, DXL_INST_WRITE, parameters, 3);
    
    uint8_t error = 0;
    uint8_t recv_params[16] = {0};
    uint16_t recv_length = 0;
    
    int result = dxl_read_status_packet(&error, recv_params, &recv_length);
    
    if (result == DXL_COMM_SUCCESS && error == 0) {
        ESP_LOGI(TAG, "Torque enabled for servo ID %d", id);
        return true;
    } else {
        ESP_LOGE(TAG, "Failed to enable torque for servo ID %d", id);
        return false;
    }
}

bool dxl_disable_torque(uint8_t id)
{
    if (!dxl_initialized) {
        ESP_LOGE(TAG, "Dynamixel not initialized");
        return false;
    }
    
    ESP_LOGI(TAG, "Disabling torque for servo ID %d", id);
    
    uint8_t parameters[3];
    parameters[0] = DXL_LOBYTE(DXL_ADDR_TORQUE_ENABLE);
    parameters[1] = DXL_HIBYTE(DXL_ADDR_TORQUE_ENABLE);
    parameters[2] = 0; // Disable
    
    dxl_send_packet(id, DXL_INST_WRITE, parameters, 3);
    
    uint8_t error = 0;
    uint8_t recv_params[16] = {0};
    uint16_t recv_length = 0;
    
    int result = dxl_read_status_packet(&error, recv_params, &recv_length);
    
    if (result == DXL_COMM_SUCCESS && error == 0) {
        ESP_LOGI(TAG, "Torque disabled for servo ID %d", id);
        return true;
    } else {
        ESP_LOGE(TAG, "Failed to disable torque for servo ID %d", id);
        return false;
    }
}

bool dxl_set_position(uint8_t id, uint32_t position)
{
    if (!dxl_initialized) {
        ESP_LOGE(TAG, "Dynamixel not initialized");
        return false;
    }
    
    ESP_LOGI(TAG, "Setting servo ID %d to position: %d", id, (int)position);
    
    uint8_t parameters[6];
    parameters[0] = DXL_LOBYTE(DXL_ADDR_GOAL_POSITION);
    parameters[1] = DXL_HIBYTE(DXL_ADDR_GOAL_POSITION);
    parameters[2] = DXL_LOBYTE(DXL_LOWORD(position));
    parameters[3] = DXL_HIBYTE(DXL_LOWORD(position));
    parameters[4] = DXL_LOBYTE(DXL_HIWORD(position));
    parameters[5] = DXL_HIBYTE(DXL_HIWORD(position));
    
    dxl_send_packet(id, DXL_INST_WRITE, parameters, 6);
    
    uint8_t error = 0;
    uint8_t recv_params[16] = {0};
    uint16_t recv_length = 0;
    
    int result = dxl_read_status_packet(&error, recv_params, &recv_length);
    
    if (result == DXL_COMM_SUCCESS && error == 0) {
        ESP_LOGI(TAG, "Position set for servo ID %d", id);
        return true;
    } else {
        ESP_LOGE(TAG, "Failed to set position for servo ID %d", id);
        return false;
    }
}

bool dxl_read_position(uint8_t id, uint32_t *position)
{
    if (!dxl_initialized) {
        ESP_LOGE(TAG, "Dynamixel not initialized");
        return false;
    }
    
    if (position == NULL) {
        ESP_LOGE(TAG, "NULL pointer provided to dxl_read_position");
        return false;
    }
    
    uint8_t parameters[4];
    parameters[0] = DXL_LOBYTE(DXL_ADDR_PRESENT_POSITION);
    parameters[1] = DXL_HIBYTE(DXL_ADDR_PRESENT_POSITION);
    parameters[2] = DXL_LOBYTE(4); // Read 4 bytes (32-bit position)
    parameters[3] = DXL_HIBYTE(4);
    
    dxl_send_packet(id, DXL_INST_READ, parameters, 4);
    
    uint8_t error = 0;
    uint8_t recv_params[16] = {0};
    uint16_t recv_length = 0;
    
    int result = dxl_read_status_packet(&error, recv_params, &recv_length);
    
    if (result == DXL_COMM_SUCCESS && error == 0) {
        if (recv_length >= 4) {
            *position = DXL_MAKEWORD(recv_params[0], recv_params[1]) | 
                       (DXL_MAKEWORD(recv_params[2], recv_params[3]) << 16);
            ESP_LOGI(TAG, "Current position of servo ID %d: %d", id, (int)*position);
            return true;
        } else {
            ESP_LOGE(TAG, "Did not receive enough data for position");
            return false;
        }
    } else {
        ESP_LOGE(TAG, "Failed to read position for servo ID %d", id);
        return false;
    }
}

bool dxl_set_velocity(uint8_t id, uint32_t velocity)
{
    if (!dxl_initialized) {
        ESP_LOGE(TAG, "Dynamixel not initialized");
        return false;
    }
    
    ESP_LOGI(TAG, "Setting velocity for servo ID %d: %d", id, (int)velocity);
    
    uint8_t parameters[6];
    parameters[0] = DXL_LOBYTE(DXL_ADDR_PROFILE_VELOCITY);
    parameters[1] = DXL_HIBYTE(DXL_ADDR_PROFILE_VELOCITY);
    parameters[2] = DXL_LOBYTE(DXL_LOWORD(velocity));
    parameters[3] = DXL_HIBYTE(DXL_LOWORD(velocity));
    parameters[4] = DXL_LOBYTE(DXL_HIWORD(velocity));
    parameters[5] = DXL_HIBYTE(DXL_HIWORD(velocity));
    
    dxl_send_packet(id, DXL_INST_WRITE, parameters, 6);
    
    uint8_t error = 0;
    uint8_t recv_params[16] = {0};
    uint16_t recv_length = 0;
    
    int result = dxl_read_status_packet(&error, recv_params, &recv_length);
    
    if (result == DXL_COMM_SUCCESS && error == 0) {
        ESP_LOGI(TAG, "Velocity set for servo ID %d", id);
        return true;
    } else {
        ESP_LOGE(TAG, "Failed to set velocity for servo ID %d", id);
        return false;
    }
}

bool dxl_set_acceleration(uint8_t id, uint32_t acceleration)
{
    if (!dxl_initialized) {
        ESP_LOGE(TAG, "Dynamixel not initialized");
        return false;
    }
    
    ESP_LOGI(TAG, "Setting acceleration for servo ID %d: %d", id, (int)acceleration);
    
    uint8_t parameters[6];
    parameters[0] = DXL_LOBYTE(DXL_ADDR_PROFILE_ACCELERATION);
    parameters[1] = DXL_HIBYTE(DXL_ADDR_PROFILE_ACCELERATION);
    parameters[2] = DXL_LOBYTE(DXL_LOWORD(acceleration));
    parameters[3] = DXL_HIBYTE(DXL_LOWORD(acceleration));
    parameters[4] = DXL_LOBYTE(DXL_HIWORD(acceleration));
    parameters[5] = DXL_HIBYTE(DXL_HIWORD(acceleration));
    
    dxl_send_packet(id, DXL_INST_WRITE, parameters, 6);
    
    uint8_t error = 0;
    uint8_t recv_params[16] = {0};
    uint16_t recv_length = 0;
    
    int result = dxl_read_status_packet(&error, recv_params, &recv_length);
    
    if (result == DXL_COMM_SUCCESS && error == 0) {
        ESP_LOGI(TAG, "Acceleration set for servo ID %d", id);
        return true;
    } else {
        ESP_LOGE(TAG, "Failed to set acceleration for servo ID %d", id);
        return false;
    }
}

bool dxl_read_error(uint8_t id, uint8_t *error_code)
{
    if (!dxl_initialized) {
        ESP_LOGE(TAG, "Dynamixel not initialized");
        return false;
    }
    
    if (error_code == NULL) {
        ESP_LOGE(TAG, "NULL pointer provided to dxl_read_error");
        return false;
    }
    
    uint8_t parameters[4];
    parameters[0] = DXL_LOBYTE(DXL_ADDR_HARDWARE_ERROR);
    parameters[1] = DXL_HIBYTE(DXL_ADDR_HARDWARE_ERROR);
    parameters[2] = DXL_LOBYTE(1); // Read 1 byte
    parameters[3] = DXL_HIBYTE(1);
    
    dxl_send_packet(id, DXL_INST_READ, parameters, 4);
    
    uint8_t error = 0;
    uint8_t recv_params[16] = {0};
    uint16_t recv_length = 0;
    
    int result = dxl_read_status_packet(&error, recv_params, &recv_length);
    
    if (result == DXL_COMM_SUCCESS) {
        if (recv_length >= 1) {
            *error_code = recv_params[0];
            ESP_LOGI(TAG, "Hardware error status of servo ID %d: 0x%02X", id, *error_code);
            return true;
        } else {
            ESP_LOGE(TAG, "Did not receive enough data for error status");
            return false;
        }
    } else {
        ESP_LOGE(TAG, "Failed to read error status for servo ID %d", id);
        return false;
    }
}

bool dxl_read_register(uint8_t id, uint16_t address, uint8_t size, uint32_t *value)
{
    if (!dxl_initialized) {
        ESP_LOGE(TAG, "Dynamixel not initialized");
        return false;
    }
    
    if (value == NULL || size == 0 || size > 4) {
        ESP_LOGE(TAG, "Invalid parameters to dxl_read_register");
        return false;
    }
    
    uint8_t parameters[4];
    parameters[0] = DXL_LOBYTE(address);
    parameters[1] = DXL_HIBYTE(address);
    parameters[2] = DXL_LOBYTE(size);
    parameters[3] = DXL_HIBYTE(size);
    
    dxl_send_packet(id, DXL_INST_READ, parameters, 4);
    
    uint8_t error = 0;
    uint8_t recv_params[16] = {0};
    uint16_t recv_length = 0;
    
    int result = dxl_read_status_packet(&error, recv_params, &recv_length);
    
    if (result == DXL_COMM_SUCCESS && error == 0) {
        if (recv_length >= size) {
            *value = 0;
            for (uint8_t i = 0; i < size; i++) {
                *value |= ((uint32_t)recv_params[i]) << (8 * i);
            }
            ESP_LOGI(TAG, "Read register 0x%04X from servo ID %d: 0x%08x", address, id, (unsigned int)*value);
            return true;
        } else {
            ESP_LOGE(TAG, "Did not receive enough data for register read");
            return false;
        }
    } else {
        ESP_LOGE(TAG, "Failed to read register from servo ID %d", id);
        return false;
    }
}

bool dxl_write_register(uint8_t id, uint16_t address, uint8_t size, uint32_t value)
{
    if (!dxl_initialized) {
        ESP_LOGE(TAG, "Dynamixel not initialized");
        return false;
    }
    
    if (size == 0 || size > 4) {
        ESP_LOGE(TAG, "Invalid size parameter to dxl_write_register");
        return false;
    }
    
    uint8_t parameters[2 + 4]; // Maximum: 2 bytes for address + 4 bytes for value
    parameters[0] = DXL_LOBYTE(address);
    parameters[1] = DXL_HIBYTE(address);
    
    // Write bytes in little-endian order
    for (uint8_t i = 0; i < size; i++) {
        parameters[2 + i] = (value >> (8 * i)) & 0xFF;
    }
    
    dxl_send_packet(id, DXL_INST_WRITE, parameters, 2 + size);
    
    uint8_t error = 0;
    uint8_t recv_params[16] = {0};
    uint16_t recv_length = 0;
    
    int result = dxl_read_status_packet(&error, recv_params, &recv_length);
    
    if (result == DXL_COMM_SUCCESS && error == 0) {
        ESP_LOGI(TAG, "Wrote 0x%08x to register 0x%04X of servo ID %d", (unsigned int)value, address, id);
        return true;
    } else {
        ESP_LOGE(TAG, "Failed to write register to servo ID %d", id);
        return false;
    }
} 