/*******************************************************************************
* Copyright 2023 Custom ESP32 Implementation
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Custom ESP32 adaptation */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "port_handler_esp32.h"

// Default settings for ESP32
#define UART_NUM UART_NUM_2
#define BUF_SIZE 1024
#define DXL_DIR_PIN GPIO_NUM_21

// TX/RX Direction control 
#define TX_MODE 1
#define RX_MODE 0

typedef struct
{
    uart_port_t    uart_num;
    int            baudrate;
    gpio_num_t     dir_pin;
    char           port_name[100];
    
    int64_t        packet_start_time;
    double         packet_timeout;
    double         tx_time_per_byte;
} PortData;

static PortData *portData;
static const char *TAG = "PortHandler";

int portHandlerESP32(const char *port_name)
{
    int port_num;

    if (portData == NULL)
    {
        port_num = 0;
        g_used_port_num = 1;
        portData = (PortData *)calloc(1, sizeof(PortData));
        g_is_using = (uint8_t*)calloc(1, sizeof(uint8_t));
    }
    else
    {
        for (port_num = 0; port_num < g_used_port_num; port_num++)
        {
            if (!strcmp(portData[port_num].port_name, port_name))
                break;
        }

        if (port_num == g_used_port_num)
        {
            for (port_num = 0; port_num < g_used_port_num; port_num++)
            {
                if (portData[port_num].uart_num == UART_NUM_MAX)
                    break;
            }

            if (port_num == g_used_port_num)
            {
                g_used_port_num++;
                portData = (PortData*)realloc(portData, g_used_port_num * sizeof(PortData));
                g_is_using = (uint8_t*)realloc(g_is_using, g_used_port_num * sizeof(uint8_t));
            }
        }
        else
        {
            ESP_LOGI(TAG, "The port number %d has same device name... reinitialize port number %d!", port_num, port_num);
        }
    }

    portData[port_num].uart_num = UART_NUM_MAX;  // Invalid UART port
    portData[port_num].baudrate = DEFAULT_BAUDRATE;
    portData[port_num].dir_pin = DXL_DIR_PIN;  // Default direction pin
    portData[port_num].packet_start_time = 0;
    portData[port_num].packet_timeout = 0.0;
    portData[port_num].tx_time_per_byte = 0.0;

    g_is_using[port_num] = False;

    setPortNameESP32(port_num, port_name);

    return port_num;
}

uint8_t openPortESP32(int port_num)
{
    return setBaudRateESP32(port_num, portData[port_num].baudrate);
}

void closePortESP32(int port_num)
{
    if (portData[port_num].uart_num < UART_NUM_MAX)
    {
        uart_driver_delete(portData[port_num].uart_num);
        portData[port_num].uart_num = UART_NUM_MAX;
    }
}

void clearPortESP32(int port_num)
{
    uart_flush(portData[port_num].uart_num);
}

void setPortNameESP32(int port_num, const char *port_name)
{
    strcpy(portData[port_num].port_name, port_name);
    
    // Parse UART number from port name (e.g., "/dev/uart/2" -> UART_NUM_2)
    if (strstr(port_name, "/dev/uart/") != NULL) {
        int uart_num = atoi(port_name + strlen("/dev/uart/"));
        if (uart_num >= 0 && uart_num < UART_NUM_MAX) {
            portData[port_num].uart_num = uart_num;
        }
    }
}

char *getPortNameESP32(int port_num)
{
    return portData[port_num].port_name;
}

uint8_t setBaudRateESP32(int port_num, const int baudrate)
{
    closePortESP32(port_num);
    
    portData[port_num].baudrate = baudrate;
    
    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0
    };
    
    // Configure GPIO for direction control
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << portData[port_num].dir_pin),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);
    gpio_set_level(portData[port_num].dir_pin, RX_MODE);  // Default to RX mode
    
    // Install UART driver and set pins
    esp_err_t ret = uart_driver_install(portData[port_num].uart_num, BUF_SIZE * 2, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %d", ret);
        return False;
    }
    
    ret = uart_param_config(portData[port_num].uart_num, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART parameters: %d", ret);
        return False;
    }
    
    // Set UART pins based on UART number 
    // For this implementation, user should manually set pins using ESP-IDF APIs
    // before calling setBaudRate
    
    // Calculate tx_time_per_byte
    portData[port_num].tx_time_per_byte = (1000.0 / (double)baudrate) * 10.0;
    
    return True;
}

int getBaudRateESP32(int port_num)
{
    return portData[port_num].baudrate;
}

int getBytesAvailableESP32(int port_num)
{
    size_t bytes_available = 0;
    uart_get_buffered_data_len(portData[port_num].uart_num, &bytes_available);
    return (int)bytes_available;
}

int readPortESP32(int port_num, uint8_t *packet, int length)
{
    int rxlen = 0;
    rxlen = uart_read_bytes(portData[port_num].uart_num, packet, length, portMAX_DELAY);
    return rxlen;
}

int writePortESP32(int port_num, uint8_t *packet, int length)
{
    // Set to TX mode
    gpio_set_level(portData[port_num].dir_pin, TX_MODE);
    
    // Small delay to ensure direction pin has settled
    vTaskDelay(1);
    
    // Write data
    int txlen = uart_write_bytes(portData[port_num].uart_num, (const char*)packet, length);
    
    // Wait for TX to complete
    uart_wait_tx_done(portData[port_num].uart_num, portMAX_DELAY);
    
    // Set back to RX mode
    gpio_set_level(portData[port_num].dir_pin, RX_MODE);
    
    // Small delay to ensure direction pin has settled
    vTaskDelay(1);
    
    return txlen;
}

void setPacketTimeoutESP32(int port_num, uint16_t packet_length)
{
    portData[port_num].packet_start_time = esp_timer_get_time();
    portData[port_num].packet_timeout = (portData[port_num].tx_time_per_byte * (double)packet_length) + 5.0;
}

void setPacketTimeoutMSecESP32(int port_num, double msec)
{
    portData[port_num].packet_start_time = esp_timer_get_time();
    portData[port_num].packet_timeout = msec;
}

uint8_t isPacketTimeoutESP32(int port_num)
{
    if (getTimeSinceStartESP32(port_num) > portData[port_num].packet_timeout)
    {
        portData[port_num].packet_timeout = 0;
        return True;
    }
    return False;
}

double getCurrentTimeESP32()
{
    // Get current time in microseconds and convert to milliseconds
    return (double)esp_timer_get_time() / 1000.0;
}

double getTimeSinceStartESP32(int port_num)
{
    double time_since_start;
    
    time_since_start = getCurrentTimeESP32() - ((double)portData[port_num].packet_start_time / 1000.0);
    if (time_since_start < 0.0)
        time_since_start = 0.0;
        
    return time_since_start;
}

// Handler functions that link to the ESP32 implementations
// These are the functions required by the Dynamixel SDK
int     portHandler         (const char *port_name)             { return portHandlerESP32(port_name); }
uint8_t openPort            (int port_num)                      { return openPortESP32(port_num); }
void    closePort           (int port_num)                      { closePortESP32(port_num); }
void    clearPort           (int port_num)                      { clearPortESP32(port_num); }
void    setPortName         (int port_num, const char *port_name) { setPortNameESP32(port_num, port_name); }
char   *getPortName         (int port_num)                      { return getPortNameESP32(port_num); }
uint8_t setBaudRate         (int port_num, const int baudrate)  { return setBaudRateESP32(port_num, baudrate); }
int     getBaudRate         (int port_num)                      { return getBaudRateESP32(port_num); }
int     getBytesAvailable   (int port_num)                      { return getBytesAvailableESP32(port_num); }
int     readPort            (int port_num, uint8_t *packet, int length) { return readPortESP32(port_num, packet, length); }
int     writePort           (int port_num, uint8_t *packet, int length) { return writePortESP32(port_num, packet, length); }
void    setPacketTimeout    (int port_num, uint16_t packet_length) { setPacketTimeoutESP32(port_num, packet_length); }
void    setPacketTimeoutMSec(int port_num, double msec)         { setPacketTimeoutMSecESP32(port_num, msec); }
uint8_t isPacketTimeout     (int port_num)                      { return isPacketTimeoutESP32(port_num); } 