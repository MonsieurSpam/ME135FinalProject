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

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PORT_HANDLER_ESP32_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PORT_HANDLER_ESP32_H_

#include "port_handler.h"

int     portHandlerESP32    (const char *port_name);

uint8_t openPortESP32       (int port_num);
void    closePortESP32      (int port_num);
void    clearPortESP32      (int port_num);

void    setPortNameESP32    (int port_num, const char *port_name);
char   *getPortNameESP32    (int port_num);

uint8_t setBaudRateESP32    (int port_num, const int baudrate);
int     getBaudRateESP32    (int port_num);

int     readPortESP32       (int port_num, uint8_t *packet, int length);
int     writePortESP32      (int port_num, uint8_t *packet, int length);

void    setPacketTimeoutESP32     (int port_num, uint16_t packet_length);
void    setPacketTimeoutMSecESP32 (int port_num, double msec);
uint8_t isPacketTimeoutESP32      (int port_num);

double  getCurrentTimeESP32       ();
double  getTimeSinceStartESP32    (int port_num);

int     getBytesAvailableESP32    (int port_num);

#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PORT_HANDLER_ESP32_H_ */ 