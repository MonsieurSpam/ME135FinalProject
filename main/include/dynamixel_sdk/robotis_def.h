/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
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

/* Author: Ryu Woon Jung (Leon) */

#ifndef ROBOTIS_DEF_H_
#define ROBOTIS_DEF_H_

// For ESP32, include standard C types instead of redefining them
#include <stdint.h>
#include <stdbool.h>

// Define WINDECLSPEC as empty for ESP32 (no need for DLL export/import)
#define WINDECLSPEC

// Define True/False values
#ifndef True
#define True  1
#endif

#ifndef False
#define False 0
#endif

#ifndef HIGH
#define HIGH 1
#endif

#ifndef LOW
#define LOW  0
#endif

/* I/O modes */
#ifndef INPUT
#define INPUT 0
#endif

#ifndef OUTPUT
#define OUTPUT 1
#endif

#ifndef VOLTAGE_12_V
#define VOLTAGE_12_V 12.0
#endif

#ifndef VOLTAGE_5_V
#define VOLTAGE_5_V  5.0
#endif

#ifndef VOLTAGE_3_3_V
#define VOLTAGE_3_3_V  3.3
#endif

#endif /* ROBOTIS_DEF_H_ */
