/* -----------------------------------------------------------------------------
 * Copyright (c) 2022 Arm Limited (or its affiliates). All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *
 * $Date:        24. March 2022
 *
 * Project:      WiFi Driver Header for MXCHIP EMW3080 WiFi Module 
 *               (SPI variant)
 * -------------------------------------------------------------------------- */

#ifndef WiFi_EMW3080_H__
#define WiFi_EMW3080_H__

#include "Driver_WiFi.h"

#include "WiFi_EMW3080_Config.h"

// Hardware dependent functions to be called by the user code

extern void WiFi_EMW3080_Pin_NOTIFY_Rising_Edge (void);
extern void WiFi_EMW3080_Pin_FLOW_Rising_Edge   (void);

// Structure exported by the driver Driver_WiFin (default: Driver_WiFi0)

extern ARM_DRIVER_WIFI ARM_Driver_WiFi_(WIFI_EMW3080_DRV_NUM);

#endif
