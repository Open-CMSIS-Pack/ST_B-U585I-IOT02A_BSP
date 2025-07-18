/* -----------------------------------------------------------------------------
 * Copyright (c) 2022-2025 Arm Limited (or its affiliates). All rights reserved.
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
 * $Date:               18. July 2025
 * $Revision:           V2.0
 *
 * Driver:              Driver_WiFin (n = WIFI_EMW3080_DRV_NUM value)
 * Project:             WiFi Driver for MXCHIP EMW3080 WiFi Module (SPI variant)
 * Documentation:       ./Documentation/WiFi_EMW3080_README.md
 * --------------------------------------------------------------------------
 * Use the WiFi_EMW3080_Config.h file for compile time configuration 
 * of this driver.
 * -------------------------------------------------------------------------- */

/* History:
 *  Version 2.0
 *    - Changed mx_wifi component driver and configuration file location
 *  Version 1.1
 *    - Updated to work with EMW3080B MXCHIP WiFi module firmware v2.3.4 (rc 13)
 *  Version 1.0
 *    - Initial version
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "stm32u5xx_hal.h"

#include "RTE_Components.h"

#include "cmsis_os2.h"
#include "cmsis_compiler.h"

#include "WiFi_EMW3080.h"

#ifdef   RTE_Compiler_EventRecorder
#include "EventRecorder.h"              // Keil.ARM Compiler::Compiler:Event Recorder
#endif

#include "mx_wifi_conf.h"
#include "mx_wifi.h"
#include "mx_address.h"
#include "mx_wifi_io.h"

// Check Mx WiFi configuration
#if (MX_WIFI_USE_SPI == 0)
#error This driver only supports SPI interface (MX_WIFI_USE_SPI in the mx_wifi_conf.h file must be set to 1) !!!
#endif
#if (MX_WIFI_USE_CMSIS_OS == 0)
#error This driver requires CMSIS RTOS2 (MX_WIFI_USE_CMSIS_OS in the mx_wifi_conf.h file must be set to 1) !!!
#endif
#if (MX_WIFI_NETWORK_BYPASS_MODE == 1)
#error This driver does not support bypass or pass-through mode (MX_WIFI_NETWORK_BYPASS_MODE in the mx_wifi_conf.h file is ignored) !
#endif

// Backward compatibility defines
#ifndef WIFI_EMW3080_SOCKETS_RCV_RETRIES
#define WIFI_EMW3080_SOCKETS_RCV_RETRIES       (10)
#endif

// Hardware dependent functions --------

/**
  \fn            void WiFi_EMW3080_Pin_NOTIFY_Rising_Edge (void)
  \brief         Interrupt triggered by NOTIFY line state change to active state.
  \detail        This callback function must be called by external user code 
                 on interrupt when NOTIFY line changes to active state.
  \return        none
*/
void WiFi_EMW3080_Pin_NOTIFY_Rising_Edge (void) {
#ifdef MXCHIP_NOTIFY_Pin
  mxchip_WIFI_ISR(MXCHIP_NOTIFY_Pin);
#endif
}

/**
  \fn            void WiFi_EMW3080_Pin_FLOW_Rising_Edge (void)
  \brief         Interrupt triggered by FLOW line state change to active state.
  \detail        This callback function must be called by external user code 
                 on interrupt when FLOW line changes to active state.
  \return        none
*/
void WiFi_EMW3080_Pin_FLOW_Rising_Edge (void) {
#ifdef MXCHIP_FLOW_Pin
  mxchip_WIFI_ISR(MXCHIP_FLOW_Pin);
#endif
}

// WiFi Driver *****************************************************************

#define ARM_WIFI_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(2,0)         // Driver version

// Driver Version
static const ARM_DRIVER_VERSION driver_version = { ARM_WIFI_API_VERSION, ARM_WIFI_DRV_VERSION };

// Driver Capabilities
static const ARM_WIFI_CAPABILITIES driver_capabilities = { 
  1U,                                   // Station supported
  0U,                                   // Access Point not supported
  0U,                                   // Concurrent Station and Access Point not supported
  0U,                                   // WiFi Protected Setup (WPS) for Station not supported
  0U,                                   // WiFi Protected Setup (WPS) for Access Point not supported
  0U,                                   // Access Point: event not generated on Station connect
  0U,                                   // Access Point: event not generated on Station disconnect
  0U,                                   // Event not generated on Ethernet frame reception in bypass mode
  0U,                                   // Bypass or pass-through mode (Ethernet interface) not supported
  1U,                                   // IP (UDP/TCP) (Socket interface) supported
  0U,                                   // IPv6 (Socket interface) not supported
  1U,                                   // Ping (ICMP) supported
  0U                                    // Reserved (must be zero)
};

// Network to host byte order conversion
#if defined(__BIG_ENDIAN) || defined(__ARM_BIG_ENDIAN)
  #define htonl(v)                      (uint32_t)(v)
  #define htons(v)                      (uint16_t)(v)
  #define ntohl(v)                      (uint32_t)(v)
  #define ntohs(v)                      (uint16_t)(v)
#elif defined (__CC_ARM)
  /* ARM Compiler 4/5 */
  #define htonl(v)                      (uint32_t)(__rev(v))
  #define htons(v)                      (uint16_t)(__rev(v) >> 16)
  #define ntohl(v)                      (uint32_t)(__rev(v))
  #define ntohs(v)                      (uint16_t)(__rev(v) >> 16)
#else
  /* ARM Compiler 6 */
  #define htonl(v)                      __builtin_bswap32((uint32_t)(v))
  #define htons(v)                      __builtin_bswap16((uint16_t)(v))
  #define ntohl(v)                      __builtin_bswap32((uint32_t)(v))
  #define ntohs(v)                      __builtin_bswap16((uint16_t)(v))
#endif

typedef struct mx_sockaddr_storage      SOCKADDR_STORAGE;
typedef struct mx_sockaddr_in           SOCKADDR_IN;

// sock_attr access protection mutex
static osMutexId_t                      mutex_id_sock_attr = NULL;

// Status change event flags
static osEventFlagsId_t                 ef_id_sta_status   = NULL;

// Local variables and structures
static uint8_t                          driver_initialized = 0U;
static ARM_WIFI_SignalEvent_t           signal_event_fn    = NULL;

static uint8_t                          scan_buf[WIFI_EMW3080_SCAN_BUF_SIZE] __ALIGNED(4);
static MX_WIFIObject_t                 *ptrMX_WIFIObject   = NULL;

// Socket attributes
static struct {
  uint8_t  ionbio;
  int8_t   type;
  struct {
    uint16_t created    :  1;
    uint16_t bound      :  1;
    uint16_t listening  :  1;
    uint16_t connecting :  1;
    uint16_t connected  :  1;
    uint16_t reserved   : 11;
  } flags;
  uint32_t rcvtimeo;
  uint32_t sndtimeo;
  uint8_t  local_ip [4];
  uint8_t  remote_ip[4];
  uint16_t local_port;
  uint16_t remote_port;
  uint8_t  rx_ip[4];
  uint16_t rx_port;
  uint16_t rx_buf_available_len;
  uint8_t  rx_buf [WIFI_EMW3080_SOCKETS_RX_BUF_SIZE];
} sock_attr[WIFI_EMW3080_SOCKETS_NUM];

// Mutex responsible for protecting sock_attr access 
static const osMutexAttr_t mutex_sock_attr = {
  "Mutex_sock_attr",                    // Mutex name
  osMutexPrioInherit,                   // attr_bits
  NULL,                                 // Memory for control block
  0U                                    // Size for control block
};

// Helper Functions

// Convert error code: STM32Cube Mx WiFi Driver -> CMSIS WiFi Driver
static int32_t ConvertErrorCodeMxToCmsis (int32_t mx_error_code) {
  int32_t ret;

  switch (mx_error_code) {
    case MX_WIFI_STATUS_OK:
      ret = ARM_DRIVER_OK;
      break;
    case MX_WIFI_STATUS_ERROR:
      ret = ARM_DRIVER_ERROR;
      break;
    case MX_WIFI_STATUS_TIMEOUT:
      ret = ARM_DRIVER_ERROR_TIMEOUT;
      break;
    case MX_WIFI_STATUS_IO_ERROR:
      ret = ARM_DRIVER_ERROR;
      break;
    case MX_WIFI_STATUS_PARAM_ERROR:
      ret = ARM_DRIVER_ERROR_PARAMETER;
      break;
    default:
      ret = ARM_DRIVER_ERROR;
      break;
  }

  return ret;
}

// Convert security type: CMSIS WiFi Driver -> STM32Cube Mx WiFi Driver
static MX_WIFI_SecurityType_t ConvertSecurityTypeCmsisToMx (uint8_t cmsis_security_type) {
  MX_WIFI_SecurityType_t ret;

  switch (cmsis_security_type) {
    case ARM_WIFI_SECURITY_OPEN:
      ret = MX_WIFI_SEC_NONE;
      break;
    case ARM_WIFI_SECURITY_WEP:
      ret = MX_WIFI_SEC_WEP;
      break;
    case ARM_WIFI_SECURITY_WPA:
      ret = MX_WIFI_SEC_WPA_AES;
      break;
    case ARM_WIFI_SECURITY_WPA2:
    case ARM_WIFI_SECURITY_UNKNOWN:
    default:
      ret = MX_WIFI_SEC_WPA2_AES;
      break;
  }

  return ret;
}

// Convert security type: STM32Cube Mx WiFi Driver -> CMSIS WiFi Driver
static uint8_t ConvertSecurityTypeMxToCmsis (mwifi_security_t mx_security_type) {
  uint8_t ret;

  switch (mx_security_type) {
    case MX_WIFI_SEC_NONE:
      ret = ARM_WIFI_SECURITY_OPEN;
      break;
    case MX_WIFI_SEC_WEP:
      ret = ARM_WIFI_SECURITY_WEP;
      break;
    case MX_WIFI_SEC_WPA_TKIP:
    case MX_WIFI_SEC_WPA_AES:
      ret = ARM_WIFI_SECURITY_WPA;
      break;
    case MX_WIFI_SEC_WPA2_TKIP:
    case MX_WIFI_SEC_WPA2_AES:
    case MX_WIFI_SEC_WPA2_MIXED:
      ret = ARM_WIFI_SECURITY_WPA2;
      break;
    case MX_WIFI_SEC_AUTO:
    default:
      ret = ARM_WIFI_SECURITY_UNKNOWN;
      break;
  }

  return ret;
}

// Convert socket error code: STM32Cube Mx WiFi Driver -> CMSIS WiFi Driver
static int32_t ConvertSocketErrorCodeMxToCmsis (int32_t mx_socket_error_code) {
  int32_t ret;

  switch (mx_socket_error_code) {
    case MX_WIFI_STATUS_OK:
      ret = 0;
      break;
    case MX_WIFI_STATUS_ERROR:
      ret = ARM_SOCKET_ERROR;
      break;
    case MX_WIFI_STATUS_TIMEOUT:
      ret = ARM_SOCKET_ETIMEDOUT;
      break;
    case MX_WIFI_STATUS_IO_ERROR:
      ret = ARM_SOCKET_ERROR;
      break;
    case MX_WIFI_STATUS_PARAM_ERROR:
      ret = ARM_SOCKET_EINVAL;
      break;
    default:
      ret = ARM_SOCKET_ERROR;
      break;
  }

  return ret;
}

/**
  \fn            void ResetVariables (void)
  \brief         Function that resets to all local variables to default values.
*/
static void ResetVariables (void) {

  memset((void *)scan_buf,  0, sizeof(scan_buf));
  memset((void *)sock_attr, 0, sizeof(sock_attr));

  // Set default rcvtimeo
  for (int32_t i = 0; i < WIFI_EMW3080_SOCKETS_NUM; i++) {
    sock_attr[i].rcvtimeo = (uint32_t)WIFI_EMW3080_SOCKETS_RCVTIMEO;
  }
}

/**
  * @brief                   mxchip wifi status change callback
  * @param  cate             status cate
  * @param  status           status
  * @param  arg              user arguments
  */
static void mx_wifi_status_changed(uint8_t cate, uint8_t status, void *arg) {
  (void)arg;

  if (cate == (uint8_t)MC_STATION) {
    if (ef_id_sta_status != NULL) {
      (void)osEventFlagsSet(ef_id_sta_status, status);
    }
  }
}

// Driver Functions

/**
  \fn            ARM_DRIVER_VERSION WiFi_GetVersion (void)
  \brief         Get driver version.
  \return        ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION WiFi_GetVersion (void) {
  return driver_version;
}

/**
  \fn            ARM_WIFI_CAPABILITIES WiFi_GetCapabilities (void)
  \brief         Get driver capabilities.
  \return        ARM_WIFI_CAPABILITIES
*/
static ARM_WIFI_CAPABILITIES WiFi_GetCapabilities (void) { 
  return driver_capabilities;
}

/**
  \fn            int32_t WiFi_Initialize (ARM_WIFI_SignalEvent_t cb_event)
  \brief         Initialize WiFi Module.
  \param[in]     cb_event Pointer to ARM_WIFI_SignalEvent_t
  \return        execution status
                   - ARM_DRIVER_OK                : Operation successful
                   - ARM_DRIVER_ERROR             : Operation failed
*/
static int32_t WiFi_Initialize (ARM_WIFI_SignalEvent_t cb_event) {
  int32_t ret, ret_mx;

  driver_initialized = 0U;
  signal_event_fn = cb_event;           // Update pointer to callback function

  ret = ARM_DRIVER_OK;

  if (mxwifi_probe((void **)&ptrMX_WIFIObject) != 0) {
    ret = ARM_DRIVER_ERROR;
  }

  if (ret == ARM_DRIVER_OK) {
    if (ptrMX_WIFIObject->Runtime.interfaces == 0U) {
      /* WiFi module hardware reboot */
      ret_mx = MX_WIFI_HardResetModule(ptrMX_WIFIObject);
      if (ret_mx != MX_WIFI_STATUS_OK) {
        ret = ConvertErrorCodeMxToCmsis(ret_mx);
      }
    }
  }

  if (ret == ARM_DRIVER_OK) {
    /* Init the WiFi module */
    ret_mx = MX_WIFI_Init(ptrMX_WIFIObject);
    if (ret_mx != MX_WIFI_STATUS_OK) {
      ret = ConvertErrorCodeMxToCmsis(ret_mx);
    }
  }

  if (ret == ARM_DRIVER_OK) {
    /* Initialize default network settings */
    ptrMX_WIFIObject->NetSettings.DHCP_IsEnabled = 1U;

    ResetVariables();
  }

  if (ret == ARM_DRIVER_OK) {
    if (mutex_id_sock_attr == NULL) {
      mutex_id_sock_attr = osMutexNew(&mutex_sock_attr);
      if (mutex_id_sock_attr == NULL) {
        ret = ARM_DRIVER_ERROR;
      }
    }
  }

  if (ret == ARM_DRIVER_OK) {
    if (ef_id_sta_status == NULL) {
      ef_id_sta_status = osEventFlagsNew(NULL);
      if (ef_id_sta_status == NULL) {
        ret = ARM_DRIVER_ERROR;
      }
    }
  }

  if (ret == ARM_DRIVER_OK) {
    /* DHCP is enabled by default */
    ptrMX_WIFIObject->NetSettings.DHCP_IsEnabled = 1U;
  }

  if (ret == ARM_DRIVER_OK) {
    driver_initialized = 1U;
  }

  return ret;
}

/**
  \fn            int32_t WiFi_Uninitialize (void)
  \brief         De-initialize WiFi Module.
  \return        execution status
                   - ARM_DRIVER_OK                : Operation successful
                   - ARM_DRIVER_ERROR             : Operation failed
*/
static int32_t WiFi_Uninitialize (void) {
  int32_t ret, ret_mx;

  ret = ARM_DRIVER_OK;

  if (mutex_id_sock_attr != NULL) {
    if (osMutexDelete(mutex_id_sock_attr) == osOK) {
      mutex_id_sock_attr = NULL;
    } else {
      ret = ARM_DRIVER_ERROR;
    }
  }

  if (ef_id_sta_status != NULL) {
    if (osEventFlagsDelete(ef_id_sta_status) == osOK) {
      ef_id_sta_status = NULL;
    } else {
      ret = ARM_DRIVER_ERROR;
    }
  }

  if (ret == ARM_DRIVER_OK) {
    ret_mx = MX_WIFI_DeInit(ptrMX_WIFIObject);
    if (ret_mx == 0) {
      ptrMX_WIFIObject = NULL;
    } else {
      ret = ConvertErrorCodeMxToCmsis(ret_mx);
    }
  }

  if (ret == ARM_DRIVER_OK) {
    driver_initialized = 0U;
    ResetVariables();
  }

  return ret;
}

/**
  \fn            int32_t WiFi_PowerControl (ARM_POWER_STATE state)
  \brief         Control WiFi Module Power.
  \param[in]     state     Power state
                   - ARM_POWER_OFF                : Power off: no operation possible
                   - ARM_POWER_LOW                : Low-power mode: sleep or deep-sleep depending on ARM_WIFI_LP_TIMER option set
                   - ARM_POWER_FULL               : Power on: full operation at maximum performance
  \return        execution status
                   - ARM_DRIVER_OK                : Operation successful
                   - ARM_DRIVER_ERROR             : Operation failed
                   - ARM_DRIVER_ERROR_UNSUPPORTED : Operation not supported
                   - ARM_DRIVER_ERROR_PARAMETER   : Parameter error (invalid state)
*/
static int32_t WiFi_PowerControl (ARM_POWER_STATE state) {
  int32_t ret, ret_mx;

  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  ret = ARM_DRIVER_OK;

  switch (state) {
    case ARM_POWER_OFF:
      // Module does not support power-off
      ret = ARM_DRIVER_ERROR_UNSUPPORTED;
      break;

    case ARM_POWER_LOW:
      ret_mx = MX_WIFI_station_powersave(ptrMX_WIFIObject, 1);
      if (ret_mx != MX_WIFI_STATUS_OK) {
        ret = ConvertErrorCodeMxToCmsis(ret_mx);
      }
      break;

    case ARM_POWER_FULL:
      ret_mx = MX_WIFI_station_powersave(ptrMX_WIFIObject, 0);
      if (ret_mx != MX_WIFI_STATUS_OK) {
        ret = ConvertErrorCodeMxToCmsis(ret_mx);
      }
      break;

    default:
      ret = ARM_DRIVER_ERROR_PARAMETER;
      break;
  }

  return ret;
}

/**
  \fn            int32_t WiFi_GetModuleInfo (char *module_info, uint32_t max_len)
  \brief         Get Module information.
  \param[out]    module_info Pointer to character buffer were info string will be returned
  \param[in]     max_len     Maximum length of string to return (including null terminator)
  \return        execution status
                   - ARM_DRIVER_OK                : Operation successful
                   - ARM_DRIVER_ERROR             : Operation failed
                   - ARM_DRIVER_ERROR_UNSUPPORTED : Operation not supported
                   - ARM_DRIVER_ERROR_PARAMETER   : Parameter error (NULL module_info pointer or max_len equals to 0)
*/
static int32_t WiFi_GetModuleInfo (char *module_info, uint32_t max_len) {
  char    *str;
  uint32_t len, max_remain_len;

  if ((module_info == NULL) || (max_len == 0U)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  str    = module_info;
  str[0] = '\0';
  len    = 0U;
  max_remain_len = max_len - 1U;

  /* Compose the WiFi module information: Product_Name' 'Product_ID' 'FW_Rev */
  /* Product_Name */
  if (max_remain_len > 0U) {
    (void)strncpy(str, (char *)ptrMX_WIFIObject->SysInfo.Product_Name, max_remain_len);
    str[max_remain_len] = '\0';
    len = strlen(str);
    max_remain_len -= len;
  }

  /* Space character ' ' */
  if (max_remain_len > 0U) {
    str[len] = ' ';
    len++;
    str[len] = '\0';
    max_remain_len --;
  }

  /* Product_ID */
  if (max_remain_len > 0U) {
    (void)strncpy(&str[len], (char *)ptrMX_WIFIObject->SysInfo.Product_ID, max_remain_len);
    str[len+max_remain_len] = '\0';
    len = strlen(str);
    max_remain_len = (max_len - 1U) - len;
  }

  /* Space character ' ' */
  if (max_remain_len > 0U) {
    str[len] = ' ';
    len++;
    str[len] = '\0';
    max_remain_len --;
  }

  /* FW_Rev */
  if (max_remain_len > 0U) {
    (void)strncpy(&str[len], (char *)ptrMX_WIFIObject->SysInfo.FW_Rev, max_remain_len);
    str[len+max_remain_len] = '\0';
  }

  return ARM_DRIVER_OK;
}

/**
  \fn            int32_t WiFi_SetOption (uint32_t interface, uint32_t option, const void *data, uint32_t len)
  \brief         Set WiFi Module Options.
  \param[in]     interface Interface (0 = Station, 1 = Access Point)
  \param[in]     option    Option to set
  \param[in]     data      Pointer to data relevant to selected option
  \param[in]     len       Length of data (in bytes)
  \return        execution status
                   - ARM_DRIVER_OK                : Operation successful
                   - ARM_DRIVER_ERROR             : Operation failed
                   - ARM_DRIVER_ERROR_UNSUPPORTED : Operation not supported
                   - ARM_DRIVER_ERROR_PARAMETER   : Parameter error (invalid interface, NULL data pointer or len less than option specifies)
*/
static int32_t WiFi_SetOption (uint32_t interface, uint32_t option, const void *data, uint32_t len) {
  int32_t ret;

  if (interface != 0U) {
    // Access Point not supported
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if ((data == NULL) || (len < 4U)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  ret = ARM_DRIVER_OK;

  switch (option) {
    case ARM_WIFI_IP:                   // Station/AP Set IPv4 static/assigned address;           data = &ip,       len =  4, uint8_t[4]
      memcpy((void *)ptrMX_WIFIObject->NetSettings.IP_Addr, data, 4);
      break;
    case ARM_WIFI_IP_SUBNET_MASK:       // Station/AP Set IPv4 subnet mask;                       data = &mask,     len =  4, uint8_t[4]
      memcpy((void *)ptrMX_WIFIObject->NetSettings.IP_Mask, data, 4);
      break;
    case ARM_WIFI_IP_GATEWAY:           // Station/AP Set IPv4 gateway address;                   data = &ip,       len =  4, uint8_t[4]
      memcpy((void *)ptrMX_WIFIObject->NetSettings.Gateway_Addr, data, 4);
      break;
    case ARM_WIFI_IP_DNS1:              // Station/AP Set IPv4 primary   DNS address;             data = &ip,       len =  4, uint8_t[4]
      memcpy((void *)ptrMX_WIFIObject->NetSettings.DNS1, data, 4);
      break;
    case ARM_WIFI_IP_DHCP:              // Station/AP Set IPv4 DHCP client/server enable/disable; data = &dhcp,     len =  4, uint32_t: 0 = disable, non-zero = enable (default)
      if (*((const uint32_t *)data) != 0U) {
        ptrMX_WIFIObject->NetSettings.DHCP_IsEnabled = 1U;
      } else {
        ptrMX_WIFIObject->NetSettings.DHCP_IsEnabled = 0U;
      }
      break;
    case ARM_WIFI_BSSID:                // Station/AP Set BSSID of AP to connect or of AP;        data = &bssid,    len =  6, uint8_t[6]
    case ARM_WIFI_TX_POWER:             // Station/AP Set transmit power;                         data = &power,    len =  4, uint32_t: 0 .. 20 [dBm]
    case ARM_WIFI_LP_TIMER:             // Station    Set low-power deep-sleep time;              data = &time,     len =  4, uint32_t [seconds]: 0 = disable (default)
    case ARM_WIFI_DTIM:                 // Station/AP Set DTIM interval;                          data = &dtim,     len =  4, uint32_t [beacons]
    case ARM_WIFI_BEACON:               //         AP Set beacon interval;                        data = &interval, len =  4, uint32_t [ms]
    case ARM_WIFI_MAC:                  // Station/AP Set MAC;                                    data = &mac,      len =  6, uint8_t[6]
    case ARM_WIFI_IP_DNS2:              // Station/AP Set IPv4 secondary DNS address;             data = &ip,       len =  4, uint8_t[4]
    case ARM_WIFI_IP_DHCP_POOL_BEGIN:   //         AP Set IPv4 DHCP pool begin address;           data = &ip,       len =  4, uint8_t[4]
    case ARM_WIFI_IP_DHCP_POOL_END:     //         AP Set IPv4 DHCP pool end address;             data = &ip,       len =  4, uint8_t[4]
    case ARM_WIFI_IP_DHCP_LEASE_TIME:   //         AP Set IPv4 DHCP lease time;                   data = &time,     len =  4, uint32_t [seconds]
    case ARM_WIFI_IP6_GLOBAL:           // Station/AP Set IPv6 global address;                    data = &ip6,      len = 16, uint8_t[16]
    case ARM_WIFI_IP6_LINK_LOCAL:       // Station/AP Set IPv6 link local address;                data = &ip6,      len = 16, uint8_t[16]
    case ARM_WIFI_IP6_SUBNET_PREFIX_LEN:// Station/AP Set IPv6 subnet prefix length;              data = &len,      len =  4, uint32_t: 1 .. 127
    case ARM_WIFI_IP6_GATEWAY:          // Station/AP Set IPv6 gateway address;                   data = &ip6,      len = 16, uint8_t[16]
    case ARM_WIFI_IP6_DNS1:             // Station/AP Set IPv6 primary   DNS address;             data = &ip6,      len = 16, uint8_t[16]
    case ARM_WIFI_IP6_DNS2:             // Station/AP Set IPv6 secondary DNS address;             data = &ip6,      len = 16, uint8_t[16]
    case ARM_WIFI_IP6_DHCP_MODE:        // Station/AP Set IPv6 DHCPv6 client mode;                data = &mode,     len =  4, uint32_t: ARM_WIFI_IP6_DHCP_xxx (default Off)
    default:
      ret = ARM_DRIVER_ERROR_UNSUPPORTED;
      break;
  }

  return ret;
}

/**
  \fn            int32_t WiFi_GetOption (uint32_t interface, uint32_t option, void *data, uint32_t *len)
  \brief         Get WiFi Module Options.
  \param[in]     interface Interface (0 = Station, 1 = Access Point)
  \param[in]     option    Option to get
  \param[out]    data      Pointer to memory where data for selected option will be returned
  \param[in,out] len       Pointer to length of data (input/output)
                   - input: maximum length of data that can be returned (in bytes)
                   - output: length of returned data (in bytes)
  \return        execution status
                   - ARM_DRIVER_OK                : Operation successful
                   - ARM_DRIVER_ERROR             : Operation failed
                   - ARM_DRIVER_ERROR_UNSUPPORTED : Operation not supported
                   - ARM_DRIVER_ERROR_PARAMETER   : Parameter error (invalid interface, NULL data or len pointer, or *len less than option specifies)
*/
static int32_t WiFi_GetOption (uint32_t interface, uint32_t option, void *data, uint32_t *len) {
  int32_t  ret, ret_mx;
  uint32_t ipaddr;

  if (interface != 0U) {
    // Access Point not supported
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if ((data == NULL) || (len == NULL) || (*len < 4U)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  ret = ARM_DRIVER_OK;

  switch (option) {
    case ARM_WIFI_MAC:                  // Station/AP Get MAC;                                    data = &mac,      len =  6, uint8_t[6]
      if (*len >= 6U) {
        memcpy(data, ptrMX_WIFIObject->SysInfo.MAC, *len);
        *len = 6U;
      } else {
        ret = ARM_DRIVER_ERROR_PARAMETER;
      }
      break;
    case ARM_WIFI_IP:                   // Station/AP Get IPv4 static/assigned address;           data = &ip,       len =  4, uint8_t[4]
    case ARM_WIFI_IP_SUBNET_MASK:       // Station/AP Get IPv4 subnet mask;                       data = &mask,     len =  4, uint8_t[4]
    case ARM_WIFI_IP_GATEWAY:           // Station/AP Get IPv4 gateway address;                   data = &ip,       len =  4, uint8_t[4]
    case ARM_WIFI_IP_DNS1:              // Station/AP Get IPv4 primary   DNS address;             data = &ip,       len =  4, uint8_t[4]
      // If station is connected, get actual settings
      if (ptrMX_WIFIObject->NetSettings.IsConnected != 0) {
        ret_mx = MX_WIFI_GetIPAddress(ptrMX_WIFIObject, (uint8_t *)&ipaddr, (mwifi_if_t)interface);
        if (ret_mx != MX_WIFI_STATUS_OK) {
          ret = ConvertErrorCodeMxToCmsis(ret_mx);
        }
      }
      if (ret == ARM_DRIVER_OK) {
        switch (option) {
          case ARM_WIFI_IP:
            memcpy(data, ptrMX_WIFIObject->NetSettings.IP_Addr, 4);
            *len = 4U;
            break;
          case ARM_WIFI_IP_SUBNET_MASK:
            memcpy(data, ptrMX_WIFIObject->NetSettings.IP_Mask, 4);
            *len = 4U;
            break;
          case ARM_WIFI_IP_GATEWAY:
            memcpy(data, ptrMX_WIFIObject->NetSettings.Gateway_Addr, 4);
            *len = 4U;
            break;
          case ARM_WIFI_IP_DNS1:
            memcpy(data, ptrMX_WIFIObject->NetSettings.DNS1, 4);
            *len = 4U;
            break;
          default:
            ret = ARM_DRIVER_ERROR_UNSUPPORTED;
            break;
        }
      }
      break;
    case ARM_WIFI_IP_DHCP:              // Station/AP Get IPv4 DHCP client/server enable/disable; data = &dhcp,     len =  4, uint32_t: 0 = disable, non-zero = enable (default)
      if (ptrMX_WIFIObject->NetSettings.DHCP_IsEnabled != 0U) {
        *((uint32_t *)data) = 1U;
      } else {
        *((uint32_t *)data) = 0U;
      }
      *len = 4U;
      break;
    case ARM_WIFI_BSSID:                // Station/AP Get BSSID of AP to connect or of AP;        data = &bssid,    len =  6, uint8_t[6]
    case ARM_WIFI_TX_POWER:             // Station/AP Get transmit power;                         data = &power,    len =  4, uint32_t: 0 .. 20 [dBm]
    case ARM_WIFI_LP_TIMER:             // Station    Get low-power deep-sleep time;              data = &time,     len =  4, uint32_t [seconds]: 0 = disable (default)
    case ARM_WIFI_DTIM:                 // Station/AP Get DTIM interval;                          data = &dtim,     len =  4, uint32_t [beacons]
    case ARM_WIFI_BEACON:               //         AP Get beacon interval;                        data = &interval, len =  4, uint32_t [ms]
    case ARM_WIFI_IP_DNS2:              // Station/AP Get IPv4 secondary DNS address;             data = &ip,       len =  4, uint8_t[4]
    case ARM_WIFI_IP_DHCP_POOL_BEGIN:   //         AP Get IPv4 DHCP pool begin address;           data = &ip,       len =  4, uint8_t[4]
    case ARM_WIFI_IP_DHCP_POOL_END:     //         AP Get IPv4 DHCP pool end address;             data = &ip,       len =  4, uint8_t[4]
    case ARM_WIFI_IP_DHCP_LEASE_TIME:   //         AP Get IPv4 DHCP lease time;                   data = &time,     len =  4, uint32_t [seconds]
    case ARM_WIFI_IP6_GLOBAL:           // Station/AP Get IPv6 global address;                    data = &ip6,      len = 16, uint8_t[16]
    case ARM_WIFI_IP6_LINK_LOCAL:       // Station/AP Get IPv6 link local address;                data = &ip6,      len = 16, uint8_t[16]
    case ARM_WIFI_IP6_SUBNET_PREFIX_LEN:// Station/AP Get IPv6 subnet prefix length;              data = &len,      len =  4, uint32_t: 1 .. 127
    case ARM_WIFI_IP6_GATEWAY:          // Station/AP Get IPv6 gateway address;                   data = &ip6,      len = 16, uint8_t[16]
    case ARM_WIFI_IP6_DNS1:             // Station/AP Get IPv6 primary   DNS address;             data = &ip6,      len = 16, uint8_t[16]
    case ARM_WIFI_IP6_DNS2:             // Station/AP Get IPv6 secondary DNS address;             data = &ip6,      len = 16, uint8_t[16]
    case ARM_WIFI_IP6_DHCP_MODE:        // Station/AP Get IPv6 DHCPv6 client mode;                data = &mode,     len =  4, uint32_t: ARM_WIFI_IP6_DHCP_xxx (default Off)
    default:
      ret = ARM_DRIVER_ERROR_UNSUPPORTED;
      break;
  }

  return ret;
}

/**
  \fn            int32_t WiFi_Scan (ARM_WIFI_SCAN_INFO_t scan_info[], uint32_t max_num)
  \brief         Scan for available networks in range.
  \param[out]    scan_info Pointer to array of ARM_WIFI_SCAN_INFO_t structures where available Scan Information will be returned
  \param[in]     max_num   Maximum number of Network Information structures to return
  \return        number of ARM_WIFI_SCAN_INFO_t structures returned or error code
                   - value >= 0                   : Number of ARM_WIFI_SCAN_INFO_t structures returned
                   - ARM_DRIVER_ERROR             : Operation failed
                   - ARM_DRIVER_ERROR_PARAMETER   : Parameter error (NULL scan_info pointer or max_num equal to 0)
*/
static int32_t WiFi_Scan (ARM_WIFI_SCAN_INFO_t scan_info[], uint32_t max_num) {
  int32_t ret;
  int8_t  i, ap_num;
  uint32_t ssid_len;
  const mwifi_ap_info_t *ptr_ap_info;

  if ((scan_info == NULL) || (max_num == 0U)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  ret = ARM_DRIVER_OK;
  ap_num = 0;

  if (MX_WIFI_Scan(ptrMX_WIFIObject, MC_SCAN_PASSIVE, NULL, 0) != MX_WIFI_STATUS_OK) {
    ret = ARM_DRIVER_ERROR;
  }

  if (ret == ARM_DRIVER_OK) {
    ap_num = MX_WIFI_Get_scan_result(ptrMX_WIFIObject, scan_buf, (uint8_t)max_num);
    if (ap_num < 0) {
      ret = ARM_DRIVER_ERROR;
    }
  }

  if (ret == ARM_DRIVER_OK) {

    if (ap_num > (int8_t)max_num) {
      ap_num = (int8_t)max_num;
    }

    // Repack scan results from scan_buf into scan_info
    ptr_ap_info = (mwifi_ap_info_t *)scan_buf;

    ssid_len = sizeof(scan_info[0].ssid);
    if (ssid_len > sizeof(ptr_ap_info[0].ssid)) {
      ssid_len = sizeof(ptr_ap_info[0].ssid);
    }

    for (i = 0; i < ap_num; i++) {
      // Repack SSID
      memcpy((void *)scan_info[i].ssid, (const void *)ptr_ap_info[i].ssid, ssid_len); 

      // Repack BSSID
      memcpy((void *)scan_info[i].bssid, (const void *)ptr_ap_info[i].bssid, 6); 

      // Repack Security type
      scan_info[i].security = ConvertSecurityTypeMxToCmsis(ptr_ap_info[i].security);

      // Repack Channel
      scan_info[i].ch = (uint8_t)ptr_ap_info[i].channel;

      // Repack RSSI
      scan_info[i].rssi = (uint8_t)ptr_ap_info[i].rssi;
    }

    ret = ap_num;
  }

  return ret;
}

/**
  \fn            int32_t WiFi_Activate (uint32_t interface, const ARM_WIFI_CONFIG_t *config)
  \brief         Activate interface (Connect to a wireless network or activate an access point).
  \param[in]     interface Interface (0 = Station, 1 = Access Point)
  \param[in]     config    Pointer to ARM_WIFI_CONFIG_t structure where Configuration parameters are located
  \return        execution status
                   - ARM_DRIVER_OK                : Operation successful
                   - ARM_DRIVER_ERROR             : Operation failed
                   - ARM_DRIVER_ERROR_TIMEOUT     : Timeout occurred
                   - ARM_DRIVER_ERROR_UNSUPPORTED : Operation not supported (security type, channel autodetect or WPS not supported)
                   - ARM_DRIVER_ERROR_PARAMETER   : Parameter error (invalid interface, NULL config pointer or invalid configuration)
*/
static int32_t WiFi_Activate (uint32_t interface, const ARM_WIFI_CONFIG_t *config) {
  int32_t  ret, ret_mx;
  uint8_t  tout;
  uint32_t flags, local_ip;

  if (interface != 0U) {
    // Access Point not supported
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if (config == NULL) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if (config->ch != 0U) {
    // Only auto channel selection is supported
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if (config->security == ARM_WIFI_SECURITY_UNKNOWN) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  ret = ARM_DRIVER_OK;

  // Register status change callback
  ret_mx = MX_WIFI_RegisterStatusCallback(ptrMX_WIFIObject, mx_wifi_status_changed, NULL);
  if (ret_mx != MX_WIFI_STATUS_OK) {
    ret = ConvertErrorCodeMxToCmsis(ret_mx);
  }

  /* Connect to AP */
  if (ret == ARM_DRIVER_OK) {
    ret_mx = MX_WIFI_Connect(ptrMX_WIFIObject, config->ssid, config->pass, ConvertSecurityTypeCmsisToMx(config->security));
  }
  if (ret_mx != MX_WIFI_STATUS_OK) {
    ret = ConvertErrorCodeMxToCmsis(ret_mx);
  }

  // Wait for connect event
  if (ret == ARM_DRIVER_OK) {
    flags = osEventFlagsWait(ef_id_sta_status, MWIFI_EVENT_STA_UP, osFlagsWaitAll, 60000U);
    if ((flags & 0x80000000UL) != 0U) {
      // Timeout or error
      ret = ARM_DRIVER_ERROR;
    }
  }   

  /* Get IP */
  if (ret == ARM_DRIVER_OK) {
    for (tout = 0U; tout < 60U; tout++) {
      ret_mx = MX_WIFI_GetIPAddress(ptrMX_WIFIObject, (uint8_t *)&local_ip, (mwifi_if_t)MC_STATION);
      if (ret_mx == MX_WIFI_STATUS_OK) {
        break;
      }
      (void)osDelay(1000U);
    }
  }
  if (ret_mx != MX_WIFI_STATUS_OK) {
    ret = ConvertErrorCodeMxToCmsis(ret_mx);
  }

  return ret;
}

/**
  \fn            int32_t WiFi_Deactivate (uint32_t interface)
  \brief         Deactivate interface (Disconnect from a wireless network or deactivate an access point).
  \param[in]     interface Interface (0 = Station, 1 = Access Point)
  \return        execution status
                   - ARM_DRIVER_OK                : Operation successful
                   - ARM_DRIVER_ERROR             : Operation failed
                   - ARM_DRIVER_ERROR_PARAMETER   : Parameter error (invalid interface)
*/
static int32_t WiFi_Deactivate (uint32_t interface) {
  int32_t ret, ret_mx;

  if (interface != 0U) {
    // Access Point not supported
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  ret = ARM_DRIVER_OK;

  ret_mx = MX_WIFI_Disconnect(ptrMX_WIFIObject);
  if (ret_mx != MX_WIFI_STATUS_OK) {
    ret = ConvertErrorCodeMxToCmsis(ret_mx);
  }

  // Un-register status change callback
  if (ret == ARM_DRIVER_OK) {
    ret_mx = MX_WIFI_UnRegisterStatusCallback_if(ptrMX_WIFIObject, (mwifi_if_t)MC_STATION);
    if (ret_mx != MX_WIFI_STATUS_OK) {
      ret = ConvertErrorCodeMxToCmsis(ret_mx);
    }
  }

  return ret;
}

/**
  \fn            uint32_t WiFi_IsConnected (void)
  \brief         Get station connection status.
  \return        station connection status
                   - value != 0: Station connected
                   - value = 0: Station not connected
*/
static uint32_t WiFi_IsConnected (void) {

  if (driver_initialized == 0U) {
    return 0U;
  }

  return ((uint32_t)MX_WIFI_IsConnected(ptrMX_WIFIObject));
}

/**
  \fn            int32_t WiFi_GetNetInfo (ARM_WIFI_NET_INFO_t *net_info)
  \brief         Get station Network Information.
  \param[out]    net_info  Pointer to ARM_WIFI_NET_INFO_t structure where station Network Information will be returned
  \return        execution status
                   - ARM_DRIVER_OK                : Operation successful
                   - ARM_DRIVER_ERROR             : Operation failed (station not connected)
                   - ARM_DRIVER_ERROR_UNSUPPORTED : Operation not supported
                   - ARM_DRIVER_ERROR_PARAMETER   : Parameter error (invalid interface or NULL net_info pointer)
*/
static int32_t WiFi_GetNetInfo (ARM_WIFI_NET_INFO_t *net_info) {
  (void)net_info;

  return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
  \fn            int32_t WiFi_SocketCreate (int32_t af, int32_t type, int32_t protocol)
  \brief         Create a communication socket.
  \param[in]     af       Address family
  \param[in]     type     Socket type
  \param[in]     protocol Socket protocol
  \return        status information
                   - Socket identification number (>=0)
                   - ARM_SOCKET_EINVAL            : Invalid argument
                   - ARM_SOCKET_ENOTSUP           : Operation not supported
                   - ARM_SOCKET_ENOMEM            : Not enough memory
                   - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketCreate (int32_t af, int32_t type, int32_t protocol) {
  int32_t rc, mx_domain, mx_type, mx_protocol;
  uint32_t val;

  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
  }

  // Convert and check parameters
  switch (af) {
    case ARM_SOCKET_AF_INET:
      mx_domain = MX_AF_INET;
      break;
    case ARM_SOCKET_AF_INET6:
    default:
      return ARM_SOCKET_EINVAL;
  }
  switch (type) {
    case ARM_SOCKET_SOCK_STREAM:
      mx_type = MX_SOCK_STREAM;
      if (protocol == 0) {              // Autodetect protocol
        protocol = ARM_SOCKET_IPPROTO_TCP;
      } else if (protocol != ARM_SOCKET_IPPROTO_TCP) {
        return ARM_SOCKET_EINVAL;
      }
      break;
    case ARM_SOCKET_SOCK_DGRAM:
      mx_type = MX_SOCK_DGRAM;
      if (protocol == 0) {              // Autodetect protocol
        protocol = ARM_SOCKET_IPPROTO_UDP;
      } else if (protocol != ARM_SOCKET_IPPROTO_UDP) {
        return ARM_SOCKET_EINVAL;
      }
      break;
    default:
      return ARM_SOCKET_EINVAL;
  }
  switch (protocol) {
    case ARM_SOCKET_IPPROTO_TCP:
      mx_protocol = MX_IPPROTO_TCP;
      break;
    case ARM_SOCKET_IPPROTO_UDP:
      mx_protocol = MX_IPPROTO_UDP;
      break;
    default:
      return ARM_SOCKET_EINVAL;
  }

  if (osMutexAcquire(mutex_id_sock_attr, WIFI_EMW3080_SOCKETS_TIMEOUT) == osOK) {

    rc = MX_WIFI_Socket_create(ptrMX_WIFIObject, mx_domain, mx_type, mx_protocol);
    if ((rc >= 0) && (rc < WIFI_EMW3080_SOCKETS_NUM)) {         // If create has succeeded and socket number is valid
      memset (&sock_attr[rc], 0, sizeof(sock_attr[0]));
      sock_attr[rc].type = (int8_t)type;
      sock_attr[rc].flags.created = 1U;
      sock_attr[rc].rcvtimeo = (uint32_t)WIFI_EMW3080_SOCKETS_RCVTIMEO;

      // Set default receive timeout for socket to 1 ms, since blocking mode will be emulated by 
      // periodically polling until timeout, so SPI is not blocked for long time
      val = 1;
      (void)MX_WIFI_Socket_setsockopt(ptrMX_WIFIObject, rc, MX_SOL_SOCKET, (int32_t)MX_SO_RCVTIMEO, &val, 4);
    } else if (rc >= WIFI_EMW3080_SOCKETS_NUM) {                // If create has succeeded but socket number is too high
      (void)MX_WIFI_Socket_close(ptrMX_WIFIObject, rc);
      rc = ARM_SOCKET_ENOMEM;
    } else {                                                    // If create has failed
      rc = ConvertSocketErrorCodeMxToCmsis(rc);
    }

    if (osMutexRelease(mutex_id_sock_attr) != osOK) {
      rc = ARM_SOCKET_ERROR;
    }
  } else {
    rc = ARM_SOCKET_ERROR;
  }

  return rc;
}

/**
  \fn            int32_t WiFi_SocketBind (int32_t socket, const uint8_t *ip, uint32_t ip_len, uint16_t port)
  \brief         Assign a local address to a socket.
  \param[in]     socket   Socket identification number
  \param[in]     ip       Pointer to local IP address
  \param[in]     ip_len   Length of 'ip' address in bytes
  \param[in]     port     Local port number
  \return        status information
                   - 0                            : Operation successful
                   - ARM_SOCKET_ESOCK             : Invalid socket
                   - ARM_SOCKET_EINVAL            : Invalid argument (address or socket already bound)
                   - ARM_SOCKET_EADDRINUSE        : Address already in use
                   - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketBind (int32_t socket, const uint8_t *ip, uint32_t ip_len, uint16_t port) {
  SOCKADDR_STORAGE addr;
  int32_t addr_len;
  int32_t rc;

  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
  }

  // Check parameters
  if ((socket < 0) || (socket >= WIFI_EMW3080_SOCKETS_NUM)) {
    return ARM_SOCKET_ESOCK;
  }
  if ((ip == NULL) || (port == 0U)) {
    return ARM_SOCKET_EINVAL;
  }

  // Construct local address
  switch (ip_len) {
    case 4U: {
      SOCKADDR_IN *sa = (SOCKADDR_IN *)&addr;
      sa->sin_family = MX_AF_INET;
      memcpy(&sa->sin_addr, ip, 4U);
      sa->sin_port = (uint16_t)htons(port);
      addr_len     = (int32_t)sizeof(SOCKADDR_IN);
    } break;
    default:
      return ARM_SOCKET_EINVAL;
  }

  if (osMutexAcquire(mutex_id_sock_attr, WIFI_EMW3080_SOCKETS_TIMEOUT) == osOK) {

    // Check socket status and IP
    if (sock_attr[socket].flags.created == 0U) {
      rc = ARM_SOCKET_ESOCK;
    } else if (sock_attr[socket].flags.connected == 1U) {
      rc = ARM_SOCKET_EISCONN;
    } else if ((sock_attr[socket].flags.bound == 1U) && 
               (memcmp(sock_attr[socket].local_ip, ip, 4) == 0)) {      // If attempt to bind to already bound address
      rc = ARM_SOCKET_EINVAL;
    } else {

      rc = 0;
      for (int32_t i = 0; i < WIFI_EMW3080_SOCKETS_NUM; i++) {
        if ((sock_attr[i].flags.bound == 1U) &&
            (sock_attr[i].local_port == port) &&                // If another socket is already bound to same port and
           ((memcmp(sock_attr[i].local_ip, ip, 4) == 0) ||      // is already bound to requested IP
           ((sock_attr[i].local_ip[0] == 0U) &&
            (sock_attr[i].local_ip[1] == 0U) &&
            (sock_attr[i].local_ip[2] == 0U) &&
            (sock_attr[i].local_ip[3] == 0U)))) {               // is already bound to any IP (0.0.0.0)
          rc = ARM_SOCKET_EADDRINUSE;
        }
      }

      if (rc == 0) {
        rc = MX_WIFI_Socket_bind(ptrMX_WIFIObject, socket, (const struct mx_sockaddr *)&addr, addr_len);
        if (rc == 0) {                                          // If bind has succeeded
          sock_attr[socket].flags.bound = 1U;
          memcpy(sock_attr[socket].local_ip, ip, 4);            // Store local IP
          sock_attr[socket].local_port = port;                  // Store local port
        } else if (rc < 0) {                                    // If bind has failed
          rc = ConvertSocketErrorCodeMxToCmsis(rc);
        }
      }
    }

    if (osMutexRelease(mutex_id_sock_attr) != osOK) {
      rc = ARM_SOCKET_ERROR;
    }
  } else {
    rc = ARM_SOCKET_ERROR;
  }

  return rc;
}

/**
  \fn            int32_t WiFi_SocketListen (int32_t socket, int32_t backlog)
  \brief         Listen for socket connections.
  \param[in]     socket   Socket identification number
  \param[in]     backlog  Number of connection requests that can be queued
  \return        status information
                   - 0                            : Operation successful
                   - ARM_SOCKET_ESOCK             : Invalid socket
                   - ARM_SOCKET_EINVAL            : Invalid argument (socket not bound)
                   - ARM_SOCKET_ENOTSUP           : Operation not supported
                   - ARM_SOCKET_EISCONN           : Socket is already connected
                   - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketListen (int32_t socket, int32_t backlog) {
  int32_t rc;

  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
  }

  // Check parameters
  if ((socket < 0) || (socket >= WIFI_EMW3080_SOCKETS_NUM)) {
    return ARM_SOCKET_ESOCK;
  }

  if (osMutexAcquire(mutex_id_sock_attr, WIFI_EMW3080_SOCKETS_TIMEOUT) == osOK) {

    // Check socket type and status
    if (sock_attr[socket].type == ARM_SOCKET_SOCK_DGRAM) {
      rc = ARM_SOCKET_ENOTSUP;
    } else if (sock_attr[socket].flags.created == 0U) {
      rc = ARM_SOCKET_ESOCK;
    } else if (sock_attr[socket].flags.bound == 0U) {
      rc = ARM_SOCKET_EINVAL;
    } else if (sock_attr[socket].flags.listening == 1U) {
      rc = ARM_SOCKET_EINVAL;
    } else {

      rc = MX_WIFI_Socket_listen(ptrMX_WIFIObject, socket, backlog);
      if (rc == 0) {                                            // If listen has succeeded
        sock_attr[socket].flags.listening = 1U;
      } else if (rc < 0) {                                      // If listen has failed
        rc = ConvertSocketErrorCodeMxToCmsis(rc);
      }
    }

    if (osMutexRelease(mutex_id_sock_attr) != osOK) {
      rc = ARM_SOCKET_ERROR;
    }
  } else {
    rc = ARM_SOCKET_ERROR;
  }

  return rc;
}

/**
  \fn            int32_t WiFi_SocketAccept (int32_t socket, uint8_t *ip, uint32_t *ip_len, uint16_t *port)
  \brief         Accept a new connection on a socket.
  \param[in]     socket   Socket identification number
  \param[out]    ip       Pointer to buffer where address of connecting socket shall be returned (NULL for none)
  \param[in,out] ip_len   Pointer to length of 'ip' (or NULL if 'ip' is NULL)
                   - length of supplied 'ip' on input
                   - length of stored 'ip' on output
  \param[out]    port     Pointer to buffer where port of connecting socket shall be returned (NULL for none)
  \return        status information
                   - socket identification number of accepted socket (>=0)
                   - ARM_SOCKET_ESOCK             : Invalid socket
                   - ARM_SOCKET_EINVAL            : Invalid argument (socket not in listen mode)
                   - ARM_SOCKET_ENOTSUP           : Operation not supported (socket type does not support accepting connections)
                   - ARM_SOCKET_ECONNRESET        : Connection reset by the peer
                   - ARM_SOCKET_ECONNABORTED      : Connection aborted locally
                   - ARM_SOCKET_EAGAIN            : Operation would block or timed out (may be called again)
                   - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketAccept (int32_t socket, uint8_t *ip, uint32_t *ip_len, uint16_t *port) {
  SOCKADDR_STORAGE addr;
  int32_t addr_len = (int32_t)sizeof(addr);
  int32_t rc;
  uint8_t nb;

  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
  }

  // Check parameters
  if ((socket < 0) || (socket >= WIFI_EMW3080_SOCKETS_NUM)) {
    return ARM_SOCKET_ESOCK;
  }

  if (osMutexAcquire(mutex_id_sock_attr, WIFI_EMW3080_SOCKETS_TIMEOUT) == osOK) {

    // Check socket type and status
    if (sock_attr[socket].type == ARM_SOCKET_SOCK_DGRAM) {
      rc = ARM_SOCKET_ENOTSUP;
    } else if (sock_attr[socket].flags.created == 0U) {
      rc = ARM_SOCKET_ESOCK;
    } else {
      rc = 0;
    }

    if (osMutexRelease(mutex_id_sock_attr) != osOK) {
      rc = ARM_SOCKET_ERROR;
    }
  } else {
    rc = ARM_SOCKET_ERROR;
  }

  if (rc == 0) {
    if (sock_attr[socket].ionbio == 0U) {       // If socket is in blocking mode (emulate by polling until timeout)
      nb = 0U;
    } else {                                    // If socket is in non-blocking mode
      nb = 1U;
    }

    do {
      if (osMutexAcquire(mutex_id_sock_attr, WIFI_EMW3080_SOCKETS_TIMEOUT) == osOK) {
        rc = MX_WIFI_Socket_accept(ptrMX_WIFIObject, socket, (struct mx_sockaddr *)&addr, (uint32_t *)&addr_len);
        if ((rc >= 0) && (rc < WIFI_EMW3080_SOCKETS_NUM)) {       // If accept has succeeded and socket number is valid
          // Inherit listening socket's settings
          memset (&sock_attr[rc], 0, sizeof(sock_attr[0]));
          sock_attr[rc].ionbio   = sock_attr[socket].ionbio;
          sock_attr[rc].type     = sock_attr[socket].type;
          sock_attr[rc].rcvtimeo = sock_attr[socket].rcvtimeo;
          sock_attr[rc].sndtimeo = sock_attr[socket].sndtimeo;

          // Implicitly handle accepted socket: created, bound, connected
          sock_attr[rc].flags.created    = 1U;
          sock_attr[rc].flags.bound      = 1U;
          sock_attr[rc].flags.connecting = 0U;
          sock_attr[rc].flags.connected  = 1U;

          // Process remote IP address and port
          if (addr.ss_family == (uint8_t)MX_AF_INET) {
            const SOCKADDR_IN *sa = (SOCKADDR_IN *)&addr;
            memcpy(sock_attr[rc].remote_ip, &sa->sin_addr, 4);    // Store remote IP
            if ((ip != NULL) && (ip_len != NULL) && (*ip_len >= sizeof(sa->sin_addr))) {
              memcpy(ip, &sa->sin_addr, sizeof(sa->sin_addr));
              *ip_len = sizeof(sa->sin_addr);
            }
            sock_attr[rc].remote_port = ntohs (sa->sin_port);     // Store remote port
            if (port != NULL) {
              *port   = ntohs (sa->sin_port);
            }
          }
        } else if (rc >= WIFI_EMW3080_SOCKETS_NUM) {              // If accept has succeeded but socket number is too high
          (void)MX_WIFI_Socket_close(ptrMX_WIFIObject, rc);
          rc = ARM_SOCKET_ERROR;
        } else {                                                  // If accept has failed
          rc = 0;
        }

        if (osMutexRelease(mutex_id_sock_attr) != osOK) {
          rc = ARM_SOCKET_ERROR;
        }
      } else {
        rc = ARM_SOCKET_ERROR;
      }

      if ((rc == 0) && (nb == 0U)) {
        (void)osDelay(WIFI_EMW3080_SOCKETS_INTERVAL);
      }
    } while ((rc == 0) && (nb == 0U));
  }

  if (rc == 0) {                        // If operation would block or timed out
    rc = ARM_SOCKET_EAGAIN;
  }

  return rc;
}

/**
  \fn            int32_t WiFi_SocketConnect (int32_t socket, const uint8_t *ip, uint32_t ip_len, uint16_t port)
  \brief         Connect a socket to a remote host.
  \param[in]     socket   Socket identification number
  \param[in]     ip       Pointer to remote IP address
  \param[in]     ip_len   Length of 'ip' address in bytes
  \param[in]     port     Remote port number
  \return        status information
                   - 0                            : Operation successful
                   - ARM_SOCKET_ESOCK             : Invalid socket
                   - ARM_SOCKET_EINVAL            : Invalid argument
                   - ARM_SOCKET_EALREADY          : Connection already in progress
                   - ARM_SOCKET_EINPROGRESS       : Operation in progress
                   - ARM_SOCKET_EISCONN           : Socket is connected
                   - ARM_SOCKET_ECONNREFUSED      : Connection rejected by the peer
                   - ARM_SOCKET_ECONNABORTED      : Connection aborted locally
                   - ARM_SOCKET_EADDRINUSE        : Address already in use
                   - ARM_SOCKET_ETIMEDOUT         : Operation timed out
                   - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketConnect (int32_t socket, const uint8_t *ip, uint32_t ip_len, uint16_t port) {
  SOCKADDR_STORAGE addr;
  int32_t addr_len;
  int32_t rc;

  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
  }

  // Check parameters
  if ((socket < 0) || (socket >= WIFI_EMW3080_SOCKETS_NUM)) {
    return ARM_SOCKET_ESOCK;
  }
  if ((ip == NULL) || (port == 0U) || (ip_len != 4U)) {
    return ARM_SOCKET_EINVAL;
  }
  if ((ip[0] == 0U) && (ip[1] == 0U) && (ip[2] == 0U) && (ip[3] == 0U)) {
    return ARM_SOCKET_EINVAL;
  }

  // Construct remote host address
  switch (ip_len) {
    case 4U: {
      SOCKADDR_IN *sa = (SOCKADDR_IN *)&addr;
      sa->sin_family = MX_AF_INET;
      memcpy(&sa->sin_addr, ip, 4U);
      sa->sin_port = (uint16_t)htons(port);
      addr_len     = (int32_t)sizeof(SOCKADDR_IN);
    } break;
    default:
      return ARM_SOCKET_EINVAL;
  }

  rc = 0;

  if (osMutexAcquire(mutex_id_sock_attr, WIFI_EMW3080_SOCKETS_TIMEOUT) == osOK) {

    // Check socket status
    if (sock_attr[socket].flags.created == 0U) {
      rc = ARM_SOCKET_ESOCK;
    } else if (sock_attr[socket].flags.listening == 1U) {
      rc = ARM_SOCKET_EINVAL;
    } else if (sock_attr[socket].flags.connected == 1U) {
      rc = ARM_SOCKET_EISCONN;
    } else {

      rc = MX_WIFI_Socket_connect(ptrMX_WIFIObject, socket, (struct mx_sockaddr *)&addr, addr_len);
      if (rc == 0) {                                          // If connect has succeeded
        sock_attr[socket].flags.connecting = 0U;
        sock_attr[socket].flags.connected  = 1U;
        sock_attr[socket].flags.bound      = 1U;              // Socket is also implicitly bound when connect succeeds
        memcpy(sock_attr[socket].remote_ip, ip, 4);           // Store remote IP
        sock_attr[socket].remote_port = port;                 // Store remote port
      } else if (rc < 0) {                                    // If connect has failed
        rc = ConvertSocketErrorCodeMxToCmsis(rc);
      }
      if ((sock_attr[socket].ionbio != 0U) &&                 // If non-blocking connect and
          (sock_attr[socket].flags.connecting == 0U)) {       // socket connect first call
        sock_attr[socket].flags.connecting = 1U;
        rc = ARM_SOCKET_EINPROGRESS;
      }
    }

    if (osMutexRelease(mutex_id_sock_attr) != osOK) {
      rc = ARM_SOCKET_ERROR;
    }
  } else {
    rc = ARM_SOCKET_ERROR;
  }

  return rc;
}

/**
  \fn            int32_t WiFi_SocketRecv (int32_t socket, void *buf, uint32_t len)
  \brief         Receive data or check if data is available on a connected socket.
  \param[in]     socket   Socket identification number
  \param[out]    buf      Pointer to buffer where data should be stored
  \param[in]     len      Length of buffer (in bytes), set len = 0 to check if data is available
  \return        status information
                   - number of bytes received (>=0), if len != 0
                   - 0                            : Data is available (len = 0)
                   - ARM_SOCKET_ESOCK             : Invalid socket
                   - ARM_SOCKET_EINVAL            : Invalid argument (pointer to buffer or length)
                   - ARM_SOCKET_ENOTCONN          : Socket is not connected
                   - ARM_SOCKET_ECONNRESET        : Connection reset by the peer
                   - ARM_SOCKET_ECONNABORTED      : Connection aborted locally
                   - ARM_SOCKET_EAGAIN            : Operation would block or timed out (may be called again)
                   - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketRecv (int32_t socket, void *buf, uint32_t len) {
  int32_t  rc, rc_;
  uint32_t to;
  uint32_t ofs = 0U;
  uint32_t retry;
  uint8_t  forever = 0U;
  uint8_t  nb;

  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
  }

  // Check parameters
  if ((socket < 0) || (socket >= WIFI_EMW3080_SOCKETS_NUM)) {
    return ARM_SOCKET_ESOCK;
  }
  if ((buf == NULL) && (len != 0U)) {
    return ARM_SOCKET_EINVAL;
  }

  if (len == 0U) {
    // This is a special functionality in which len = 0 is used to 
    // check if receive function has data available to be read.
    // Function behaves the same way regardless if socket is in blocking 
    // or non-blocking mode.
    // If there is data available it returns 0, otherwise it returns ARM_SOCKET_EAGAIN
    if (sock_attr[socket].rx_buf_available_len != 0U) { // If data is available (data was already read)
      return 0;
    }
  }

  if (osMutexAcquire(mutex_id_sock_attr, WIFI_EMW3080_SOCKETS_TIMEOUT) == osOK) {

    // Check socket status
    if (sock_attr[socket].flags.created == 0U) {
      rc = ARM_SOCKET_ESOCK;
    } else if ((sock_attr[socket].type == ARM_SOCKET_SOCK_STREAM) && (sock_attr[socket].flags.connected == 0U)) {
      rc = ARM_SOCKET_ENOTCONN;
    } else {
      // Check and handle if there is data already received in local buffer (on previous call with len = 0)
      if ((len != 0U) && (sock_attr[socket].rx_buf_available_len == 1U)) {
        *((uint8_t *)buf) = sock_attr[socket].rx_buf[0];
        sock_attr[socket].rx_buf_available_len = 0U;
        ofs = 1U;
        rc = 1;
      } else {
        ofs = 0U;
        rc = 0;
      }
    }

    if (osMutexRelease(mutex_id_sock_attr) != osOK) {
      rc = ARM_SOCKET_ERROR;
    }
  } else {
    rc = ARM_SOCKET_ERROR;
  }

  if ((rc == 0) || ((rc == 1) && (len > 1))) {
    if (sock_attr[socket].ionbio == 0U) {       // If socket is in blocking mode (emulate by polling until timeout)
      nb = 0U;
      to = sock_attr[socket].rcvtimeo;
      if (to == 0U) {
        forever = 1U;
      } else {
        forever = 0U;
      }
    } else {                                    // If socket is in non-blocking mode
      nb = 1U;
      to = 0U;
    }

    retry = (uint32_t)WIFI_EMW3080_SOCKETS_RCV_RETRIES;
    do {
      if (osMutexAcquire(mutex_id_sock_attr, WIFI_EMW3080_SOCKETS_TIMEOUT) == osOK) {
        if (len == 0U) {                        // if len = 0, try to receive 1 byte to local buffer
          rc = MX_WIFI_Socket_recv(ptrMX_WIFIObject, socket, (uint8_t *)sock_attr[socket].rx_buf, 1, 0);
          if (rc > 0) {                         // If 1 byte was received
            sock_attr[socket].rx_buf_available_len = (uint16_t)rc;
          } else {
            if (retry > 0U) {
              retry = retry - 1U;
              rc = 0;
            } else {
              rc = ConvertSocketErrorCodeMxToCmsis(rc);
            }
          }
        } else {                                // if len != 0, try to receive into buffer provided as function parameter
          rc_ = MX_WIFI_Socket_recv(ptrMX_WIFIObject, socket, ((uint8_t *)buf)+ofs, (int32_t)(len-ofs), 0);
          if (rc_ > 0) {
            rc += rc_;
          } else {
            rc = rc_;
          }
          if (rc < 0) {
            if (retry > 0U) {
              retry = retry - 1U;
              rc = 0;
            } else {
              rc = ConvertSocketErrorCodeMxToCmsis(rc);
            }
          }
        }

        if (osMutexRelease(mutex_id_sock_attr) != osOK) {
          rc = ARM_SOCKET_ERROR;
        }
      } else {
        rc = ARM_SOCKET_ERROR;
      }

      if (rc == 0) {
        if ((to >=  (uint32_t)WIFI_EMW3080_SOCKETS_INTERVAL) || (forever != 0U)) {
          (void)osDelay((uint32_t)WIFI_EMW3080_SOCKETS_INTERVAL);
          if (to >= (uint32_t)WIFI_EMW3080_SOCKETS_INTERVAL) {
            to -=   (uint32_t)WIFI_EMW3080_SOCKETS_INTERVAL;
          }
        } else {
          (void)osDelay(to);
          to = 0U;
        }
      }
    } while (((to != 0U) || (forever != 0U)) && (rc == 0) && (nb == 0U));
  }

  if (rc == 0) {                        // If operation would block or timed out
    rc = ARM_SOCKET_EAGAIN;
  } else if ((rc > 0) && (len == 0U)) { // If data is available to be read
    rc = 0;
  }

  return rc;
}

/**
  \fn            int32_t WiFi_SocketRecvFrom (int32_t socket, void *buf, uint32_t len, uint8_t *ip, uint32_t *ip_len, uint16_t *port)
  \brief         Receive data or check if data is available on a socket.
  \param[in]     socket   Socket identification number
  \param[out]    buf      Pointer to buffer where data should be stored
  \param[in]     len      Length of buffer (in bytes), set len = 0 to check if data is available
  \param[out]    ip       Pointer to buffer where remote source address shall be returned (NULL for none)
  \param[in,out] ip_len   Pointer to length of 'ip' (or NULL if 'ip' is NULL)
                   - length of supplied 'ip' on input
                   - length of stored 'ip' on output
  \param[out]    port     Pointer to buffer where remote source port shall be returned (NULL for none)
  \return        status information
                   - number of bytes received (>=0), if len != 0
                   - 0                            : Data is available (len = 0)
                   - ARM_SOCKET_ESOCK             : Invalid socket
                   - ARM_SOCKET_EINVAL            : Invalid argument (pointer to buffer or length)
                   - ARM_SOCKET_ENOTCONN          : Socket is not connected
                   - ARM_SOCKET_ECONNRESET        : Connection reset by the peer
                   - ARM_SOCKET_ECONNABORTED      : Connection aborted locally
                   - ARM_SOCKET_EAGAIN            : Operation would block or timed out (may be called again)
                   - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketRecvFrom (int32_t socket, void *buf, uint32_t len, uint8_t *ip, uint32_t *ip_len, uint16_t *port) {
  SOCKADDR_STORAGE addr;
  int32_t  addr_len = (int32_t)sizeof(addr);
  int32_t  rc;
  uint32_t to;
  uint32_t len_to_copy;
  uint8_t  forever = 0U;
  uint8_t  nb;

  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
  }

  // Check parameters
  if ((socket < 0) || (socket >= WIFI_EMW3080_SOCKETS_NUM)) {
    return ARM_SOCKET_ESOCK;
  }
  if ((buf == NULL) && (len != 0U)) {
    return ARM_SOCKET_EINVAL;
  }

  if (len == 0U) {
    // This is a special functionality in which len = 0 is used to 
    // check if receive function has data available to be read.
    // Function behaves the same way regardless if socket is in blocking 
    // or non-blocking mode.
    // If there is data available it returns 0, otherwise it returns ARM_SOCKET_EAGAIN
    if (sock_attr[socket].rx_buf_available_len != 0U) { // If data is available (data was already read)
      return 0;
    }
  }

  if (osMutexAcquire(mutex_id_sock_attr, WIFI_EMW3080_SOCKETS_TIMEOUT) == osOK) {

    // Check socket status
    if (sock_attr[socket].flags.created == 0U) {
      rc = ARM_SOCKET_ESOCK;
    } else if ((sock_attr[socket].type == ARM_SOCKET_SOCK_STREAM) && (sock_attr[socket].flags.connected == 0U)) {
      rc = ARM_SOCKET_ENOTCONN;
    } else {
      // Check and handle if data was already received in local buffer (on previous call with len = 0)
      if ((len != 0U) && (sock_attr[socket].rx_buf_available_len != 0U)) {
        len_to_copy = len;
        if (len_to_copy > sock_attr[socket].rx_buf_available_len) {
          len_to_copy = sock_attr[socket].rx_buf_available_len;
        }
        memcpy(buf, sock_attr[socket].rx_buf, len_to_copy);
        sock_attr[socket].rx_buf_available_len = 0U;
        if ((ip != NULL) && (ip_len != NULL)) {
          memcpy(ip, sock_attr[socket].rx_ip, 4);
          *ip_len = 4;
        }
        if (port != NULL) {
          *port   = sock_attr[socket].rx_port;
        }
        rc = (int32_t)len_to_copy;
      } else {
        rc = 0;
      }
    }

    if (osMutexRelease(mutex_id_sock_attr) != osOK) {
      rc = ARM_SOCKET_ERROR;
    }
  } else {
    rc = ARM_SOCKET_ERROR;
  }

  if (rc == 0) {
    if (sock_attr[socket].ionbio == 0U) {       // If socket is in blocking mode (emulate by polling until timeout)
      nb = 0U;
      to = sock_attr[socket].rcvtimeo;
      if (to == 0U) {
        forever = 1U;
      } else {
        forever = 0U;
      }
    } else {                                    // If socket is in non-blocking mode
      nb = 1U;
      to = 0U;
    }

    do {
      if (osMutexAcquire(mutex_id_sock_attr, WIFI_EMW3080_SOCKETS_TIMEOUT) == osOK) {
        if (len == 0U) {                        // if len = 0, try to receive to local buffer
          rc = MX_WIFI_Socket_recvfrom(ptrMX_WIFIObject, socket, (uint8_t *)sock_attr[socket].rx_buf, WIFI_EMW3080_SOCKETS_RX_BUF_SIZE, 0, (struct mx_sockaddr *)&addr, (uint32_t *)&addr_len);
          if (rc > 0) {                         // If something was received
            // Store remote IP address and port, data is received int local buffer
            sock_attr[socket].rx_buf_available_len = (uint16_t)rc;
            const SOCKADDR_IN *sa = (SOCKADDR_IN *)&addr;
            if ((sa->sin_family == (uint8_t)MX_AF_INET) && (sizeof(sa->sin_addr) >= 4U)) {
              memcpy(sock_attr[socket].rx_ip, &sa->sin_addr, 4);
              sock_attr[socket].rx_port = ntohs (sa->sin_port);
            }
          } else if (rc < 0) {
            rc = ConvertSocketErrorCodeMxToCmsis(rc);
          }
        } else {                                // if len != 0, try to receive into buffer provided as function parameter
          rc = MX_WIFI_Socket_recvfrom(ptrMX_WIFIObject, socket, (uint8_t *)buf, (int32_t)len, 0, (struct mx_sockaddr *)&addr, (uint32_t *)&addr_len);
          if (rc > 0) {                         // If something was received
            // Store remote IP address and port, data is already in buffer
            if ((ip != NULL) && (ip_len != NULL)) {
              const SOCKADDR_IN *sa = (SOCKADDR_IN *)&addr;
              if ((sa->sin_family == (uint8_t)MX_AF_INET) && (*ip_len >= sizeof(sa->sin_addr))) {
                memcpy(ip, &sa->sin_addr, sizeof(sa->sin_addr));
                *ip_len = sizeof(sa->sin_addr);
              }
              if (port != NULL) {
                *port   = ntohs (sa->sin_port);
              }
            }
          } else if (rc < 0) {
            rc = ConvertSocketErrorCodeMxToCmsis(rc);
          }
        }

        if (osMutexRelease(mutex_id_sock_attr) != osOK) {
          rc = ARM_SOCKET_ERROR;
        }
      } else {
        rc = ARM_SOCKET_ERROR;
      }

      if (rc == 0) {
        if ((to >=  (uint32_t)WIFI_EMW3080_SOCKETS_INTERVAL) || (forever != 0U)) {
          (void)osDelay((uint32_t)WIFI_EMW3080_SOCKETS_INTERVAL);
          if (to >= (uint32_t)WIFI_EMW3080_SOCKETS_INTERVAL) {
            to -=   (uint32_t)WIFI_EMW3080_SOCKETS_INTERVAL;
          }
        } else {
          (void)osDelay(to);
          to = 0U;
        }
      }
    } while (((to != 0U) || (forever != 0U)) && (rc == 0) && (nb == 0U));
  }

  if (rc == 0) {                        // If operation would block or timed out
    rc = ARM_SOCKET_EAGAIN;
  } else if ((rc > 0) && (len == 0U)) { // If data is available to be read
    rc = 0;
  }

  return rc;
}

/**
  \fn            int32_t WiFi_SocketSend (int32_t socket, const void *buf, uint32_t len)
  \brief         Send data or check if data can be sent on a connected socket.
  \param[in]     socket   Socket identification number
  \param[in]     buf      Pointer to buffer containing data to send
  \param[in]     len      Length of data (in bytes), set len = 0 to check if data can be sent
  \return        status information
                   - number of bytes sent (>=0), if len != 0
                   - 0                            : Data can be sent (len = 0)
                   - ARM_SOCKET_ESOCK             : Invalid socket
                   - ARM_SOCKET_EINVAL            : Invalid argument (pointer to buffer or length)
                   - ARM_SOCKET_ENOTCONN          : Socket is not connected
                   - ARM_SOCKET_ECONNRESET        : Connection reset by the peer
                   - ARM_SOCKET_ECONNABORTED      : Connection aborted locally
                   - ARM_SOCKET_EAGAIN            : Operation would block or timed out (may be called again)
                   - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketSend (int32_t socket, const void *buf, uint32_t len) {
  int32_t rc;
  uint8_t retry;

  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
  }

  // Check parameters
  if ((socket < 0) || (socket >= WIFI_EMW3080_SOCKETS_NUM)) {
    return ARM_SOCKET_ESOCK;
  }
  if ((buf == NULL) && (len != 0U)) {
    return ARM_SOCKET_EINVAL;
  }
  if (len == 0U) {
    return 0;
  }

  if (osMutexAcquire(mutex_id_sock_attr, WIFI_EMW3080_SOCKETS_TIMEOUT) == osOK) {

    // Check socket status
    if (sock_attr[socket].flags.created == 0U) {
      rc = ARM_SOCKET_ESOCK;
    } else if (sock_attr[socket].flags.connected == 0U) {
      rc = ARM_SOCKET_ENOTCONN;
    } else {

      for (retry = 3U; retry != 0U; retry--) {
        rc = MX_WIFI_Socket_send(ptrMX_WIFIObject, socket, (uint8_t *)buf, (int32_t)len, 0);
        if (rc > 0) {
          break;
        }
        (void)osDelay(10U);
      }
      if (rc < 0) {
        sock_attr[socket].flags.connecting = 0U;
        sock_attr[socket].flags.connected  = 0U;
        rc = ARM_SOCKET_ECONNRESET;
      }
    }

    if (osMutexRelease(mutex_id_sock_attr) != osOK) {
      rc = ARM_SOCKET_ERROR;
    }
  } else {
    rc = ARM_SOCKET_ERROR;
  }

  return rc;
}

/**
  \fn            int32_t WiFi_SocketSendTo (int32_t socket, const void *buf, uint32_t len, const uint8_t *ip, uint32_t ip_len, uint16_t port)
  \brief         Send data or check if data can be sent on a socket.
  \param[in]     socket   Socket identification number
  \param[in]     buf      Pointer to buffer containing data to send
  \param[in]     len      Length of data (in bytes), set len = 0 to check if data can be sent
  \param[in]     ip       Pointer to remote destination IP address
  \param[in]     ip_len   Length of 'ip' address in bytes
  \param[in]     port     Remote destination port number
  \return        status information
                   - number of bytes sent (>=0), if len != 0
                   - 0                            : Data can be sent (len = 0)
                   - ARM_SOCKET_ESOCK             : Invalid socket
                   - ARM_SOCKET_EINVAL            : Invalid argument (pointer to buffer or length)
                   - ARM_SOCKET_ENOTCONN          : Socket is not connected
                   - ARM_SOCKET_ECONNRESET        : Connection reset by the peer
                   - ARM_SOCKET_ECONNABORTED      : Connection aborted locally
                   - ARM_SOCKET_EAGAIN            : Operation would block or timed out (may be called again)
                   - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketSendTo (int32_t socket, const void *buf, uint32_t len, const uint8_t *ip, uint32_t ip_len, uint16_t port) {
  SOCKADDR_STORAGE addr;
  SOCKADDR_IN     *ptr_addr;
  int32_t addr_len;
  int32_t rc;
  uint8_t retry;

  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
  }

  // Check parameters
  if ((socket < 0) || (socket >= WIFI_EMW3080_SOCKETS_NUM)) {
    return ARM_SOCKET_ESOCK;
  }
  if ((buf == NULL) && (len != 0U)) {
    return ARM_SOCKET_EINVAL;
  }
  if (len == 0U) {
    return 0;
  }

  if (ip != NULL) {
    // Construct remote host address
    switch (ip_len) {
      case 4U: {
        SOCKADDR_IN *sa = (SOCKADDR_IN *)&addr;
        sa->sin_family = MX_AF_INET;
        memcpy(&sa->sin_addr, ip, 4U);
        sa->sin_port = (uint16_t)htons(port);
        ptr_addr     = (SOCKADDR_IN *)&addr;
        addr_len     = (int32_t)sizeof(SOCKADDR_IN);
        } break;
      default:
        return ARM_SOCKET_EINVAL;
    }
  } else {
    ptr_addr = NULL;
    addr_len = 0;
  }

  if (osMutexAcquire(mutex_id_sock_attr, WIFI_EMW3080_SOCKETS_TIMEOUT) == osOK) {

    // Check socket status
    if (sock_attr[socket].flags.created == 0U) {
      rc = ARM_SOCKET_ESOCK;
    } else {

      for (retry = 3U; retry != 0U; retry--) {
        rc = MX_WIFI_Socket_sendto(ptrMX_WIFIObject, socket, (uint8_t *)buf, (int32_t)len, 0, (struct mx_sockaddr *)ptr_addr, addr_len);
        if (rc > 0) {
          break;
        }
        (void)osDelay(10U);
      }
      if (rc < 0) {
        sock_attr[socket].flags.connecting = 0U;
        sock_attr[socket].flags.connected  = 0U;
        rc = ARM_SOCKET_ECONNRESET;
      }
    }

    if (osMutexRelease(mutex_id_sock_attr) != osOK) {
      rc = ARM_SOCKET_ERROR;
    }
  } else {
    rc = ARM_SOCKET_ERROR;
  }

  return rc;
}

/**
  \fn            int32_t WiFi_SocketGetSockName (int32_t socket, uint8_t *ip, uint32_t *ip_len, uint16_t *port)
  \brief         Retrieve local IP address and port of a socket.
  \param[in]     socket   Socket identification number
  \param[out]    ip       Pointer to buffer where local address shall be returned (NULL for none)
  \param[in,out] ip_len   Pointer to length of 'ip' (or NULL if 'ip' is NULL)
                   - length of supplied 'ip' on input
                   - length of stored 'ip' on output
  \param[out]    port     Pointer to buffer where local port shall be returned (NULL for none)
  \return        status information
                   - 0                            : Operation successful
                   - ARM_SOCKET_ESOCK             : Invalid socket
                   - ARM_SOCKET_EINVAL            : Invalid argument (pointer to buffer or length)
                   - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketGetSockName (int32_t socket, uint8_t *ip, uint32_t *ip_len, uint16_t *port) {
  SOCKADDR_STORAGE addr;
  int32_t addr_len = sizeof(addr);
  int32_t rc;

  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
  }

  // Check parameters
  if ((socket < 0) || (socket >= WIFI_EMW3080_SOCKETS_NUM)) {
    return ARM_SOCKET_ESOCK;
  }

  if (osMutexAcquire(mutex_id_sock_attr, WIFI_EMW3080_SOCKETS_TIMEOUT) == osOK) {

    // Check socket status
    if (sock_attr[socket].flags.created == 0U) {
      rc = ARM_SOCKET_ESOCK;
    } else if (sock_attr[socket].flags.bound == 0U) {
      rc = ARM_SOCKET_EINVAL;
    } else {

      rc = MX_WIFI_Socket_getsockname(ptrMX_WIFIObject, socket, (struct mx_sockaddr *)&addr, (uint32_t *)&addr_len);
      if (rc == 0) {                                            // If GetSockName has succeeded
        // Handle local IP address and port
        if (addr.ss_family == (uint8_t)MX_AF_INET) {
          const SOCKADDR_IN *sa = (SOCKADDR_IN *)&addr;
          if ((ip != NULL) && (ip_len != NULL)) {
            if (*ip_len >= 4U) {
              memcpy(ip, &sa->sin_addr, 4U);
              *ip_len = 4U;
              rc = 0;
            }
          }
          if (port != NULL) {
            *port = ntohs (sa->sin_port);
            rc = 0;
          }
        }   
      } else if (rc < 0) {                                      // If GetSockName has failed
        rc = ConvertSocketErrorCodeMxToCmsis(rc);
      }
    }

    if (osMutexRelease(mutex_id_sock_attr) != osOK) {
      rc = ARM_SOCKET_ERROR;
    }
  } else {
    rc = ARM_SOCKET_ERROR;
  }

  return rc;
}

/**
  \fn            int32_t WiFi_SocketGetPeerName (int32_t socket, uint8_t *ip, uint32_t *ip_len, uint16_t *port)
  \brief         Retrieve remote IP address and port of a socket.
  \param[in]     socket   Socket identification number
  \param[out]    ip       Pointer to buffer where remote address shall be returned (NULL for none)
  \param[in,out] ip_len   Pointer to length of 'ip' (or NULL if 'ip' is NULL)
                   - length of supplied 'ip' on input
                   - length of stored 'ip' on output
  \param[out]    port     Pointer to buffer where remote port shall be returned (NULL for none)
  \return        status information
                   - 0                            : Operation successful
                   - ARM_SOCKET_ESOCK             : Invalid socket
                   - ARM_SOCKET_EINVAL            : Invalid argument (pointer to buffer or length)
                   - ARM_SOCKET_ENOTCONN          : Socket is not connected
                   - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketGetPeerName (int32_t socket, uint8_t *ip, uint32_t *ip_len, uint16_t *port) {
  SOCKADDR_STORAGE addr;
  int32_t addr_len = sizeof(addr);
  int32_t rc;

  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
  }

  // Check parameters
  if ((socket < 0) || (socket >= WIFI_EMW3080_SOCKETS_NUM)) {
    return ARM_SOCKET_ESOCK;
  }

  if (osMutexAcquire(mutex_id_sock_attr, WIFI_EMW3080_SOCKETS_TIMEOUT) == osOK) {

    // Check socket status
    if (sock_attr[socket].flags.created == 0U) {
      rc = ARM_SOCKET_ESOCK;
    } else if (sock_attr[socket].flags.connected == 0U) {
      rc = ARM_SOCKET_ENOTCONN;
    } else {

      rc = MX_WIFI_Socket_getpeername(ptrMX_WIFIObject, socket, (struct mx_sockaddr *)&addr, (uint32_t *)&addr_len);
      if (rc == 0) {                                            // If SocketGetPeerName has succeeded
        // Handle remote IP address and port
        if (addr.ss_family == (uint8_t)MX_AF_INET) {
          const SOCKADDR_IN *sa = (SOCKADDR_IN *)&addr;
          if ((ip != NULL) && (ip_len != NULL)) {
            if (*ip_len >= 4U) {
              memcpy(ip, &sa->sin_addr, 4U);
              *ip_len = 4U;
            }
          }
          if (port != NULL) {
            *port = ntohs (sa->sin_port);
          }
        }
      } else if (rc < 0) {                                      // If SocketGetPeerName has failed
        rc = ConvertSocketErrorCodeMxToCmsis(rc);
      }
    }

    if (osMutexRelease(mutex_id_sock_attr) != osOK) {
      rc = ARM_SOCKET_ERROR;
    }
  } else {
    rc = ARM_SOCKET_ERROR;
  }


  return rc;
}

/**
  \fn            int32_t WiFi_SocketGetOpt (int32_t socket, int32_t opt_id, void *opt_val, uint32_t *opt_len)
  \brief         Get socket option.
  \param[in]     socket   Socket identification number
  \param[in]     opt_id   Option identifier
  \param[out]    opt_val  Pointer to the buffer that will receive the option value
  \param[in,out] opt_len  Pointer to length of the option value
                   - length of buffer on input
                   - length of data on output
  \return        status information
                   - 0                            : Operation successful
                   - ARM_SOCKET_ESOCK             : Invalid socket
                   - ARM_SOCKET_EINVAL            : Invalid argument
                   - ARM_SOCKET_ENOTSUP           : Operation not supported
                   - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketGetOpt (int32_t socket, int32_t opt_id, void *opt_val, uint32_t *opt_len) {
  int32_t rc;
  uint32_t len;

  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
  }

  // Check parameters
  if ((socket < 0) || (socket >= WIFI_EMW3080_SOCKETS_NUM)) {
    return ARM_SOCKET_ESOCK;
  }
  if ((opt_val == NULL) || (opt_len == NULL) || (*opt_len < 4U)) {
    return ARM_SOCKET_EINVAL;
  }

  // All supported options are 4 bytes long
  len = *opt_len;
  if (len > 4U) {
    len = 4U;
  }

  if (osMutexAcquire(mutex_id_sock_attr, WIFI_EMW3080_SOCKETS_TIMEOUT) == osOK) {

    // Check socket status
    if (sock_attr[socket].flags.created == 0U) {
      rc = ARM_SOCKET_ESOCK;
    } else {

      switch (opt_id) {
        case ARM_SOCKET_IO_FIONBIO:
          rc = ARM_SOCKET_EINVAL;
          break;
        case ARM_SOCKET_SO_RCVTIMEO:
//        MX_WIFI_Socket_getsockopt function returns error, so value is taken from sock_attr
//        rc = MX_WIFI_Socket_getsockopt(ptrMX_WIFIObject, socket, MX_SOL_SOCKET, (int32_t)MX_SO_RCVTIMEO,  opt_val, &len);
          *((uint32_t *)opt_val) = sock_attr[socket].rcvtimeo;
          *opt_len = 4U;
          rc = 0;
          break;
        case ARM_SOCKET_SO_SNDTIMEO:
//        MX_WIFI_Socket_getsockopt function returns error, so value is taken from sock_attr
//        rc = MX_WIFI_Socket_getsockopt(ptrMX_WIFIObject, socket, MX_SOL_SOCKET, (int32_t)MX_SO_SNDTIMEO,  opt_val, &len);
          *((uint32_t *)opt_val) = sock_attr[socket].sndtimeo;
          *opt_len = 4U;
          rc = 0;
          break;
        case ARM_SOCKET_SO_KEEPALIVE:
          rc = MX_WIFI_Socket_getsockopt(ptrMX_WIFIObject, socket, MX_SOL_SOCKET, (int32_t)MX_SO_KEEPALIVE, opt_val, &len);
          if (rc == 0) {
            *opt_len = len;
          } else if (rc < 0) {
            rc = ConvertSocketErrorCodeMxToCmsis(rc);
          }
          break;
        case ARM_SOCKET_SO_TYPE:
          rc = MX_WIFI_Socket_getsockopt(ptrMX_WIFIObject, socket, MX_SOL_SOCKET, (int32_t)MX_SO_TYPE,      opt_val, &len);
          if (rc == 0) {
            *opt_len = len;
          } else if (rc < 0) {
            rc = ConvertSocketErrorCodeMxToCmsis(rc);
          }
          break;
        default:
          rc = ARM_SOCKET_EINVAL;
          break;
      }
    }

    if (osMutexRelease(mutex_id_sock_attr) != osOK) {
      rc = ARM_SOCKET_ERROR;
    }
  } else {
    rc = ARM_SOCKET_ERROR;
  }

  return rc;
}

/**
  \fn            int32_t WiFi_SocketSetOpt (int32_t socket, int32_t opt_id, const void *opt_val, uint32_t opt_len)
  \brief         Set socket option.
  \param[in]     socket   Socket identification number
  \param[in]     opt_id   Option identifier
  \param[in]     opt_val  Pointer to the option value
  \param[in]     opt_len  Length of the option value in bytes
  \return        status information
                   - 0                            : Operation successful
                   - ARM_SOCKET_ESOCK             : Invalid socket
                   - ARM_SOCKET_EINVAL            : Invalid argument
                   - ARM_SOCKET_ENOTSUP           : Operation not supported
                   - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketSetOpt (int32_t socket, int32_t opt_id, const void *opt_val, uint32_t opt_len) {
  int32_t rc;

  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
  }

  // Check parameters
  if ((socket < 0) || (socket >= WIFI_EMW3080_SOCKETS_NUM)) {
    return ARM_SOCKET_ESOCK;
  }
  if ((opt_val == NULL) || (opt_len != 4U)) {
    return ARM_SOCKET_EINVAL;
  }

  if (osMutexAcquire(mutex_id_sock_attr, WIFI_EMW3080_SOCKETS_TIMEOUT) == osOK) {

    // Check socket status
    if (sock_attr[socket].flags.created == 0U) {
      rc = ARM_SOCKET_ESOCK;
    } else {

      switch (opt_id) {
        case ARM_SOCKET_IO_FIONBIO:
          if (*((uint32_t *)opt_val) != 0U) {
            sock_attr[socket].ionbio = 1U;
          } else {
            sock_attr[socket].ionbio = 0U;
          }
          rc = 0;
          break;
        case ARM_SOCKET_SO_RCVTIMEO:
          // Timeout for reception is always 1 ms, it is set in create function.
          // Blocking mode is emulated by periodically polling for reception
          // until timeout, so SPI is not blocked for long time, 
          // for non-blocking mode 1 ms timeout is used.
          sock_attr[socket].rcvtimeo = *((uint32_t *)opt_val);
          rc = 0;
          break;
        case ARM_SOCKET_SO_SNDTIMEO:
          rc = MX_WIFI_Socket_setsockopt(ptrMX_WIFIObject, socket, MX_SOL_SOCKET, (int32_t)MX_SO_SNDTIMEO,  opt_val, 4);
          if (rc == 0) {
            sock_attr[socket].sndtimeo = *((uint32_t *)opt_val);
          }
          break;
        case ARM_SOCKET_SO_KEEPALIVE:
          rc = MX_WIFI_Socket_setsockopt(ptrMX_WIFIObject, socket, MX_SOL_SOCKET, (int32_t)MX_SO_KEEPALIVE, opt_val, 4);
          break;
        case ARM_SOCKET_SO_TYPE:
          rc = ARM_SOCKET_EINVAL;
          break;
        default:
          rc = ARM_SOCKET_EINVAL;
          break;
      }
    }

    if (osMutexRelease(mutex_id_sock_attr) != osOK) {
      rc = ARM_SOCKET_ERROR;
    }
  } else {
    rc = ARM_SOCKET_ERROR;
  }

  return rc;
}

/**
  \fn            int32_t WiFi_SocketClose (int32_t socket)
  \brief         Close and release a socket.
  \param[in]     socket   Socket identification number
  \return        status information
                   - 0                            : Operation successful
                   - ARM_SOCKET_ESOCK             : Invalid socket
                   - ARM_SOCKET_EAGAIN            : Operation would block (may be called again)
                   - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketClose (int32_t socket) {
  int32_t rc;

  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
  }

  // Check parameters
  if ((socket < 0) || (socket >= WIFI_EMW3080_SOCKETS_NUM)) {
    return ARM_SOCKET_ESOCK;
  }

  if (osMutexAcquire(mutex_id_sock_attr, WIFI_EMW3080_SOCKETS_TIMEOUT) == osOK) {

    // Check socket status
    if (sock_attr[socket].flags.created == 0U) {
      rc = ARM_SOCKET_ESOCK;
    } else {
      rc = MX_WIFI_Socket_close(ptrMX_WIFIObject, socket);
      if (rc == 0) {                                              // If close has succeeded
        memset (&sock_attr[socket], 0, sizeof(sock_attr[0]));
      } else if (rc < 0) {                                        // If close has failed
        rc = ConvertSocketErrorCodeMxToCmsis(rc);
      }
    }

    if (osMutexRelease(mutex_id_sock_attr) != osOK) {
      rc = ARM_SOCKET_ERROR;
    }
  } else {
    rc = ARM_SOCKET_ERROR;
  }

  return rc;
}

/**
  \fn            int32_t WiFi_SocketGetHostByName (const char *name, int32_t af, uint8_t *ip, uint32_t *ip_len)
  \brief         Retrieve host IP address from host name.
  \param[in]     name     Host name
  \param[in]     af       Address family
  \param[out]    ip       Pointer to buffer where resolved IP address shall be returned
  \param[in,out] ip_len   Pointer to length of 'ip'
                   - length of supplied 'ip' on input
                   - length of stored 'ip' on output
  \return        status information
                   - 0                            : Operation successful
                   - ARM_SOCKET_EINVAL            : Invalid argument
                   - ARM_SOCKET_ENOTSUP           : Operation not supported
                   - ARM_SOCKET_ETIMEDOUT         : Operation timed out
                   - ARM_SOCKET_EHOSTNOTFOUND     : Host not found
                   - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketGetHostByName (const char *name, int32_t af, uint8_t *ip, uint32_t *ip_len) {
  SOCKADDR_STORAGE addr;
  int32_t rc;

  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
  }

  // Check parameters
  if ((name == NULL) || (ip == NULL) || (ip_len == NULL)) {
    return ARM_SOCKET_EINVAL;
  }
  switch (af) {
    case ARM_SOCKET_AF_INET:
      if (*ip_len < 4U) {
        return ARM_SOCKET_EINVAL;
      }
      break;
    default:
      return ARM_SOCKET_EINVAL;
  }

  // Resolve hostname
  rc = MX_WIFI_Socket_gethostbyname(ptrMX_WIFIObject, (struct mx_sockaddr *)&addr, (char *)name);
  if (rc < 0) {
    if (rc == MX_WIFI_STATUS_ERROR) {
      // Consider MX_WIFI_STATUS_ERROR means that host was not found
      return ARM_SOCKET_EHOSTNOTFOUND;
    } else {
      rc = ConvertSocketErrorCodeMxToCmsis(rc);
      return rc;
    }
  }

  // Copy resolved IP address
  if (addr.ss_family == (uint8_t)MX_AF_INET) {
    const SOCKADDR_IN *sa = (SOCKADDR_IN *)&addr;
    if (*ip_len >= sizeof(sa->sin_addr)) {
      memcpy(ip, &sa->sin_addr, sizeof(sa->sin_addr));
      *ip_len = sizeof(sa->sin_addr);
    }
  }

  return 0;
}

/**
  \fn            int32_t WiFi_Ping (const uint8_t *ip, uint32_t ip_len)
  \brief         Probe remote host with Ping command.
  \param[in]     ip       Pointer to remote host IP address
  \param[in]     ip_len   Length of 'ip' address in bytes
  \return        execution status
                   - ARM_DRIVER_OK                : Operation successful
                   - ARM_DRIVER_ERROR             : Operation failed
                   - ARM_DRIVER_ERROR_TIMEOUT     : Timeout occurred
                   - ARM_DRIVER_ERROR_UNSUPPORTED : Operation not supported
                   - ARM_DRIVER_ERROR_PARAMETER   : Parameter error (NULL ip pointer or ip_len different than 4 or 16)
*/
static int32_t WiFi_Ping (const uint8_t *ip, uint32_t ip_len) {
  char str_addr[16];
  int32_t response_time;
  int32_t rc, str_rc;

  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
  }

  // Check parameters
  if ((ip == NULL) || (ip_len != 4U)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  str_rc = snprintf(str_addr, 16, "%i.%i.%i.%i", ip[0], ip[1], ip[2], ip[3]);
  if ((str_rc < 0) || (str_rc > 15)) {
    return ARM_SOCKET_ERROR;
  }

  rc = MX_WIFI_Socket_ping(ptrMX_WIFIObject, (const char *)str_addr, 1, 0, &response_time);
  rc = ConvertSocketErrorCodeMxToCmsis(rc);

  return rc;
}


// Structure exported by driver Driver_WiFin (default: Driver_WiFi0)

ARM_DRIVER_WIFI ARM_Driver_WiFi_(WIFI_EMW3080_DRV_NUM) = { 
  WiFi_GetVersion,
  WiFi_GetCapabilities,
  WiFi_Initialize,
  WiFi_Uninitialize,
  WiFi_PowerControl,
  WiFi_GetModuleInfo,
  WiFi_SetOption,
  WiFi_GetOption,
  WiFi_Scan,
  WiFi_Activate,
  WiFi_Deactivate,
  WiFi_IsConnected,
  WiFi_GetNetInfo,
  NULL,
  NULL,
  NULL,
  NULL,
  WiFi_SocketCreate,
  WiFi_SocketBind,
  WiFi_SocketListen,
  WiFi_SocketAccept,
  WiFi_SocketConnect,
  WiFi_SocketRecv,
  WiFi_SocketRecvFrom,
  WiFi_SocketSend,
  WiFi_SocketSendTo,
  WiFi_SocketGetSockName,
  WiFi_SocketGetPeerName,
  WiFi_SocketGetOpt,
  WiFi_SocketSetOpt,
  WiFi_SocketClose,
  WiFi_SocketGetHostByName,
  WiFi_Ping
};
