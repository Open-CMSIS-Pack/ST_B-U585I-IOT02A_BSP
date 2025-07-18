/******************************************************************************
 * @file     vstream_accelerometer.c
 * @brief    CMSIS Virtual Streaming interface Driver implementation for
 *           Accelerometer sensor (ISM330DHCX) on the
 *           STMicroelectronics B-U585I-IOT02A board
 * @version  V1.0.0
 * @date     11. April 2025
 ******************************************************************************/
/*
 * Copyright (c) 2025 Arm Limited. All rights reserved.
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
 */

#include <stddef.h>
#include <string.h>
#include <stdio.h>

#include "vstream_accelerometer_config.h"
#include "vstream_accelerometer.h"

#include "RTE_Components.h"
#include CMSIS_device_header

#include "cmsis_os2.h"

#include "b_u585i_iot02a_bus.h"
#include "b_u585i_iot02a_env_sensors.h"
#include "b_u585i_iot02a_motion_sensors.h"

#include "ism330dhcx.h"
#include "ism330dhcx_fifo.h"


// Local macros ------------------------

// Flags for polling thread
#define FLAG_POLLING_START              (1U)
#define FLAG_POLLING_STOP               (1U << 1)
#define FLAG_POLLING_THREAD_TERMINATE   (1U << 2)
#define MASK_POLLING_FLAGS              (0x07U)


// Local typedefs ----------------------

// Raw accelerometer sensor sample structure
typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} sensor_sample_raw_t;

// vStream driver runtime information structure
typedef struct {
           vStreamEvent_t  fn_event_cb;                 // Event handling callback function
           uint8_t        *data_buf;                    // Buffer for sensor data
           uint32_t        data_buf_size;               // Size of sensor data buffer
           uint32_t        data_block_size;             // Size of sensor data block
  volatile uint8_t        *data_in_ptr;                 // Pointer to where new incoming sensor data is stored (inside data buffer)
  volatile uint8_t        *data_rd_ptr;                 // Pointer to oldest unread data (inside data buffer)
  volatile uint32_t        data_in_cnt;                 // Count of sensor-acquired bytes in data buffer
  volatile uint32_t        data_rd_cnt;                 // Count of bytes read from data buffer
  volatile osThreadId_t    threadId_threadPolling;      // Sensor data polling thread ID
  volatile uint8_t         active;                      // Streaming (data acquisition) active status
  volatile uint8_t         overflow;                    // Data buffer overflow status
           uint8_t         sampling_mode;               // Sampling mode selected on Start (VSTREAM_MODE_CONTINUOUS or VSTREAM_MODE_SINGLE)
} vstream_info_t;


// Local variables ---------------------

static vstream_info_t vstream_info;     // vStream driver runtime information


// Local function prototypes -----------

static int32_t Initialize   (vStreamEvent_t event_cb);
static int32_t Uninitialize (void);
static int32_t SetBuf       (void *buf, uint32_t buf_size, uint32_t block_size);
static int32_t Start        (uint32_t mode);
static int32_t Stop         (void);
static void *  GetBlock     (void);
static int32_t ReleaseBlock (void);

static __NO_RETURN void threadPollingAccelerometer (void *argument);


// Local function definitions ----------

/**
  \fn           int32_t Initialize (vStreamEvent_t event_cb)
  \brief        Initialize Virtual Streaming interface.
  \return       VSTREAM_OK on success; otherwise, an appropriate error code
*/
static int32_t Initialize (vStreamEvent_t event_cb) {

  // Clear vStream runtime information
  memset(&vstream_info, 0, sizeof(vstream_info));

  // Register event callback function
  vstream_info.fn_event_cb = event_cb;

  // Initialize and configure accelerometer sensor
  if (BSP_MOTION_SENSOR_Init(0U, MOTION_ACCELERO) != BSP_ERROR_NONE) {
    return VSTREAM_ERROR;
  }

  // Configure accelerometer scale to 2 G
  if (BSP_MOTION_SENSOR_SetFullScale(0, MOTION_ACCELERO, 2) != BSP_ERROR_NONE) {
    return VSTREAM_ERROR;
  }

  // Configure accelerometer sampling rate
  if (BSP_MOTION_SENSOR_SetOutputDataRate(0, MOTION_ACCELERO, (float)SENSOR_SAMPLING_RATE) != BSP_ERROR_NONE) {
    return VSTREAM_ERROR;
  }

  // Configure accelerometer sampling rate for FIFO to same as sampling rate for sensor
  if (ISM330DHCX_FIFO_ACC_Set_BDR(&ISM330DHCX_Obj, (float)SENSOR_SAMPLING_RATE) != BSP_ERROR_NONE) {
    return VSTREAM_ERROR;
  }

  // Enable FIFO mode for accelerometer
  if (ISM330DHCX_FIFO_Set_Mode(&ISM330DHCX_Obj, ISM330DHCX_STREAM_MODE) != BSP_ERROR_NONE) {
    return VSTREAM_ERROR;
  }

  // Start sensor polling thread if it is not already running
  if (vstream_info.threadId_threadPolling == NULL) {
    vstream_info.threadId_threadPolling = osThreadNew(threadPollingAccelerometer, NULL, NULL);
  }

  // If thread was not created successfully return error
  if (vstream_info.threadId_threadPolling == NULL) {
    return VSTREAM_ERROR;
  }

  return VSTREAM_OK;
}

/**
  \fn           int32_t Uninitialize (void)
  \brief        De-initialize Virtual Streaming interface.
  \return       VSTREAM_OK on success; otherwise, an VSTREAM_ERROR error code
*/
static int32_t Uninitialize (void) {

  // De-register event callback function
  vstream_info.fn_event_cb = NULL;

  // If the sensor polling thread exists, set flag to self-terminate it in a controlled manner
  // and wait for it to be terminated
  if (vstream_info.threadId_threadPolling != NULL) {
    (void)osThreadFlagsSet(vstream_info.threadId_threadPolling, FLAG_POLLING_THREAD_TERMINATE);

    for (uint8_t i = 0U; i <= 10U; i++) {
      if (vstream_info.threadId_threadPolling == NULL) {        // If polling thread has self-terminated
        break;
      }
      if (i == 10U) {                                           // If sampling thread did not terminate in 1000 OS ticks
        return VSTREAM_ERROR;
      }
      (void)osDelay(100U);
    }
  }

  // Disable accelerometer
  (void)BSP_MOTION_SENSOR_Disable(0U, MOTION_ACCELERO);

  // De-initialize accelerometer
  BSP_MOTION_SENSOR_DeInit(0U);

  return VSTREAM_OK;
}

/**
  \fn           int32_t SetBuf (void *buf, uint32_t buf_size, uint32_t block_size)
  \brief        Set Virtual Streaming data buffer.
  \param[in]    buf             pointer to memory buffer used for streaming data
  \param[in]    buf_size        total size of the streaming data buffer (in bytes)
  \param[in]    block_size      streaming data block size (in bytes)
  \return       VSTREAM_OK on success; otherwise, an appropriate error code
*/
static int32_t SetBuf (void *buf, uint32_t buf_size, uint32_t block_size) {

  // Check if parameters are not valid
  if ((buf == NULL) || (buf_size == 0U) || (block_size == 0U) || (block_size > buf_size)) {
    return VSTREAM_ERROR_PARAMETER;
  }

  // Check if block size is not an integer multiple of sample size
  if ((block_size % sizeof(sensor_sample_raw_t)) != 0U) {
    return VSTREAM_ERROR_PARAMETER;
  }

  // Register buffer information
  vstream_info.data_buf        = (uint8_t *)buf;
  vstream_info.data_buf_size   = (buf_size / block_size) * block_size;       // Buf size rounded to block size
  vstream_info.data_block_size = block_size;

  // Initialize data pointers
  vstream_info.data_in_ptr     = (uint8_t *)buf;
  vstream_info.data_rd_ptr     = (uint8_t *)buf;

  return VSTREAM_OK;
}

/**
  \fn           int32_t Start (uint32_t mode)
  \brief        Start streaming.
  \param[in]    mode            streaming mode
  \return       VSTREAM_OK on success; otherwise, an appropriate error code
*/
static int32_t Start (uint32_t mode) {

  // Check if streaming is already active and return VSTREAM_OK if it is so
  if (vstream_info.active != 0U) {
    return VSTREAM_OK;
  }

  // Check if parameters are not valid
  if ((mode != VSTREAM_MODE_CONTINUOUS) && (mode != VSTREAM_MODE_SINGLE)) {
    return VSTREAM_ERROR_PARAMETER;
  }

  // Check if data buffer address is not valid
  if (vstream_info.data_buf == NULL) {
    return VSTREAM_ERROR;
  }

  // Check if sensor polling thread does not exists
  if (vstream_info.threadId_threadPolling == NULL) {
    return VSTREAM_ERROR;
  }

  // Register sampling mode
  vstream_info.sampling_mode = mode;

  // Set polling thread flag to start sampling
  (void)osThreadFlagsSet(vstream_info.threadId_threadPolling, FLAG_POLLING_START);

  return VSTREAM_OK;
}

/**
  \fn           int32_t Stop (void)
  \brief        Stop streaming.
  \return       VSTREAM_OK on success; otherwise, an VSTREAM_ERROR error code
*/
static int32_t Stop (void) {

  // Check if streaming is not active and return VSTREAM_OK if it is so
  if (vstream_info.active == 0U) {
    return VSTREAM_OK;
  }

  // Check if sensor polling thread does not exists
  if (vstream_info.threadId_threadPolling == NULL) {
    return VSTREAM_ERROR;
  }

  // If the sensor polling thread exists, set flag to stop sampling and wait for it to be stop sampling
  if (vstream_info.threadId_threadPolling != NULL) {
    (void)osThreadFlagsSet(vstream_info.threadId_threadPolling, FLAG_POLLING_STOP);

    for (uint8_t i = 0U; i <= 100U; i++) {
      if (vstream_info.active == 0U) {                          // If sampling thread stopped sampling
        break;
      }
      if (i == 100U) {                                          // If sampling thread did not terminate in 1000 OS ticks
        return VSTREAM_ERROR;
      }
      (void)osDelay(100U);
    }
  }

  // Reset data counters (flush data)
  vstream_info.data_in_cnt = 0U;
  vstream_info.data_rd_cnt = 0U;

  // Reset data pointers
  vstream_info.data_in_ptr = vstream_info.data_buf;
  vstream_info.data_rd_ptr = vstream_info.data_buf;

  return VSTREAM_OK;
}

/**
  \fn           void *GetBlock (void)
  \brief        Get pointer to Virtual Streaming data block.
  \return       pointer to data block, returns NULL if no block is available
*/
static void *GetBlock (void) {

  // Check if buffer information is not valid
  if (vstream_info.data_buf == NULL) {
    return NULL;
  }

  // Check if size of available data is less than 1 block
  if ((vstream_info.data_in_cnt - vstream_info.data_rd_cnt) < vstream_info.data_block_size) {
    return NULL;
  }

  // Return pointer to oldest unread data block
  return ((void *)vstream_info.data_rd_ptr);
}

/**
  \fn           int32_t ReleaseBlock (void)
  \brief        Release Virtual Streaming data block.
  \return       VSTREAM_OK on success; otherwise, an VSTREAM_ERROR error code
*/
static int32_t ReleaseBlock (void) {

  // Check if buffer information is not valid
  if (vstream_info.data_buf == NULL) {
    return VSTREAM_ERROR;
  }

  // Check if size of available data is less than 1 block
  if ((vstream_info.data_in_cnt - vstream_info.data_rd_cnt) < vstream_info.data_block_size) {
    return VSTREAM_ERROR;
  }

  // Increment read data counter by data block size
  vstream_info.data_rd_cnt += vstream_info.data_block_size;

  // If pointer to last unread block would cross end of data buffer -> wrap to start of data buffer
  // else increment pointer to oldest unread block by data block size
  if ((vstream_info.data_rd_ptr + vstream_info.data_block_size) >= (vstream_info.data_buf + vstream_info.data_buf_size)) {
    vstream_info.data_rd_ptr  = vstream_info.data_buf;
  } else {
    vstream_info.data_rd_ptr += vstream_info.data_block_size;
  }

  return VSTREAM_OK;
}

/**
  \fn           vStreamStatus_t GetStatus (void)
  \brief        Get Virtual Streaming status.
  \return       streaming status structure
*/
static vStreamStatus_t GetStatus (void) {
  vStreamStatus_t stat = { 0U, 0U, 0U, 0U, 0U };

  // Handle active flag
  if (vstream_info.active != 0U) {
    stat.active = 1U;
  }

  // Handle overflow flag
  if (vstream_info.overflow != 0U) {
    vstream_info.overflow = 0U;
    stat.overflow = 1U;
  }

  // Underflow cannot happen on input stream
  // EOS cannot happen with accelerometer

  return stat;
}

// Thread: Polling of data from sensor (FIFO) and storing it into data buffer
static __NO_RETURN void threadPollingAccelerometer (void *argument) {
  uint32_t            flags;
  uint32_t            timestamp;
  uint32_t            samples_read_num;
  uint32_t            bytes_read_num;
  uint32_t            max_samples_to_read_num;
  uint32_t            in_rd_cnt_diff;
  sensor_sample_raw_t sample_raw[2];
  uint8_t             discard_initial_samples_num;
  uint32_t            events;
  (void) argument;

  for (;;) {
    flags = osThreadFlagsWait(MASK_POLLING_FLAGS, osFlagsWaitAny, osWaitForever);

    // If there was error retrieving flags -> self-terminate this thread
    if ((flags & osFlagsError) != 0U) {
      vstream_info.threadId_threadPolling = NULL;
      vstream_info.active = 0U;
      osThreadExit();
    }

    // Is sampling was requested by Start function
    if ((flags & FLAG_POLLING_START) != 0U) {
      flags &= ~FLAG_POLLING_START;

      vstream_info.active = 1U;

      // Start accelerometer sampling with FIFO mode
      ISM330DHCX_FIFO_Init(ISM330DHCX_ID_ACCELEROMETER);
      ISM330DHCX_ACC_Enable(&ISM330DHCX_Obj);

      // Register number of initial sample(s) that should be discarded due to accelerometer turn-on/off time
      discard_initial_samples_num = SENSOR_STARTUP_DISCARD_SAMPLES;

      timestamp = osKernelGetTickCount();                       // Register initial timestamp for polling interval handling

      for (;;) {
        flags |= osThreadFlagsGet();                            // Get any new flags

        // If stop flag was set -> clear active status, disable accelerometer and FIFO and exit polling
        if ((flags & FLAG_POLLING_STOP) != 0U) {
          vstream_info.active = 0U;
          ISM330DHCX_ACC_Disable(&ISM330DHCX_Obj);
          ISM330DHCX_FIFO_Uninit(ISM330DHCX_ID_ACCELEROMETER);
          break;
        }

        // If terminate flag was set -> exit polling loop
        if ((flags & FLAG_POLLING_THREAD_TERMINATE) != 0U) {
          break;
        }

        // Determine if 2 samples can be read if they are available, if there is only space for
        // 1 sample before block is filled read only 1 sample because it might be necessary to
        // generate event or that pointer to where next incoming sample will be written rolls over
        // also if number of initial samples to discard is not 0, read only 1 sample per cycle
        max_samples_to_read_num = 2U;
        if (((vstream_info.data_in_cnt - vstream_info.data_rd_cnt) + sizeof(sensor_sample_raw_t)) == vstream_info.data_block_size) {
          max_samples_to_read_num = 1U;
        }
        if (discard_initial_samples_num != 0U) {
          max_samples_to_read_num = 1U;
        }

        // Try to read max_samples_to_read_num of samples
        samples_read_num = ISM330DHCX_FIFO_Read(ISM330DHCX_ID_ACCELEROMETER, max_samples_to_read_num, (uint8_t *)sample_raw);

        // Discard initial sample(s) due to accelerometer turn-on/off time
        if ((samples_read_num != 0U) && (discard_initial_samples_num != 0U)) {
          discard_initial_samples_num--;
          continue;
        }

        if (samples_read_num != 0U) {

          // Provision for printf debugging, printing number of samples read and raw data of first read sample
          // printf("num = %d, Xraw = %d, Yraw = %d, Zraw = %d\n", samples_read_num, sample_raw[0].x, sample_raw[0].y, sample_raw[0].z);

          // Calculate number of bytes read
          bytes_read_num = samples_read_num * sizeof(sensor_sample_raw_t);

          // Put new sample into data buffer
          memcpy((void *)vstream_info.data_in_ptr, (const void *)sample_raw, bytes_read_num);

          // Increment input data counter by newly added size
          vstream_info.data_in_cnt += bytes_read_num;

          // If pointer to where next incoming sample will be written would cross end of data buffer -> wrap to start of data buffer
          // else increment pointer to where next incoming sample will be written
          if ((vstream_info.data_in_ptr + bytes_read_num) >= (vstream_info.data_buf + vstream_info.data_buf_size)) {
            vstream_info.data_in_ptr  = vstream_info.data_buf;
          } else {
            vstream_info.data_in_ptr += bytes_read_num;
          }

          events = 0U;

          // Difference between number of incoming and read out samples
          in_rd_cnt_diff = vstream_info.data_in_cnt - vstream_info.data_rd_cnt;

          // If incoming data started overwriting unread data -> register overflow
          if (in_rd_cnt_diff > vstream_info.data_buf_size) {
            vstream_info.overflow = 1U;
            events = VSTREAM_EVENT_OVERFLOW;
          }

          // If available data size reached multiple of block size -> prepare DATA event
          // If mode is VSTREAM_MODE_SINGLE stop further sampling
          if ((in_rd_cnt_diff > 0U) && ((in_rd_cnt_diff % vstream_info.data_block_size) == 0U)) {

            // If single mode sampling -> clear active status and disable accelerometer and FIFO
            if (vstream_info.sampling_mode == VSTREAM_MODE_SINGLE) {
              vstream_info.active = 0U;
              ISM330DHCX_ACC_Disable(&ISM330DHCX_Obj);
              ISM330DHCX_FIFO_Uninit(ISM330DHCX_ID_ACCELEROMETER);
            }

            events |= VSTREAM_EVENT_DATA;
          }

          // If signal function was registered -> signal active events
          if ((vstream_info.fn_event_cb != NULL) && (events != 0U)) {
            vstream_info.fn_event_cb(events);
          }

          // If 1 block got filled and single mode sampling is active -> exit the loop thus stop polling
          if (((events & VSTREAM_EVENT_DATA) != 0U) && (vstream_info.sampling_mode == VSTREAM_MODE_SINGLE)) {
            break;
          }
        }

        timestamp += SENSOR_POLLING_INTERVAL;
        (void)osDelayUntil(timestamp);          // Wait until next sampling interval
      }
    }

    // If flag to terminate thread was set -> self-terminate this thread
    if ((flags & FLAG_POLLING_THREAD_TERMINATE) != 0U) {
      vstream_info.threadId_threadPolling = NULL;
      vstream_info.active = 0U;
      osThreadExit();
    }
  }
}


// Global driver structure

vStreamDriver_t Driver_vStreamAccelerometer = {
  Initialize,
  Uninitialize,
  SetBuf,
  Start,
  Stop,
  GetBlock,
  ReleaseBlock,
  GetStatus
};
