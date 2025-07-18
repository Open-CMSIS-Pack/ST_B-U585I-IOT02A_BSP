/******************************************************************************
 * @file     vstream_audio_in.c
 * @brief    CMSIS Virtual Streaming interface Driver implementation for
 *           Audio In (microphone) on the
 *           STMicroelectronics B-U585I-IOT02A board
 * @version  V1.0.0
 * @date     14. July 2025
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

/*
  IMPORTANT NOTES:

  Driver Capabilities

  This audio input driver currently supports the following features and has the following limitations:
  1. Mono Mode Only:
     Stereo recording of interleaved data (L, R, L, R, ...) is not supported due to hardware limitations.
     The underlying hardware supports capturing Left and Right channels into separate buffers only.
     If stereo output is required, interleaving must be performed in software.
  2. 16-bit Samples Only:
     The driver is limited to 16 bits per sample due to constraints in the BSP audio implementation.
  3. Streaming Buffer Size:
     The streaming data buffer must be twice the size of the streaming data block,
     to enable double-buffering with DMA.
  4. Maximum Streaming Buffer Size:
     The total streaming data buffer size must be less than 65535 bytes, as per BSP audio driver limitations.

  Driver Functionality Overview

  The BSP_AUDIO_IN_Record function initializes and starts audio capture using DMA in linked-list circular mode.
  During recording:
  - when the first half of the buffer is filled, the BSP_AUDIO_IN_HalfTransfer_CallBack is invoked.
  - when the entire buffer is filled, the BSP_AUDIO_IN_TransferComplete_CallBack is triggered.
  These callbacks are driven by DMA interrupts, specifically:
  - GPDMA1_Channel0_IRQHandler and GPDMA1_Channel6_IRQHandler
    Additional handlers implemented in the stm32u5xx_it.c file are necessary for invoking the 
    BSP_AUDIO_IN_HalfTransfer_CallBack and BSP_AUDIO_IN_TransferComplete_CallBack callbacks.

  Note: Immediately after calling BSP_AUDIO_IN_Record, the BSP_AUDIO_IN_TransferComplete_CallBack
  is called once prematurely, before any audio data is recorded.
*/


#include <stddef.h>
#include <string.h>

#include "vstream_audio_in_config.h"
#include "vstream_audio_in.h"

#include "RTE_Components.h"
#include CMSIS_device_header

#include "b_u585i_iot02a_audio.h"


// Local macros ------------------------

// Check compile-time configuration

#if (AUDIO_IN_MODE != 0)
#error "Only mono mode is supported (AUDIO_IN_MODE)!"
#endif

#if (AUDIO_IN_BITS_PER_SAMPLE != 16)
#error "This driver supports only 16 bits per sample (AUDIO_IN_BITS_PER_SAMPLE)!"
#endif

#if ((AUDIO_IN_SAMPLING_RATE < 8000) || (AUDIO_IN_SAMPLING_RATE > 192000))
#error "Sampling rate must be in range 8 kHz to 192 kHz (AUDIO_IN_SAMPLING_RATE)!"
#endif


// Local typedefs ----------------------

// vStream driver runtime information structure
typedef struct vstream_info_s {
           vStreamEvent_t       fn_event_cb;            // Event handling callback function
           uint8_t             *data_buf;               // Buffer for audio data
  volatile uint32_t             data_block_in_cnt;      // Count of recorded data blocks
  volatile uint32_t             data_block_rd_cnt;      // Count of read data blocks
           uint16_t             data_block_size;        // Size of audio data block
  volatile uint8_t              ignore_callback;        // Flag to ignore callback
  volatile uint8_t              streaming_active;       // Streaming (data acquisition) active status
  volatile uint8_t              data_overflow;          // Data buffer overflow status
} vstream_info_t;


// Local variables ---------------------

static vstream_info_t vstream_info;     // vStream driver runtime information


// Local function prototypes -----------

static int32_t  Initialize      (vStreamEvent_t event_cb);
static int32_t  Uninitialize    (void);
static int32_t  SetBuf          (void *buf, uint32_t buf_size, uint32_t block_size);
static int32_t  Start           (uint32_t mode);
static int32_t  Stop            (void);
static void *   GetBlock        (void);
static int32_t  ReleaseBlock    (void);

static void     NewBlockRecorded(void);


// Local function definitions ----------

/**
  \fn           int32_t Initialize (vStreamEvent_t event_cb)
  \brief        Initialize Virtual Streaming interface.
  \return       VSTREAM_OK on success; otherwise, an appropriate error code
*/
static int32_t Initialize (vStreamEvent_t event_cb) {
  BSP_AUDIO_Init_t AudioInit;
  int32_t          status;

  // Clear vStream runtime information
  memset(&vstream_info, 0, sizeof(vstream_info));

  // Register event callback function
  vstream_info.fn_event_cb = event_cb;

  // Configure BSP audio driver parameters
  AudioInit.BitsPerSample = AUDIO_IN_BITS_PER_SAMPLE;
  AudioInit.SampleRate    = AUDIO_IN_SAMPLING_RATE;
#if (AUDIO_IN_MODE == 0)                // Mono mode
  AudioInit.Device        = AUDIO_IN_DEVICE_DIGITAL_MIC1;
  AudioInit.ChannelsNbr   = 1U;
#else                                   // Stereo mode
  AudioInit.Device        = AUDIO_IN_DEVICE_DIGITAL_MIC;
  AudioInit.ChannelsNbr   = 2U;
#endif
  AudioInit.Volume        = 100U;       // Not used

  // Initialize BSP audio driver
  status = BSP_AUDIO_IN_Init(0U, &AudioInit);
  if (status != BSP_ERROR_NONE) {
    return VSTREAM_ERROR;
  }

  // Set volume (on B-U585I-IOT02A this function is not supported)
  status = BSP_AUDIO_IN_SetVolume(0U, 100U);
  if ((status != BSP_ERROR_NONE) && (status != BSP_ERROR_FEATURE_NOT_SUPPORTED)) {
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
  int32_t status;

  // De-register event callback function
  vstream_info.fn_event_cb = NULL;

  // De-initialize BSP audio driver
  status = BSP_AUDIO_IN_DeInit(0U);
  if (status != BSP_ERROR_NONE) {
    return VSTREAM_ERROR;
  }

  // Clear vStream runtime information
  memset(&vstream_info, 0, sizeof(vstream_info));

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

  // Check parameters
  // Notes:
  //  - buf_size must be equal to 2 * block_size, to allow double-buffering with DMA
  //  - buf_size must less than than 65535 because of BSP audio driver limitation
  if ((buf == NULL) || (block_size == 0U) || (buf_size > UINT16_MAX) || (buf_size != (block_size * 2))) {
    return VSTREAM_ERROR_PARAMETER;
  }

  // If streaming is already active return error
  if (vstream_info.streaming_active != 0U) {
    return VSTREAM_ERROR;
  }

  // Register buffer information
  vstream_info.data_buf        = (uint8_t *)buf;
  vstream_info.data_block_size = block_size;

  // Initialize data block counters
  vstream_info.data_block_in_cnt = 0U;
  vstream_info.data_block_rd_cnt = 0U;

  return VSTREAM_OK;
}

/**
  \fn           int32_t Start (uint32_t mode)
  \brief        Start streaming.
  \param[in]    mode            streaming mode
  \return       VSTREAM_OK on success; otherwise, an appropriate error code
*/
static int32_t Start (uint32_t mode) {
  int32_t status;

  // Check parameter
  // Note: only continuous mode is supported
  if (mode != VSTREAM_MODE_CONTINUOUS) {
    return VSTREAM_ERROR_PARAMETER;
  }

  // If streaming is already active return Ok
  if (vstream_info.streaming_active != 0U) {
    return VSTREAM_OK;
  }

  // If data_buf is NULL return error
  if (vstream_info.data_buf == NULL) {
    return VSTREAM_ERROR;
  }

  // Set flag to ignore first BSP_AUDIO_IN_TransferComplete_CallBack callback 
  // because it happens immediately after recording is started and before actual
  // data was recorded and
  // start recording
  vstream_info.ignore_callback = 1U;
  status = BSP_AUDIO_IN_Record(0U, vstream_info.data_buf, vstream_info.data_block_size * 2U);
  if (status != BSP_ERROR_NONE) {
    return VSTREAM_ERROR;
  }

  // Set streaming active flag
  vstream_info.streaming_active = 1U;

  return VSTREAM_OK;
}

/**
  \fn           int32_t Stop (void)
  \brief        Stop streaming.
  \return       VSTREAM_OK on success; otherwise, an VSTREAM_ERROR error code
*/
static int32_t Stop (void) {
  int32_t status;

  // If streaming is not active return Ok
  if (vstream_info.streaming_active == 0U) {
    return VSTREAM_OK;
  }

  status = BSP_AUDIO_IN_Stop(0U);       // Stop recording
  if (status != BSP_ERROR_NONE) {
    return VSTREAM_ERROR;
  }

  // Reset data block counters (flush data)
  vstream_info.data_block_in_cnt = 0U;
  vstream_info.data_block_rd_cnt = 0U;

  // Clear streaming active flag
  vstream_info.streaming_active = 0U;

  return VSTREAM_OK;
}

/**
  \fn           void *GetBlock (void)
  \brief        Get pointer to Virtual Streaming data block.
  \return       pointer to data block, returns NULL if no block is available
*/
static void *GetBlock (void) {
  uint32_t data_addr;

  // If data_buf is NULL return NULL
  if (vstream_info.data_buf == NULL) {
    return NULL;
  }

  // If there is no data available return NULL
  if (vstream_info.data_block_in_cnt == vstream_info.data_block_rd_cnt) {
    return NULL;
  }

  // Calculate address of oldest unread data block
  data_addr = (uint32_t)vstream_info.data_buf;
  if ((vstream_info.data_block_rd_cnt & 1U) == 1U) {
    data_addr += vstream_info.data_block_size;
  }

  return ((void *)data_addr);
}

/**
  \fn           int32_t ReleaseBlock (void)
  \brief        Release Virtual Streaming data block.
  \return       VSTREAM_OK on success; otherwise, an VSTREAM_ERROR error code
*/
static int32_t ReleaseBlock (void) {

  // If there is no data available return error
  if (vstream_info.data_block_in_cnt == vstream_info.data_block_rd_cnt) {
    return VSTREAM_ERROR;
  }

  // Increment read data block counter
  vstream_info.data_block_rd_cnt += 1U;

  return VSTREAM_OK;
}

/**
  \fn           vStreamStatus_t GetStatus (void)
  \brief        Get Virtual Streaming status.
  \return       streaming status structure
*/
static vStreamStatus_t GetStatus (void) {
  vStreamStatus_t stat;

  // Initialize (clear) stat structure
  memset(&stat, 0, sizeof(stat));

  // Check streaming active status
  if (vstream_info.streaming_active != 0U) {
    stat.active = 1U;
  }

  // Handle data_overflow flag
  if (vstream_info.data_overflow != 0U) {
    vstream_info.data_overflow = 0U;
    stat.overflow = 1U;
  }

  // Underflow cannot happen on input stream
  // EOS cannot happen with audio

  return stat;
}

/**
  \fn           void NewBlockRecorded (void)
  \brief        Handle new block recorded (called from IRQ).
*/
static void NewBlockRecorded (void) {
  uint32_t events;

  // Increment recorded data block counter
  vstream_info.data_block_in_cnt += 1U;

  events = VSTREAM_EVENT_DATA;

  // Check if data overflow happened
  if ((vstream_info.data_block_in_cnt - vstream_info.data_block_rd_cnt) > 2U) {
    vstream_info.data_overflow = 1U;
    events |= VSTREAM_EVENT_OVERFLOW;
  }

  // If signal function was registered -> signal active events
  if ((vstream_info.fn_event_cb != NULL) && (events != 0U)) {
    vstream_info.fn_event_cb(events);
  }
}


// Global function definitions (HAL callbacks) ---------

/**
  \fn           void BSP_AUDIO_IN_HalfTransfer_CallBack (uint32_t Instance)
  \brief        Manage the BSP audio in half transfer complete event.
  \param[in]    Instance Audio in instance.
*/
void BSP_AUDIO_IN_HalfTransfer_CallBack (uint32_t Instance) {
  (void)Instance;

  NewBlockRecorded();                   // Handle new block recorded
}

/**
  \fn           void BSP_AUDIO_IN_TransferComplete_CallBack (uint32_t Instance)
  \brief        Manage the BSP audio in transfer complete event.
  \param[in]    Instance Audio in instance.
*/
void BSP_AUDIO_IN_TransferComplete_CallBack (uint32_t Instance) {
  (void)Instance;

  if (vstream_info.ignore_callback) {   // If flag is active to ignore first callback
    vstream_info.ignore_callback = 0U;  // Clear flag to not ignore callbacks anymore
    return;                             // and exit
  }

  NewBlockRecorded();                   // Handle new block recorded
}


// Global driver structure

vStreamDriver_t Driver_vStreamAudioIn = {
  Initialize,
  Uninitialize,
  SetBuf,
  Start,
  Stop,
  GetBlock,
  ReleaseBlock,
  GetStatus
};
