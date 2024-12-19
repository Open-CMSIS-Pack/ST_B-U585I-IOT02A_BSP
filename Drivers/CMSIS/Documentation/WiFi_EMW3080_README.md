# CMSIS-Driver WiFi for the MXCHIP EMW3080 module

## Requirements

### Module

### Firmware: v2.3.4 rc 13

> Note: (for update procedure please consult
>       [WiFi firmware update for MXCHIP EMW3080B on STM32 boards](https://www.st.com/en/development-tools/x-wifi-emw3080b.html))

## STM32CubeMX Setup

The description below shows the **STM32CubeMX** settings for the **B-U585I-IOT02A** board.

Select or enter the values that are marked in **bold**.

### Pinout & Configuration tab

  1. Select functions of all of the interface pins :
     - Click on pin **PD1** in the chip diagram and select **SPI2_SCK**
     - Click on pin **PD3** in the chip diagram and select **SPI2_MISO**
     - Click on pin **PD4** in the chip diagram and select **SPI2_MOSI**
     - Click on pin **PB12** in the chip diagram and select **GPIO_Output**
     - Click on pin **PD14** in the chip diagram and select **GPIO_EXTI14**
     - Click on pin **PF15** in the chip diagram and select **GPIO_Output**
     - Click on pin **PG15** in the chip diagram and select **GPIO_EXTI15**

  2. Configure the **SPI** peripheral :
     - Under category **Connectivity** select **SPI2** and set:
       - Mode: **Full-Duplex Master**
       - Hardware NSS Signal: **Disable**
       - Hardware RDY Signal: **Disable**
     - Select **Parameter Settings** tab and set:
       - Basic Parameters
         - Frame Format: **Motorola**
         - Data Size: **8 Bits**
         - First Bit: **MSB First**
       - Clock Parameters
         - Prescaler (for Baud Rate): **8**
         - Baud Rate: 20.0 MBits/s
         - Clock Polarity (CPOL): **Low**
         - Clock Phase (CPHA): **1 Edge**
       - Autonomous Mode
         - State: **Disable**
       - CRC Parameters
         - CRC Calculation: **Disabled**
       - Advanced Parameters
         - NSSP Mode: **Disabled**
         - NSS Signal Type: **Software**
         - Fifo Threshold: **Fifo Threashold 01 Data**
         - Nss Polarity: **Nss Polarity Low**
         - Master Ss Idleness: **00 Cycle**
         - Master Inter Data Idleness: **00 Cycle**
         - Master Receiver Auto Susp: **Disable**
         - Master Keep Io State: **Master Keep Io State Disable**
         - IO Swap: **Disabled**
         - Ready Master Management: **Internal**
         - Ready Signal Polarity: **High**

  3. Configure all of the interface pins :
     - Under category **System Core** select **GPIO** and select **SPI** tab and set:
       Pin Name | Signal on Pin     | GPIO mode           | GPIO Pull-up/Pull-down      | Maximum out | User Label
       :--------|:------------------|:--------------------|:----------------------------|:------------|:----------
       PD1      | SPI2_SCK          | Alternate           | No pull-up and no pull-down | High        |.
       PD3      | SPI2_MISO         | Alternate           | No pull-up and no pull-down | High        |.
       PD4      | SPI2_MOSI         | Alternate           | No pull-up and no pull-down | High        |.

     - Under category **System Core** select **GPIO** and select **GPIO** tab and set:
       Pin Name | GPIO output level | GPIO mode           | GPIO Pull-up/Pull-down      | Maximum out | User Label
       :--------|:------------------|:--------------------|:----------------------------|:------------|:----------
       PB12     | High              | Output Push Pull    | No pull-up and no pull-down | High        | **MXCHIP_NSS**
       PD14     | n/a               | Ext Int Rising edge | No pull-up and no pull-down | n/a         | **MXCHIP_NOTIFY**
       PF15     | Low               | Output Push Pull    | No pull-up and no pull-down | Low         | **MXCHIP_RESET**
       PG15     | n/a               | Ext Int Rising edge | No pull-up and no pull-down | n/a         | **MXCHIP_FLOW**

  4. Configure the **NVIC** :
     - Under category **System Core** select **NVIC** and select **NVIC** tab and set:
       - Priority Group: **4 bits for pre-emption priority 0 bits for subpriority**
       - Force DMA channel Interrupts: **yes**

       NVIC Interrupt Table           | Enable | Preemption Priority | Sub Priority
       :------------------------------|:-------|:--------------------|:--------------
       EXTI Line14 interrupt          |  yes   | 8                   | 0
       EXTI Line15 interrupt          |  yes   | 8                   | 0
       SPI2 global interrupt          |  yes   | 8                   | 0

### Clock Configuration tab

  1. Configure the **SPI clock**
     - Setup **To SPI2 (MHz)** to **160 MHz** (same as HCLK clock)

### Project Manager tab

  1. Select **Advanced Settings** tab and:
     - in the **Register Callback** window change value for **SPI** to **ENABLE**

## Source Code Setup

 - Open the STM32CubeMX generated **main.c** file.
   Add the following include into the section between  
   `USER CODE BEGIN Includes` and `USER CODE END Includes`:

   ```C
   #include "GPIO_STM32.h"
   #include "WiFi_EMW3080.h"
   ```

 - Add code for handling External Interrupts by copy-pasting the following code snippet into the section
   between  
   `USER CODE BEGIN 0` and `USER CODE END 0`:

   ```C
   static void GPIO_SignalEvent (ARM_GPIO_Pin_t pin, uint32_t event) {
    
     switch (pin) {
       case GPIO_PIN_ID_PORTD(14):                         // MXCHIP_NOTIFY pin (PD14)
         if ((event & ARM_GPIO_EVENT_RISING_EDGE) != 0U) { // If rising edge was detected
           WiFi_EMW3080_Pin_NOTIFY_Rising_Edge();
         }
         break;
       case GPIO_PIN_ID_PORTG(15):                         // MXCHIP_FLOW pin (PG15)
         if ((event & ARM_GPIO_EVENT_RISING_EDGE) != 0U) { // If rising edge was detected
           WiFi_EMW3080_Pin_FLOW_Rising_Edge();
         }
         break;
     }
   }
   ```

 - Add code for initializing GPIOs for **Flow** and **Notify** by copy-pasting the following code snippet into the section
   in function **MX_GPIO_Init**, between  
   `USER CODE BEGIN MX_GPIO_Init_2` and `USER CODE END MX_GPIO_Init_2`:

   ```C
   // Configure MXCHIP_NOTIFY pin (PD14) to generate event on the rising edge
   Driver_GPIO0.Setup          (GPIO_PIN_ID_PORTD(14), GPIO_SignalEvent);
   Driver_GPIO0.SetDirection   (GPIO_PIN_ID_PORTD(14), ARM_GPIO_INPUT);
   Driver_GPIO0.SetEventTrigger(GPIO_PIN_ID_PORTD(14), ARM_GPIO_TRIGGER_RISING_EDGE);

   // Configure MXCHIP_FLOW pin (PG15) to generate event on the rising edge
   Driver_GPIO0.Setup          (GPIO_PIN_ID_PORTG(15), GPIO_SignalEvent);
   Driver_GPIO0.SetDirection   (GPIO_PIN_ID_PORTG(15), ARM_GPIO_INPUT);
   Driver_GPIO0.SetEventTrigger(GPIO_PIN_ID_PORTG(15), ARM_GPIO_TRIGGER_RISING_EDGE);
   ```

## Driver Configuration

This driver is built on top of the **MX_WIFI Component Driver**, so the configuration is also done separately in two configuration files.  
Respectively **WiFi_EMW3080_Config.h** file is used for the CMSIS-Driver configuration and **mx_wifi_conf.h** file is used for the
MX_WIFI Component Driver configuration.

### CMSIS-Driver Configuration Settings: WiFi_EMW3080_Config.h file

 - **WIFI_EMW3080_DRV_NUM** specifies the exported driver number  
   (default value is **0**, default exported driver structure is **Driver_WiFi0**).
 - **WIFI_EMW3080_SCAN_BUF_SIZE** specifies the maximum size of the buffer used for Scan function  
   (default value is **2048** bytes).
 - **WIFI_EMW3080_SOCKETS_NUM** specifies the maximum number of sockets supported by the driver  
   (default value is **4**).
 - **WIFI_EMW3080_SOCKETS_RX_BUF_SIZE** specifies the maximum size of the Socket Receive buffer  
   (default value is **1500** bytes).
 - **WIFI_EMW3080_SOCKETS_TIMEOUT** specifies the timeout for locking of the socket structure to prevent concurrent access  
   (default value is **10000** ms).
 - **WIFI_EMW3080_SOCKETS_RCVTIMEO** specifies the Socket Receive timeout  
   (default value is **20000** ms).
 - **WIFI_EMW3080_SOCKETS_RCV_RETRIES** specifies the number of receive retries  
   (default value is **10**).
 - **WIFI_EMW3080_SOCKETS_INTERVAL** specifies the polling interval for emulating blocking sockets  
   (default value is **250** ms).

### MX_WIFI Component Driver Configuration Settings: mx_wifi_conf.h file

 - **MX_WIFI_USE_SPI** specifies SPI Interface usage. Since this Firmware only supports SPI Interface this setting must be set to **1**.
 - **MXCHIP_SPI** specifies the SPI peripheral handle. On this board SPI2 is used, so this setting should be set to **hspi2**.  
   If different SPI peripheral is used this setting should be changed accordingly.
 - **DMA_ON_USE** specifies DMA usage for the SPI transfers. If DMA is used set this setting to 1, otherwise set it to 0.  
   By **default** this setting is set to **0** to enable SPI to work on devices with TrustZone enabled and SAU disabled.
 - **MX_WIFI_USE_CMSIS_OS** specifies usage of the CMSIS RTOS2. This setting must be set to **1**.
 - **MX_WIFI_NETWORK_BYPASS_MODE** enables or disables bypass mode. Set it to 1 to enable bypass mode, otherwise set it to 0.  
   By **default** this setting is set to **0** thus bypass mode is disabled.
 - **MX_WIFI_TX_BUFFER_NO_COPY** enables or disables transmit buffer copying. Set it to 1 not to use transmit buffer copying, otherwise set it to 0.  
   By **default** this setting is set to **1** thus transmit buffer copying is disabled.
 - **MX_WIFI_API_DEBUG** specifies if the Host driver API functions output debugging messages.  
   Define this macro to enable debugging messages.
 - **MX_WIFI_IPC_DEBUG** specifies if the Host driver IPC protocol functions output debugging messages.  
   Define this macro to enable debugging messages.
 - **MX_WIFI_HCI_DEBUG** specifies if the Host driver HCI protocol functions output debugging messages.  
   Define this macro to enable debugging messages.
 - **MX_WIFI_SLIP_DEBUG** specifies if the SLIP protocol functions output debugging messages.  
   Define this macro to enable debugging messages.
 - **MX_WIFI_IO_DEBUG** specifies if the Host driver low-level I/O (SPI/UART) functions output debugging messages.  
   Define this macro to enable debugging messages.
 - for other settings please consult source file implementation on their usage.
