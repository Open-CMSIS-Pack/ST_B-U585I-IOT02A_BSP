Board: STMicroelectronics [B-U585I-IOT02A](https://www.st.com/en/evaluation-tools/b-u585i-iot02a.html)
------------------------------------------

Device: **STM32U585AII6QU**
System Core Clock: **160 MHz**

This setup is configured using **STM32CubeMX**, an interactive tool provided by STMicroelectronics for device configuration.
Refer to ["Configure STM32 Devices with CubeMX"](https://github.com/Open-CMSIS-Pack/cmsis-toolbox/blob/main/docs/CubeMX.md) for additional information.

### System Configuration

**TrustZone** disabled (option bit TZEN=0)

| System resource       | Setting
|:----------------------|:----------------------------------------------
| Heap                  | 1 kB (configured in the STM32CubeMX)
| Stack (MSP)           | 1 kB (configured in the STM32CubeMX)

### STDIO mapping

**STDIO** is routed to Virtual COM port on the ST-Link (using **USART1** peripheral)

### CMSIS-Driver mapping

| CMSIS-Driver          | Peripheral
|:----------------------|:----------------------------------------------
| Driver_GPIO0          | GPIO0
| Driver_I2C1           | I2C1
| Driver_SPI1           | SPI1
| Driver_USART2         | USART2
| Driver_USART3         | USART3
| Driver_USBD0          | USB_OTG_FS

### CMSIS-Driver Virtual I/O mapping

| CMSIS-Driver VIO      | Physical resource
|:----------------------|:----------------------------------------------
| vioBUTTON0            | Button USER (PC13)
| vioLED0               | LED6 RED    (PH6)
| vioLED1               | LED7 GREEN  (PH7)

## Arduino UNO mapping

| Arduino resource      | Driver
|:----------------------|:----------------------------------------------
| UART (D0,D1)          | USART3 Driver (ARDUINO_UNO_UART)
| SPI  (D11,D12,D13)    | SPI1   Driver (ARDUINO_UNO_SPI)
| I2C  (D20,D21)        | I2C1   Driver (ARDUINO_UNO_I2C)
| Digital I/O: D2       | GPIO0  Driver (ARDUINO_UNO_D2)
| Digital I/O: D3       | GPIO0  Driver (ARDUINO_UNO_D3)
| Digital I/O: D4       | GPIO0  Driver (ARDUINO_UNO_D4)
| Digital I/O: D5       | GPIO0  Driver (ARDUINO_UNO_D5)
| Digital I/O: D6       | GPIO0  Driver (ARDUINO_UNO_D6)
| Digital I/O: D7       | GPIO0  Driver (ARDUINO_UNO_D7)
| Digital I/O: D8       | GPIO0  Driver (ARDUINO_UNO_D8)
| Digital I/O: D9       | GPIO0  Driver (ARDUINO_UNO_D9)
| Digital I/O: D10      | GPIO0  Driver (ARDUINO_UNO_D10)
| Digital I/O: D14      | GPIO0  Driver (ARDUINO_UNO_D14)
| Digital I/O: D15      | GPIO0  Driver (ARDUINO_UNO_D15)
| Digital I/O: D16      | GPIO0  Driver (ARDUINO_UNO_D16)
| Digital I/O: D17      | GPIO0  Driver (ARDUINO_UNO_D17)
| Digital I/O: D18      | GPIO0  Driver (ARDUINO_UNO_D18)
| Digital I/O: D19      | GPIO0  Driver (ARDUINO_UNO_D19)

Reference to [Arduino UNO connector description](https://github.com/Open-CMSIS-Pack/cmsis-toolbox/blob/main/docs/ReferenceApplications.md#arduino-shield).

## Board configuration

**Board setup**
  - Insure that the **5V_USB_STLK** and **JP3** jumpers are bridged and the remaining jumpers are not bridged
  - Check that the **BOOT0** DIP switch is in the **0** / right position (closest to the ST-LINK STLK CN8 USB connector)
  - Connect a **USB micro-B cable** between the **STLK** connector and your **Personal Computer**
