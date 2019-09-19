## STM32F7_Control_Board

## Purpose:
In Sensor Systems Lab, there is a PR2 robot that has the internal cable connecting proximity sensors to the computer for trasmitting data. However, we want an application that can employ this feature on other robots. So for this application, it does not require the robot to have the internal cable. It can transfer the data from registers in proximity sensors to the Wi-Fi hub and send the data back to a computer wirelessly.

---

## Requirement:

System: Windows 7 or higher

Hardware: STM32F411E Discovery board

Software: STM32CubeMX, Keil uVision IDE with MDK-ARM toolchain, Putty

---

## Hardware construction:

The hardware has three parts: I2C to connectors, microcontroller, and the USB
I2C: the I2C circuits are for connecting the proximity sensors. (The connectors are FFC & FPC connectors)
Microcontroller: STM32F723 microcontroller, which has embedded PHY chip for the USB 2.0
USB: the USB circuits consist of suggested ESD protection (Check details from datasheet of ECMF02-2AMX6)
The rest of the components are mostly capacitors and resistors. Resistors are for LEDs and as pull-up resistors. Capacitors are as simple low pass filters for filtering high frequency noise signals for pairs of VDD and VSS on microcontroller.

---

## Configuration:

**Pre-connection:**
Disconnect the 2 jumpers from CN3 on STM32F411 Discovery board for programming/debugging external STM32 application and connect CN2 debug connector to the connector on the control board (Check details on the datasheet from STM32F411 discovery board)

**Use the STM32CubeMX to configure the following setting of the microcontroller:**

**Peripherals:**

I2C: I2C1

RCC: High Speed Clock (HSE) -> Crystal/Ceramic Resonator

SYS: Debug -> Serial Wire

USB_STG_HS: Internal Phy -> Device_Only, Active_VBUS checkbox

**MiddleWares:**

USB_DEVICE: Classs for HS IP -> Communication Device Class (Virtual Port Com)

**On_board:**

Configure PE2 as GPIO_Output

---

## Code:

The main method includes the initiation of the gpio expander device for controlling each VL6180 device in the array, turning all VL6180 devices off, turning the VL6180 devices on one by one and changing the its slave address, and finally testing if the application can send out string over the USB to the computer and shown on a terminal through virtual communication port.

---

## Testing:

Note that there are two tests for now: testing if the application can send only stirng to the terminal, and testing if the terminal can receive different string according to user input as keys.

For the first test, define the macro SEND_ONLY in main.c. For the second test, define the macro CALLBACK in main.c, usbd_cdc_if.c and usbd_cdc_if.h. In line 333 of the usbd_cdc_if.c and in line 140 of the usbd_cdc_if.h, the declaration of the weak object function for the call back function is for making the call back in the USB receive cdc function, which is in line 302 of the usbd_cdc_if.c. And the actual call back action is defined in main.c, which is in line 207.

*Detailed setting can be refered in [video](https://www.youtube.com/watch?v=7oED-m34EKk)

---