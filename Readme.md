# OpenPPG Hub

Circuit that handles interfacing all other electronic devices.

## Features
- Powerful Cortex®-M0 STM32F0 architecture
- Barometric pressure sensor (for altitude estimation)
- 9-DOF IMU with sensors for Accel, Mag, Gyro, Temperature
- Communication with the ESCs – Includes handling the ESC telemetry to get data like current measurements, RPM and more.
- RS485 connections to controller and BMS
- Black box data logging - microSD card logs data from each motor during your flights to help graph out your flying as well as help us debug potential issues. Data is saved as a simple CSV for reading in Excel etc.


## Setup

The most affordable way to flash code onto your OpenPPG hub is to use a [STM Nucleo board](https://www.digikey.com/product-detail/en/NUCLEO-F302R8/497-14594-ND/) as the programmer.
For setup instructions see https://jeelabs.org/book/1547a/

Once drivers are installed for your OS for the Nucleo board
download and install [TrueSTUDIO](https://atollic.com/truestudio/)

Open the OpenPPG hub project code from within TrueSTUDIO and begin making your changes

## Debugging & Flashing

When ready to flash simply click the debug icon and the code will be loaded onto the OpenPPG hub.

Note: you may need to choose the correct debugger (ST-LINK) in the debug configuration. J-Link debugger can also be used and is a great way to inspect code at user-defined breakpoints

Breakpoints are a great way to stop the code execution and inspect variables etc. These are supported natively when flashing with the debug command in TrueStudio.

New code can also be flashed by placing the complied firmware on the microSD card and rebooting the hub. When the firmware is successfully flashed this way, the file will be deleted from the card. More details can be found at the bootloader link below.

### Bootloader
The OpenPPG hub requires a bootloader to be flashed to run this code. Each shipped hub already contains a flashed bootloader and shouldn't need to be flashed again.

Bootloader information can be found at https://github.com/openppg/eppg-hub-bootloader


## Contributing

Bug reports and pull requests for features, fixes, etc. are welcome on GitHub
