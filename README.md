# TwoStepperControl

## Introduction
TSC (TwoStepperControl) is a software project by Wolfgang Birkfellner and Steffen Elste to realize an open integrated control system for large and heavy astronomical telescopes. The basic idea is to use a Raspberry Pi to control a telescope and to provide basic autoguiding functionality without additional external computers. As Raspian is not an RTOS, it cannot control stepper drivers directly like a microcontroller; therefore, extra hardware is necessary which receives basic parameters (number of steps, velocity, acceleration) from the software. Currently, we are using a Phidgets 1067 bipolar driver board with 4A maximum current per coil and a maximum voltage of 30 V. The steppers are monitored by the Pi in separate concurrent threads so that software functionality is maintained during stepper operation. Details are given in the Wiki.

## Features
TSC currently provides the following functionality
- Telescope tracking and GOTO using internal catalogs
- Telescope control via LX200 - both USB/Ethernet and WLAN are supported. In the absence of a WLAN-access point, TSC opens a WLAN network of its own to connect to tablets, cell phones and other computers.
- Remote control via VNC (Virtual Network Clients).
- Support for a custom wireless Bluetooth handbox.
- Support for ST4 guiding.
- Image acquisition from guiding cameras via INDI.
- Autoguiding integrated on the controller.
- DSLR control including dithering.
- Support for two additional focuser motors.
- Free customization of drives and gears.
- Supports both german equatorials and fork mounts.

## Documentation
Because we - not intentionally - messed this up a bit in the past we're setting up a dedicated repository/site for documentation:
[TwoStepperControl Documentation](https://selste.github.io/tsc-site/)

The [Wiki](https://github.com/selste/TwoStepperControl/wiki) in this repository will henceforth only deal with information regarding compilation (and the setup required) and related stuff.
