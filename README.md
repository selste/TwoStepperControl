# TwoStepperControl

## Introduction
TSC (TwoStepperControl) is a software project by Wolfgang Birkfellner and Steffen Elste to realize an open integrated control system for large and heavy astronomical telescopes. The basic idea is to use a Raspberry Pi to control a telescope and to provide basic autoguiding functionality without additional external computers. As Raspian is not an RTOS, it cannot control stepper drivers directly like a microcontroller; therefore, extra hardware is necessary which receives basic parameters (number of steps, velocity, acceleration) from the software. We were using Phidgets 1067 bipolar driver boards with 4A maximum current per coil and a maximum voltage of 30 V, but urrently we are switching to a standard NXT/STP interface with AMIS drivers. The steppers are monitored by the Pi in separate concurrent threads so that software functionality is maintained during stepper operation. Details are given in the Wiki.

## Features
TSC basically provides the following functionality
- Telescope tracking and GOTO using internal catalogs
- Telescope control via LX200 - both USB/Ethernet and WLAN are supported. In the absence of a WLAN-access point, TSC opens a WLAN network of its own to connect to tablets, cell phones and other computers.
- Remote control via VNC (Virtual Network Clients).
- DSLR control including dithering.
- Support for two additional focuser motors.
- Free customization of drives and gears with microstepping ratios between 1/128 and 1/4.
- Image acquisition from guiding cameras via INDI.

Currently unde re-development:
- Support for ST4 guiding.
- Autoguiding integrated on the controller.
- Supports both german equatorials and fork mounts.

## Documentation
Because we - not intentionally - messed this up a bit in the past we're setting up a dedicated repository/site for documentation:
[TwoStepperControl Documentation](https://tscatm.wordpress.com/)
Please also take a closer look at the manual, which covers all aspects of usage and assembly. However, with the switch to the AMIS drivers, things are in motion

The [Wiki](https://github.com/selste/TwoStepperControl/wiki) in this repository will henceforth only deal with information regarding compilation (and the setup required) and related stuff.
