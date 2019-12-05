# TwoStepperControl

## Introduction
TSC (TwoStepperControl) is a software project by Wolfgang Birkfellner and Steffen Elste to realize an open integrated control system for large and heavy astronomical telescopes. The basic idea is to use a Raspberry Pi to control a telescope and to provide basic autoguiding functionality without additional external computers. As Raspian is not an RTOS, it cannot control stepper drivers directly like a microcontroller; therefore, extra hardware is necessary which receives basic parameters (number of steps, velocity, acceleration) from the software. We are currently using AMIS 30543 bipolar driver boards with 3A maximum current per coil and a maximum voltage of 30 V.

## Features
TSC basically provides the following functionality
- Telescope tracking and GOTO using internal catalogs for german equatorial and fork mounts in the higher range - up to NEMA 34
steppers are used
- Telescope control via LX200 - both USB/Ethernet and WLAN are supported. In the absence of a WLAN-access point, TSC opens a WLAN network of its own to connect to tablets, cell phones and other computers. Namely, control via SkySafari aon iOS and Android is provided.
- Remote control via VNC (Virtual Network Clients).
- DSLR control including dithering.
- Support for two additional focuser motors.
- Free customization of drives and gears with microstepping ratios between 1/128 and 1/4.
- Image acquisition and autoguiding from guiding cameras via INDI.
- ST4 is also supported

Currently unde re-development:
- Autoguiding integrated on the controller.

## Documentation
Because we - not intentionally - messed this up a bit in the past we're setting up a dedicated repository/site for documentation:
[TwoStepperControl Documentation](https://tscatm.wordpress.com/)
Please also take a closer look at the manual, which covers all aspects of usage and assembly. However, with the switch to the AMIS drivers, things are in motion

The [Wiki](https://github.com/selste/TwoStepperControl/wiki) in this repository will henceforth only deal with information regarding compilation (and the setup required) and related stuff.
