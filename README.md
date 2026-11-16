# Pico2_I2C_Controller

**Raspberry Pi Pico 2–based I²C controller and logic-analyzer interface for Serial Flux (SF) and the I2C Logic Analyzer GUI.**

## Overview

Pico2_I2C_Controller uses the **Raspberry Pi Pico 2** as a USB-connected hardware interface for working with I²C devices. The project is intended to support two PC-side workflows:

- **Serial Flux (SF):** A general-purpose communication and debugging application for interacting with connected hardware, sending commands, and inspecting responses.
- **I2C Logic Analyzer GUI:** A dedicated interface for examining I²C bus activity and making transactions easier to understand during development and troubleshooting.

The Pico 2 provides the physical connection to the target I²C bus, while the PC applications provide the user interface. Exact firmware commands, capture capabilities, and supported operating modes depend on the firmware implementation.

## Raspberry Pi Pico 2 hardware

The Raspberry Pi Pico 2 is a compact microcontroller development board built around the **Raspberry Pi RP2350**.

| Feature | Pico 2 specification |
| --- | --- |
| Microcontroller | Raspberry Pi RP2350 |
| CPU | Two Arm Cortex-M33 cores **or** two Hazard3 RISC-V cores (selectable architecture) |
| Maximum nominal CPU clock | 150 MHz |
| On-chip SRAM | 520 KB |
| On-board flash | 4 MB |
| USB | USB 1.1 controller with device support; Micro-USB connector |
| Logic voltage | 3.3 V GPIO |
| GPIO | 26 user-accessible multifunction GPIO pins |
| Hardware I²C | Two I²C controllers |
| Programmable I/O | 12 PIO state machines across three PIO blocks |
| Form factor | 40-pin, breadboard-friendly board |

The RP2350's hardware I²C controllers and programmable I/O (PIO) make the Pico 2 useful for embedded communication tools. The specific pins and peripherals used by this project will be documented in the pinout section.

> **Electrical caution:** Pico 2 GPIO operates at 3.3 V and is **not 5 V tolerant**. Check the target bus voltage and use appropriate level shifting when necessary. I²C SDA and SCL require suitable pull-up resistors; avoid conflicting or excessive pull-ups on an existing bus.

## System architecture

```text
                  Windows PC
        +-----------------------------+
        | Serial Flux (SF)            |
        | I2C Logic Analyzer GUI      |
        +--------------+--------------+
                       |
                 USB connection
                       |
        +--------------v--------------+
        | Raspberry Pi Pico 2         |
        | RP2350 / project firmware   |
        +--------------+--------------+
                       |
                  I2C interface
                  SDA / SCL / GND
                       |
        +--------------v--------------+
        | Target board / I2C devices  |
        +-----------------------------+
```

## Using the controller with Serial Flux

**Serial Flux** is the PC-side environment for communicating with and debugging embedded devices. With the appropriate Pico2_I2C_Controller firmware and SF configuration, the Pico 2 can serve as the hardware bridge between SF and an I²C target.

Typical development tasks include sending supported commands, viewing device responses, and recording communication for troubleshooting. The exact command syntax, USB interface, and supported I²C operations will be documented alongside the firmware.

## Using the controller with the I2C Logic Analyzer GUI

The **I2C Logic Analyzer GUI** is intended for inspecting I²C activity through the Pico 2 interface. This workflow is useful when investigating device addressing, register transactions, acknowledgments, and communication problems.

The GUI and firmware determine which capture and decoding features are available. In particular, passive bus monitoring, supported sample rates, timestamps, and trigger behavior should be verified against the implemented firmware rather than assumed from the Pico 2 hardware alone.

## Applications

- Embedded board bring-up and I²C device evaluation
- Firmware development and debugging
- Troubleshooting I²C communication and wiring
- Examining register transactions and device responses
- Education and experimentation with I²C peripherals

## Hardware connections and pinout

**Pinout documentation is planned for the next update.** This section will identify the actual Pico 2 GPIO assignments, I²C connections, power and ground pins, and any additional signals used by the firmware.

| Signal | Pico 2 pin / GPIO | Notes |
| --- | --- | --- |
| I²C SDA | TBD | Data line |
| I²C SCL | TBD | Clock line |
| GND | TBD | Common ground with target |
| Additional signals | TBD | To be confirmed from firmware and hardware design |

**Do not wire a target based on this placeholder table.** Confirm the final pin assignments and target voltage before connecting hardware.

## KTYLabs software

To use the **Pico2_I2C_Controller** with a Windows PC, visit **[KTYLabs](https://ktylabs.com/)** for information about **Serial Flux (SF)** and the **I2C Logic Analyzer GUI**, including available software downloads and setup resources. The Pico 2 requires compatible controller firmware and PC-side software for the intended workflow.

## Software and firmware

This repository is for the Pico2_I2C_Controller project. Depending on the release, it may contain Pico 2 firmware, build files, documentation, and related utilities.

For PC-side operation, use the compatible versions of:

- **[Serial Flux (SF)](https://ktylabs.com/)**
- **I2C Logic Analyzer GUI** (see [KTYLabs](https://ktylabs.com/))

Installation, firmware flashing, USB connection settings, and application-specific setup instructions will be added once the corresponding project details are confirmed.

## Project status

Documentation in progress. The next update will define and illustrate the **Raspberry Pi Pico 2 pinout** used by Pico2_I2C_Controller.

## License

See the repository's `LICENSE` file, if provided, for the applicable licensing terms.
