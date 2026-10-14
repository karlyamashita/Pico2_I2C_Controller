# Pico 2 (RP2350) I2C Controller — separate project

Cloned from the uploaded `Pico_I2C_Controller_Source(4).zip`, with its own CMake target and RP2350 board selection. Original RP2040 project is unchanged.

## Changes
- CMake `PICO_BOARD=pico2` (standard **Pico 2**, not Pico 2 W).
- Project/output target `Pico2_I2C_Controller`.
- Replace CYW43 wireless LED support with GPIO25 onboard LED.
- Disable the original RP2040 experimental 200 MHz overclock; use SDK default RP2350 clock.
- Update firmware banner and version string.
- Preserve I2C/SPI pin assignments and command handlers from the uploaded source.

## Build
Open this folder with the Raspberry Pi Pico VS Code extension, configure the Pico SDK and select Pico 2; build. Alternatively configure with CMake using a recent Pico SDK 2.x and the ARM toolchain. The resulting UF2 is named `Pico2_I2C_Controller.uf2`.

## Important
This is a **hardware-port starting project**, not an implemented R25 live-streaming release. The original capture command logic is preserved. Firmware has not been compiled or tested on a physical Pico 2. Check RP2350 PIO/DMA behavior, bus pins, and USB serial output before using it on live hardware.
