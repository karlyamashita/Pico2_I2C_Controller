# Pico 2 W (RP2350) I2C / SPI Controller

This is the Pico 2 W port of `Pico2_I2C_Controller`. The original project is separate.

## Build and transport

- CMake board: `pico2_w` (Pico SDK 2.x with Pico 2 W support).
- CMake project / UF2: `Pico2W_I2C_Controller` / `Pico2W_I2C_Controller.uf2`.
- Onboard LED uses the CYW43 wireless chip GPIO (`cyw43_arch_init`, `pico_cyw43_arch_none`).
- Commands and capture data use USB CDC; Wi-Fi networking is not enabled.
- Existing I2C, SPI, GPIO, PIO and DMA functionality is retained.

Build with the Raspberry Pi Pico VS Code extension or CMake and the ARM toolchain. Reconfigure or delete the previous CMake build cache when changing boards.

## SPI logic capture for SLA

The firmware streams SPI samples over USB CDC as text. Serial Flux can forward these lines to the separate SPI Logic Analyzer (SLA) desktop application. The firmware does **not** open a TCP connection to SLA itself.

### Commands

| Command | Behavior |
| --- | --- |
| `SPI NEW CAPTURE` | Reset/start a new SPI capture session, arm PIO sampling and enable SLA text streaming. |
| `SPI STOP` | Stop SPI logic capture; SPI master/slave commands remain usable. |
| `SPI STATUS` | Show SPI diagnostics, capture state, packet counts and overrun information. |
| `SPI MODE <0..3>` / `SPI MODE?` | Set/query SPI mode. |
| `SPI SPEED <hz>` / `SPI SPEED?` | Set/query SPI clock speed. |
| `SPI XFER ...` / `SPI CS PULSE XFER ...` | SPI transfers remain available whether logic capture is running or stopped. |

Use `HELP` for the complete firmware command list. Legacy `SPI SNIFF` commands are not part of this interface. The **I2C** `SNIFF START`, `SNIFF STOP`, `SNIFF STATUS`, and `SNIFF DECODE` commands are separate and remain available.

### Capture stream protocol

For each captured CS-low frame, the Pico emits:

```text
SLA_BEGIN ID=<capture_id> PACKET=<packet_id> START_US=<us> END_US=<us> MODE=<mode> WORDS=<count> SAMPLE_HZ=<hz> PARTIAL_BITS=<bits> TRUNCATED=<0|1>
SLA_RAW <word_offset> <8-digit-hex-word> [<8-digit-hex-word> ...]
SLA_END ID=<capture_id> PACKET=<packet_id> WORDS=<count>
```

- Each 32-bit raw word stores ten 3-bit GPIO samples, earliest in bits 29:27.
- Sample bits: bit 0 = GP20 (MOSI), bit 1 = GP21 (MISO), bit 2 = GP22 (SCLK). CS is GP26 and marks frame boundaries.
- `SAMPLE_HZ` is the reported system clock used for PIO sampling.
- `START_US` and `END_US` are microseconds since boot measured in GPIO interrupt handling; interrupt latency may affect precision.
- `MODE` comes from the SPI configuration.
- Each `SLA_RAW` line carries up to 16 packed words. `word_offset` is the offset within the frame.

### Current limitations

- Streaming is **opt-in**: start with `SPI NEW CAPTURE`. Opening SLA alone does not arm the Pico.
- The firmware has one pending frame buffer. Another CS frame may be missed while the prior frame is being sent over USB; inspect `OVERRUNS` in `SPI STATUS`.
- PIO stores complete 30-bit groups (ten samples per word); up to nine trailing samples may be omitted.
- ASCII USB streaming is a functional integration baseline, not a guaranteed lossless high-bandwidth capture transport.
- Validate high-rate capture, backpressure and timing accuracy on hardware.

## I2C

The I2C controller and I2C logic capture remain available. Use `I2C NEW CAPTURE`, `I2C STOP`, `I2C STATUS` for logic capture, and the separate `SNIFF` commands for I2C sniffer diagnostics/decoding. Use `HELP` for supported arguments.

## Verification

The README update does not change or test firmware. Build and verify USB CDC, LED, I2C, SPI transfers and SPI capture streaming on the Pico 2 W.
