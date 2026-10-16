# SLA SPI capture stream (experimental)

Commands: `SPI NEW CAPTURE`, `SPI STATUS`, `SPI STOP`. Existing `SPI SNIFF` commands remain available.

The Pico emits `SLA_BEGIN`, followed by `SLA_RAW` lines and `SLA_END` for each CS-low frame. `SLA_RAW <word_offset> <8-digit hex words...>` contains ten 3-bit GPIO samples per word (earliest in bits 29:27); bit0=GP20 MOSI, bit1=GP21 MISO, bit2=GP22 SCLK. GP26 is CS. Sampling is PIO system-clock rate. Mode is inherited from `SPI MODE`. Timestamps in microseconds since boot are captured in the GPIO interrupt and may have interrupt latency.

**Important:** The existing single-frame buffer is retained. During USB transmission, another CS frame can be missed. Check `OVERRUNS`. The existing PIO samples only full 30-bit words; the final 1-9 samples may be lost. High-rate capture, backpressure, and exact timestamps need hardware validation. This is a firmware-side stream protocol only; the SLA desktop app must be updated to parse it.

The `SLA_RAW` text stream is intended as a functional integration baseline, not a lossless continuous high-bandwidth transport. Preserve your previous firmware build as the baseline.


R3 command interface:
- SPI NEW CAPTURE: reset counters and begin streaming captured SPI frames.
- SPI STOP: stop SPI capture, leave SPI master/slave available.
- SPI STATUS: returns existing SPI hardware diagnostics plus SPI_CAPTURE state.
- SPI XFER and SPI CS PULSE XFER: work whether capture is running or not.
- Legacy SPI SNIFF subcommands are removed. I2C SNIFF commands are unchanged.

IMPORTANT: R3 is firmware-side only. The current SLA desktop application does not yet parse SLA_BEGIN/SLA_RAW/SLA_END; automatic GUI updates require that desktop integration. The sniffer uses one pending frame buffer and may drop frames under sustained traffic. Capture is explicitly armed, not automatically enabled by opening the SLA window.
