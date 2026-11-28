# Software (PlatformIO)

This folder contains a minimal PlatformIO project targeting **STM32F103C8** using **STM32Cube (HAL)** + **FreeRTOS**.

## GNSS UART Pin Map

The hardware connects 8× ATGM336H modules using nets `TX_1..TX_8` / `RX_1..RX_8`:

- GNSS #1: TX=`PB10` (net `TX_1`), RX=`PB11` (net `RX_1`) → `USART3`
- GNSS #2: TX=`PA2` (net `TX_2`), RX=`PA3` (net `RX_2`) → `USART2`
- GNSS #3: TX=`PB6` (net `TX_3`), RX=`PB7` (net `RX_3`) → `USART1` (remapped)
- GNSS #4: TX=`PB8` (net `TX_4`), RX=`PB9` (net `RX_4`)
- GNSS #5: TX=`PB12` (net `TX_5`), RX=`PB13` (net `RX_5`)
- GNSS #6: TX=`PB14` (net `TX_6`), RX=`PB15` (net `RX_6`)
- GNSS #7: TX=`PA4` (net `TX_7`), RX=`PA5` (net `RX_7`)
- GNSS #8: TX=`PA6` (net `TX_8`), RX=`PA7` (net `RX_8`)

GNSS modules #1..#3 are on hardware USART pin pairs. Modules #4..#8 use a software UART: the `TIM3` update
tick runs at 8× the baud rate (9600 baud), samples their RX pins and clocks out their TX pins.

Commands are sent with `Gnss_SendCommand(module, "PCAS10,0")` (checksum and CRLF added), or
`Gnss_SendCommandAll()` for every module.

## Module Reset and PPS

- `MOD_RST` (`PA0`, open-drain, 10k pull-up R33): shared nRESET of the 8 modules. `Gnss_ResetModules()`
  pulls it low for 10 ms.
- `PPS` (`PA1`): 1PPS of GNSS #6 (U12), captured on the rising edge by `TIM2_CH2` at 1 MHz
  (`include/pps.h`). The RTT console prints the edge count and the measured local clock error.

## SPI Fused Output (J11)

Hardware SPI connector `J11` (JST-SH 1x06):

- Pin 1: `SS` (PB1, active-low)
- Pin 2: `MOSI` (PB5)
- Pin 3: `MISO` (PB4)
- Pin 4: `SCK` (PB3)
- Pin 5: `GND`
- Pin 6: `+3.3V`

Firmware exposes the fused GNSS solution as an `SPI1` slave (remapped to PB3/PB4/PB5). Each `SS` assertion starts
a streamed `SPI_FUSION_PACKET_SIZE`-byte packet on `MISO` (see `include/spi_fusion.h`).

Packet (little-endian, `SPI_FUSION_PACKET_SIZE == 32`):

- `u32 magic` = `0x31464745` (`EGF1`)
- `u32 tick_ms`
- `i32 lat_e7`, `i32 lon_e7`, `i32 alt_cm`
- `u16 avg_hdop_centi`, `u16 max_residual_cm`
- `u8 status`, `u8 used_modules`, `u8 rejected_modules`, `u8 has_fix`
- `u8 commands_accepted`, `u8 reserved`
- `u16 crc16_ccitt` (over first 30 bytes)

Commands from the host arrive on `MOSI` in the first 8 bytes of a read (`u32 'EGC1'`, `u8 command`,
`u8 argument`, `u16 crc16`); `0x01` resets the GNSS modules. See `include/spi_fusion.h`.

## Build / Upload

- Install PlatformIO CLI (or use the VS Code PlatformIO extension).
- From `software/`:
  - Build: `pio run`
  - Upload (J-Link, SWD): `pio run -t upload`
  - Debug console (SEGGER RTT): `python3 tools/rtt_read.py`

`FreeRTOS-Kernel` is vendored under `lib/FreeRTOS-Kernel` (Cortex-M3 port), so the build needs no extra download.
