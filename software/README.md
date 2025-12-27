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

Only GNSS modules #1..#3 are on hardware USART pin pairs; the rest need a software UART (or other method) if all
8 modules must be received concurrently.

## Build / Upload

- Install PlatformIO CLI (or use the VS Code PlatformIO extension).
- From `software/`:
  - Build: `pio run`
  - Upload (ST-Link): `pio run -t upload`

`FreeRTOS-Kernel` is pulled via `lib_deps` (requires network access on first build).
