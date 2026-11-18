# Porting the X6200 base-band firmware to the X6100

This document is the working plan for adapting Georgy Dyuldin's X6200 base-band
firmware (upstream `gdyuldin/x6200_bb_fw@014ee2d`, GPLv3) to the Xiegu X6100.

## Guiding principle: port by layer, not by `#ifdef`

The X6100 and X6200 differ on three axes that are each larger than "different
pins", so a single tree toggled with `#ifdef X6100 / X6200` would be mostly
divergent lines. Instead we keep upstream's layering and share along the
`App` / `Platform` seam:

1. **MCU family** — X6200 is an STM32H725 (Cortex-M7); the X6100 base board is an
   STM32F427 (Cortex-M4). Different HAL, DMA/DMAMUX, clock tree, cache/MPU, FPU.
   This is a HAL port, not a conditional.
2. **No hardware CORDIC** — upstream uses the H7 CORDIC peripheral for the FM
   discriminator / NBFM / boot. The F427 has no CORDIC; those paths move to a
   software CORDIC or CMSIS-DSP math.
3. **Signal-path architecture** — on the X6200 the FPGA does RF down/up-conversion
   **and** decimation and the MCU reads baseband I/Q over a bit-banged parallel
   bus (`Devices/drv_fpga.c`). The X6100 has no decimating FPGA: the F427 takes
   I/Q off a CS4272-class codec and does **all** the decimation/DSP itself. Its
   transport layer therefore has no direct X6100 counterpart.

## Layer buckets

| Bucket | Modules | Action |
|---|---|---|
| **Reuse ~as-is** (portable C) | `App/SDR/*` demodulators, `App/app_alc`, `app_protect`, `app_atu`, `app_pa_temp_comp`, `app_telemetry`, `app_nvram`, `app_scheduler`, `app_event`, `app_radio_state`, `app_cal*`; `tools/gen_*.py` filter generators | Keep; de-CORDIC the FM paths |
| **F427 shim** | `Platform/bsp_*`, `Core/` HAL glue, `Devices/drv_codec` (AIC3204 → CS4272), `drv_clockgen`, `drv_pca9535`, `drv_gauge` / `drv_charger` / `drv_pmic` | Rewrite against F4 HAL + X6100 parts |
| **Rewrite — architectural** | `Devices/drv_fpga` parallel-bus transport, `Platform/bsp_cordic`, host protocol | Replace: FPGA-fed I/Q ring → codec-I/Q + on-MCU CMSIS-DSP decimation; software CORDIC; X6100 host map |

## Not imported from upstream (re-vendored for the F427)

The verbatim import omitted the third-party vendor trees (they are ST/ARM
code, not upstream's GPLv3 IP, and are H7-specific):

- `Drivers/STM32H7xx_HAL_Driver/` → **STM32F4xx HAL**
- `Drivers/CMSIS/Device/ST/STM32H7xx/` → **CMSIS Device for STM32F4**
- `Drivers/CMSIS/DSP/` + `Middlewares/Third_Party/ARM_CMSIS/` → **CMSIS-DSP** built for Cortex-M4/FPv4-SP
- `startup_stm32h725xx.s`, `STM32H725xG_flash.ld`, `x6200_bb_fw.ioc` → F427 startup, linker, CubeMX project

## Milestones

- **M0 — scaffold** *(in progress)*: new GPLv3 repo seeded from upstream, X6100
  README + this plan. Next: re-vendor F4 HAL + CMSIS-DSP, CMake/Ninja for the
  F427 target, minimal BSP that boots on the F427 (reuse existing phase0 work).
- **M1 — App-layer spike**: one `App/SDR` demodulator compiling against CMSIS-DSP
  on the F427 with a software CORDIC, fed by a stub I/Q source. Proves the App
  layer ports before committing the project to it.
- **M2 — real transport**: graft the X6100 sample transport (codec-I/Q + on-MCU
  decimation chain) into the upstream RX ring → live RX on an upstream demod.
- **M3 — Devices**: port `Devices/` for X6100 parts (codec, clock gen, IO
  expander, fuel gauge / charger).
- **M4 — host link**: X6100 host protocol layer.
- **M5 — TX**: transmit chain bring-up (upstream TX + X6100 TX hardware). The
  main prize — no existing X6100 TX work to reuse.
- **M6 — protections + ATU**: temperature / voltage / SOC / VSWR protections and
  the automatic antenna tuner.

## Operating constraints (project rules)

- **Never flash the X6100 MCU from here.** Build the image; Franco runs the
  flasher (including any dry-run / restore).
- Firmware-only diagnosis — the radio is not opened; no hardware probing.
