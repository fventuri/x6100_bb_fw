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
| **F427 shim** | `Platform/bsp_*`, `Core/` glue, `Devices/drv_codec` (AIC3204 → CS4272), `drv_clockgen`, `drv_pca9535`, `drv_gauge` / `drv_charger` / `drv_pmic` | Reimplement over the bare-metal `Platform/f427/` foundation (no HAL) for X6100 parts |
| **Rewrite — architectural** | `Devices/drv_fpga` parallel-bus transport, `Platform/bsp_cordic`, host protocol | Replace: FPGA-fed I/Q ring → codec-I/Q + on-MCU CMSIS-DSP decimation; software CORDIC; X6100 host map |

## No STM32 HAL — ever

This firmware uses **no STM32 HAL** (and no CubeMX glue). The house style is
CMSIS device header (`stm32f427xx.h`) + the STM32 **LL header-only inline** API
(`-DUSE_FULL_LL_DRIVER`, headers only, no LL `.c`) + direct register writes —
exactly the hardware-validated approach in `Platform/f427/`. Upstream's
HAL-based `Core/` and `Platform/bsp_*`/`Devices/drv_*` innards are reimplemented
in this style as they are ported; their clean logical API surface is kept.

## Not imported from upstream (provided for the F427)

The verbatim import omitted the third-party vendor trees (ST/ARM code, not
upstream's GPLv3 IP, and H7-specific). On the X6100 these are provided by the
STM32CubeF4 at build time (headers only) plus repo-local files:

- `Drivers/STM32H7xx_HAL_Driver/` → **dropped** (no HAL)
- `Drivers/CMSIS/…` (core + device + LL headers) → from **STM32CubeF4** at build time (Cortex-M4/FPv4-SP), headers only
- `Middlewares/Third_Party/ARM_CMSIS/` (CMSIS-DSP) → **`third_party/CMSIS-DSP` git submodule**, ARM CMSIS-DSP pinned at **v1.11.0** (the earliest standalone modular release; upstream's bundled "V1.10.0" was a patched snapshot). Upstream's App relied on `MAX`/`MIN` that its patched copy added to `dsp/utils.h`; the official library lacks them, so `compat/cmsis_dsp_compat.h` defines them (guarded) and the build force-includes it. Run `git submodule update --init` after cloning.
- `startup_stm32h725xx.s`, `STM32H725xG_flash.ld` → `Platform/f427/startup.c`, `Platform/f427/stm32f427.ld` *(done)*
- `x6200_bb_fw.ioc` → not used (no CubeMX)

## Milestones

- **M0 — scaffold** *(DONE — builds + boots in emulation)*: new GPLv3 repo
  seeded from upstream, X6100 README + this plan. Bare-metal F427 foundation in
  `Platform/f427/` (no HAL — CMSIS device header + LL header-only inline +
  direct registers), build via `scripts/build_common.sh`. `bringup/m0_boot.c`
  boots, brings up the OEM 250 MHz clock, asserts the safe GPIO state, and
  prints a banner + 1 Hz heartbeat on USART1 — verified in Renode
  (`renode/run.sh`). H7 startup/linker retired. Toolchain/linker proven before
  any App code.
- **M1 — App-layer spike** *(DONE — go/no-go PASSED)*: upstream's real,
  unmodified AM demodulator (`App/SDR/sdr_am.c` + the `sdr_iir.c` elliptic
  filter designer) compiles for the F427 against CMSIS-DSP and correctly
  recovers a test tone (`bringup/m1_am_spike.c`: AM carrier +500 Hz, 1 kHz tone,
  50% depth → rms 0.351 / peak 0.496 / 1000 Hz, Renode). The portable App layer
  ports and runs. System dependencies (IRQ, logging, codec block size, event
  bus, spectrum/noise) are HAL-free shims in `bringup/m1_shims/` for the spike;
  they become real milestones (M2/M3) later.
  - *CMSIS-DSP finding:* upstream's App targets the **modular** CMSIS-DSP (the
    `dsp/` layout with `dsp/utils.h` MAX/MIN), not the older monolithic
    `arm_math.h` in CubeF4. The project must vendor a modular CMSIS-DSP (see the
    re-vendoring note above). The spike builds against Georgy's bundled copy via
    `GDSP=...` until that is vendored.
  - *Software CORDIC — done & validated.* `Platform/f427/bsp_cordic.c` serves
    the App's two CORDIC entry points (`Bsp_Cordic_Atan2` for the FM
    discriminator, `Bsp_Cordic_SinCos` for the NFM TX modulator) with CMSIS-DSP
    FastMath (`arm_atan2_f32`, `arm_sin_f32`, `arm_cos_f32`) on the FPU. The
    F427 has no CORDIC peripheral, but the whole SDR chain is already float, so
    there is no fixed<->float conversion cost — which is exactly what made the
    hardware CORDIC a wash in practice. Verified on the F427 Renode model
    (`bringup/m1_cordic_test.c`): atan2 vs libm < 1 urad, sin/cos ~19 ppm.
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
