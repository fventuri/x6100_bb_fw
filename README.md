# X6100 base-band firmware

An independent, open-source replacement for the base-band (MCU) firmware of the
**Xiegu X6100** transceiver.

## Lineage and license

This project is a **derivative work** of Georgy Dyuldin's open-source X6200
base-band firmware — <https://github.com/gdyuldin/x6200_bb_fw> (commit
`014ee2d`) — and inherits its architecture and much of its portable signal
processing. It is distributed under the **GNU General Public License v3.0**
(see [LICENSE.txt](LICENSE.txt)), the same license as the upstream project.
Original copyright and license headers are retained.

Enormous thanks to Georgy for publishing a clean, complete, from-scratch design
to build on.

## Disclaimer

This is an independent, unofficial firmware project. It is **not affiliated
with, authorized by, endorsed by, or connected to Xiegu** in any way. "Xiegu"
and "X6100" are trademarks of their respective owners, used here only to
describe hardware compatibility.

Flashing or running this firmware is done **entirely at your own risk**. It is
provided **"as is", without warranty of any kind**. The authors accept **no
liability** for damage to your radio or other hardware, data loss, or any legal
or regulatory consequence. You are solely responsible for operating within the
radio regulations of your country.

## Status

**Early port — not yet runnable.** This repo begins as a verbatim import of the
X6200 firmware and is being adapted to the X6100 board layer by layer. See
[PORTING.md](PORTING.md) for the plan and current milestone.

## How the X6100 differs from the X6200

| Area | X6200 (upstream) | X6100 (this project) |
|---|---|---|
| MCU | STM32H725 (Cortex-M7) | STM32F427 (Cortex-M4) |
| RX/TX I/Q | FPGA down/up-converts **and** decimates; MCU reads baseband I/Q over a parallel bus | No decimating FPGA — the MCU does the decimation/DSP itself off a CS4272-class codec |
| FM math | H7 hardware CORDIC | software CORDIC (no CORDIC peripheral on the F4) |
| Host link | X6200 GUI I2C control + UART telemetry | X6100 host protocol |

Because of these, the port is done **by layer**, not with `#ifdef`s: the portable
`App/` signal-processing layer is reused, the `Platform/`/`Core/` hardware layer
is rewritten for the F427, and the I/Q transport is replaced. See
[PORTING.md](PORTING.md).

## Building

The port is in progress; `bringup/` holds the milestone bring-up images (see
[PORTING.md](PORTING.md)). They build bare-metal for the STM32F427 with
`arm-none-eabi-gcc`, no STM32 HAL — CMSIS device + LL headers come from an
STM32CubeF4 install (`CUBE=~/st/STM32CubeF4` by default) and CMSIS-DSP from the
pinned `third_party/CMSIS-DSP` submodule.

```sh
git submodule update --init          # fetch the pinned CMSIS-DSP (v1.11.0)
cd bringup && ./build.sh             # M0 boot image -> m0_boot.elf/.bin
../renode/run.sh bringup/m0_boot.elf # run it headless in Renode
```

Upstream developer docs under [docs/](docs/) describe the original X6200 design
and remain a useful reference while the port is in progress. The imported
upstream `CMakeLists.txt` / `Core/` still target the H7 and are being replaced.
