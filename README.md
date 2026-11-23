# X6100 base-band firmware

An independent, open-source replacement for the base-band (MCU) firmware of the
**Xiegu X6100** transceiver.

## Status

**Working AM receive on real hardware; transmit bring-up in progress.**

- **RX** — The ported signal-processing pipeline runs on the STM32F427 off the
  X6100 codec: AM demodulation, FFT spectrum/waterfall, and AGC, all validated
  on an actual radio.
- **Host link** — The X6100 OEM host protocol is implemented (I2C3 control
  downlink + USART6 flow uplink), so the stock front panel / GUI drives the
  radio — tuning, mode and volume — with a live waterfall and RX levels
  calibrated to match the OEM firmware.
- **TX** — Transmit bring-up is underway: a single RX/TX orchestrator,
  forward/reflected/ALC and battery/PA-current sensing on the F427 dual ADC, and
  battery/thermal protection. The PA is still disabled pending further bench
  validation.

The project is organized as a sequence of milestone (M0–M5) bring-up steps, each
one radio-validated before the next; `bringup/` holds the per-milestone images.
See [PORTING.md](PORTING.md) for the plan and current milestone.

## How the X6100 differs from the X6200

| Area | X6200 (upstream) | X6100 (this project) |
|---|---|---|
| MCU | STM32H725 (Cortex-M7) | STM32F427 (Cortex-M4) |
| RX/TX I/Q | FPGA down/up-converts **and** decimates; MCU reads baseband I/Q over a parallel bus | No decimating FPGA — the MCU does the decimation/DSP itself off a CS4272-class codec |
| FM math | H7 hardware CORDIC | software CORDIC (no CORDIC peripheral on the F4) |
| Host link | X6200 GUI I2C control + UART telemetry | X6100 OEM host protocol (I2C3 + USART6) |

Because of these, the port is done **by layer**, not with `#ifdef`s: the portable
`App/` signal-processing layer is reused, the `Platform/`/`Core/` hardware layer
is rewritten for the F427, and the I/Q transport is replaced. See
[PORTING.md](PORTING.md).

## Building

`bringup/` holds the milestone bring-up images (see [PORTING.md](PORTING.md)).
They build bare-metal for the STM32F427 with `arm-none-eabi-gcc`, no STM32 HAL —
CMSIS device + LL headers come from an STM32CubeF4 install
(`CUBE=~/st/STM32CubeF4` by default) and CMSIS-DSP from the pinned
`third_party/CMSIS-DSP` submodule.

```sh
git submodule update --init          # fetch the pinned CMSIS-DSP (v1.11.0)
cd bringup && ./build.sh             # M0 boot image -> m0_boot.elf/.bin
../renode/run.sh bringup/m0_boot.elf # run it headless in Renode
```

Upstream developer docs under [docs/](docs/) describe the original X6200 design
and remain a useful reference while the port is in progress. The imported
upstream `CMakeLists.txt` / `Core/` still target the H7 and are being replaced.

## Reference

- **[gdyuldin/x6200_bb_fw](https://github.com/gdyuldin/x6200_bb_fw)** — Georgy
  Dyuldin's open-source base-band firmware for the Xiegu **X6200**. It is the
  upstream project this port is derived from and the primary design reference:
  the portable `App/` signal-processing layer comes from there (imported at
  commit `014ee2d`), and its developer docs document the architecture the X6100
  port reuses.

## Credits

This project stands on the work of others in the X6100/X6200 community:

- **Georgy Dyuldin (R2RFE)** — author of the
  [X6200 base-band firmware](https://github.com/gdyuldin/x6200_bb_fw) this
  project is derived from. Enormous thanks for publishing a clean, complete,
  from-scratch design to build on.
- **Oleg Belusov (R1CBU)** — for pioneering open-firmware and reverse-engineering
  work on the Xiegu X6100 that made independent firmware on this platform
  possible.

## License

This project is a **derivative work** of Georgy Dyuldin's open-source X6200
base-band firmware — <https://github.com/gdyuldin/x6200_bb_fw> (commit
`014ee2d`) — and inherits its architecture and much of its portable signal
processing. It is distributed under the **GNU General Public License v3.0**
(see [LICENSE.txt](LICENSE.txt)), the same license as the upstream project.
Original copyright and license headers are retained.

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
