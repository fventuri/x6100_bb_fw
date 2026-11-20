#!/usr/bin/env python3
"""
Generate the X6100 audio-output interpolation filter (50 -> 100 kHz, x2).

The ported App/SDR RX pipeline emits demodulated audio at the 50 kHz base rate
(BOARD_AUDIO_SAMPLE_RATE_HZ). The X6100 codec DAC, however, runs on the fixed,
externally-clocked 100 kHz I2S transport (common/i2s.c: i2s_rate_raw/… the TX
audio bank is 128 frames at 100 kHz). So the firmware must upsample the 50 kHz
audio by 2 before packing the codec TX-audio bank. M2's spike used a crude
linear x2; this is the proper replacement: a polyphase half-band FIR
interpolator for arm_fir_interpolate_f32 (factor L=2).

This stage is X6100-specific audio-OUT glue, NOT part of Georgy's ported
App/SDR resampling family (which ends at the 50 kHz base). It therefore lives
under Platform/f427/ with its own generator, mirroring the design method of
tools/gen_sdr_resampling_filters.py (scipy.signal.remez, time-reversed taps,
x2 level-preserving scale) so it is recognisably the same convention.

A half-band for x2 interpolation has its transition centred on fs_out/4 =
25 kHz (= the 50 kHz input Nyquist): f_pass + f_stop == fs_out/2 == 50 kHz.
The audio content of interest (AM ±4.5 kHz, SSB/CW narrower) sits far inside the
passband; every image of it lands above the stopband edge, so it is deeply
suppressed.

Run:
    python3 tools/gen_audio_out_interp.py

Output:
    Platform/f427/audio_out_interp_filter.h
"""

from pathlib import Path

import numpy as np
import scipy.signal as sig

# ---------------------------------------------------------------------------
# Configuration. Rates and band edges in kHz. Grid = the 100 kHz OUTPUT rate
# (the rate the interpolating FIR runs at). f_pass + f_stop == fs_out/2.
# ---------------------------------------------------------------------------
STAGE = {
    "name": "audio_out_interp_50_100",
    "fs_in_khz": 50,     # ported RX pipeline audio rate (base)
    "fs_out_khz": 100,   # X6100 codec DAC / I2S TX rate
    "f_pass_khz": 18.75, # same normalised edges as sdr_interp_48_9 (x2 family)
    "f_stop_khz": 31.25, # 18.75 + 31.25 == 50 == fs_out/2  -> half-band
    "order": 34,         # rounded up to a multiple of 4 (CMSIS needs taps % L)
    "weight_pass": 1.0,
    "weight_stop": 5.76,
    "note": "X6100 audio out 50 -> 100 kHz interpolation (codec DAC)",
}

OUTPUT_PATH = (Path(__file__).resolve().parent.parent
               / "Platform/f427/audio_out_interp_filter.h")

INTERP_L = 2


def format_float(x: float) -> str:
    s = f"{x:.9g}"
    if "." not in s and "e" not in s and "E" not in s:
        s += ".0"
    return s + "f"


def round_taps_to_multiple_of_4(order: int) -> tuple[int, int]:
    num_taps = order + 1
    num_taps = ((num_taps + 3) // 4) * 4
    return num_taps, num_taps - 1


def measure_response(w, h, f_pass, f_stop) -> dict:
    pass_mask = w <= f_pass
    stop_mask = w >= f_stop
    pass_mag = np.abs(h[pass_mask])
    stop_mag = np.abs(h[stop_mask])
    pass_max_db = 20.0 * np.log10(np.max(pass_mag)) if pass_mag.size else float("nan")
    pass_min_db = 20.0 * np.log10(np.min(pass_mag)) if pass_mag.size else float("nan")
    stop_atten_db = -20.0 * np.log10(np.max(stop_mag)) if stop_mag.size else float("nan")
    return {
        "pass_max_db": pass_max_db,
        "pass_min_db": pass_min_db,
        "pass_ripple_db": pass_max_db - pass_min_db,
        "stop_atten_db": stop_atten_db,
    }


def design(cfg: dict) -> dict:
    name = cfg["name"]
    fs_in = cfg["fs_in_khz"]
    fs_out = cfg["fs_out_khz"]
    f_pass = cfg["f_pass_khz"]
    f_stop = cfg["f_stop_khz"]
    order = cfg["order"]
    w_pass = cfg["weight_pass"]
    w_stop = cfg["weight_stop"]

    if fs_out != INTERP_L * fs_in:
        raise ValueError(f"{name}: expected fs_out == {INTERP_L}*fs_in")
    if not (0.0 < f_pass < f_stop < fs_out / 2.0):
        raise ValueError(f"{name}: require 0 < f_pass < f_stop < fs_out/2")
    if abs((f_pass + f_stop) - fs_out / 2.0) > 1e-9:
        raise ValueError(
            f"{name}: half-band requires f_pass + f_stop == fs_out/2 "
            f"(got {f_pass} + {f_stop} = {f_pass + f_stop}, fs_out/2 = {fs_out/2.0})"
        )
    if w_pass <= 0.0 or w_stop <= 0.0:
        raise ValueError(f"{name}: weights must be > 0")

    num_taps, adjusted_order = round_taps_to_multiple_of_4(order)

    # The FIR runs at the 100 kHz output grid; normalise band edges to fs_out.
    bands = np.array([0.0, f_pass / fs_out, f_stop / fs_out, 0.5])
    desired = np.array([1.0, 0.0])
    weights = np.array([w_pass, w_stop])

    b = sig.remez(num_taps, bands, desired, weight=weights, fs=1.0)
    coeffs = b[::-1]  # CMSIS-DSP expects time-reversed coefficients

    w, h = sig.freqz(b, 1.0, worN=16384, fs=1.0)
    measured = measure_response(w * fs_out, h, f_pass, f_stop)

    return {
        "cfg": cfg,
        "num_taps": num_taps,
        "adjusted_order": adjusted_order,
        "coeffs": coeffs,
        **measured,
    }


def format_array(coeffs) -> str:
    return "\n".join(f"    {format_float(c)}," for c in coeffs)


def build_header(res: dict) -> str:
    cfg = res["cfg"]
    name = cfg["name"]
    macro = name.upper()
    num_taps = res["num_taps"]
    return "\n".join([
        "/* SPDX-License-Identifier: GPL-3.0-or-later */",
        "/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */",
        "#pragma once",
        "",
        "#include <arm_math.h>",
        "",
        "/*",
        " * X6100 audio-out interpolation coefficients (50 -> 100 kHz, x2).",
        " * Generated by tools/gen_audio_out_interp.py. Do not edit by hand.",
        " *",
        " * Polyphase half-band FIR for arm_fir_interpolate_f32 (L=2). Time-",
        " * reversed, level-preserving (scale-invariant design * L=2 so the",
        " * interpolator's passband gain is unity). Band edges in kHz on the",
        " * 100 kHz output grid.",
        " *",
        f" *   {name}",
        " *   Kind            : FIR interpolator, interpolation by 2",
        f" *   Input rate      : {cfg['fs_in_khz']} kHz (ported RX audio base)",
        f" *   Output rate     : {cfg['fs_out_khz']} kHz (codec DAC / I2S TX)",
        f" *   Passband edge   : {cfg['f_pass_khz']} kHz",
        f" *   Stopband edge   : {cfg['f_stop_khz']} kHz (half-band: sum = fs_out/2)",
        f" *   Taps            : {num_taps} (multiple of L=2)",
        f" *   Weights         : passband={cfg['weight_pass']:g}, "
        f"stopband={cfg['weight_stop']:g}",
        f" *   Measured        : passband ripple {res['pass_ripple_db']:.3f} dB, "
        f"stopband attenuation {res['stop_atten_db']:.3f} dB",
        f" *   Purpose         : {cfg['note']}",
        " */",
        f"#define {macro}_NUM_TAPS {num_taps}u",
        f"#define {macro}_FACTOR {INTERP_L}u",
        f'_Static_assert({macro}_NUM_TAPS % {macro}_FACTOR == 0u,'
        f' "{name} taps must be a multiple of L");',
        "",
        f"static const float32_t {name}_coeffs[{macro}_NUM_TAPS] = {{",
        format_array(res["coeffs"] * float(INTERP_L)),
        "};",
        "",
    ])


def main() -> int:
    try:
        res = design(STAGE)
    except ValueError as exc:
        print(f"ERROR: {exc}")
        return 1

    cfg = STAGE
    print(f"{cfg['name']}: {cfg['fs_in_khz']} -> {cfg['fs_out_khz']} kHz, "
          f"L={INTERP_L}, kind=fir_interp")
    print(f"  passband : 0 .. {cfg['f_pass_khz']} kHz")
    print(f"  stopband : {cfg['f_stop_khz']} .. {cfg['fs_out_khz'] / 2.0:g} kHz")
    print(f"  order N  : {cfg['order']} -> {res['adjusted_order']} "
          f"({res['num_taps']} taps)")
    print(f"  ripple   : {res['pass_ripple_db']:.3f} dB")
    print(f"  atten    : {res['stop_atten_db']:.3f} dB")

    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_PATH.write_text(build_header(res), encoding="utf-8")
    print(f"Wrote {OUTPUT_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
