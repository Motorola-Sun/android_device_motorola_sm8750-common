#!/usr/bin/env python3
#
# SPDX-FileCopyrightText: The LineageOS Project
# SPDX-License-Identifier: Apache-2.0
#
# Generates effect/generated_effect.h for the Motorola TikTap vibrator HAL.
#
# usage: generate_tiktap_effects.py <haptic_ram.bin> <tiktap_wave.bin> <out.h>
#
# Both inputs live in /vendor/firmware and use the Awinic RAM image format:
#
#   u16be checksum           sum of all following bytes
#   u16be base_addr          SRAM address of the byte following this field
#   u8    version
#   n * { u16be start, u16be end }   inclusive SRAM addresses of each wave
#   int8  samples...
#
# Wave count is (first_start - base_addr - 1) / 4, same as get_ram_num() in
# the haptic_hv kernel driver.
#
# haptic_ram.bin holds the short click waves Motorola tuned for the LRA (they
# are loaded into the chip SRAM by the kernel and used for duration based
# vibrations). tiktap_wave.bin holds the TikTap short wave library, a family
# of braked pulses with different carrier frequencies (~100 Hz .. ~330 Hz).
#
# All waves are 24 kHz int8 PCM designed for f0 = 170 Hz (f0_pre in the DT,
# also the first byte of tiktap_params.bin). The HAL resamples them to the
# measured f0 at runtime, and streams them through /proc/tiktap_buf.
#
# Effects that don't exist in the firmware (rise/fall/spin primitives) are
# synthesized as a sine at f0 with an envelope and an anti-phase brake.

import math
import struct
import sys
from typing import Dict, List, Tuple

PLAY_RATE_HZ = 24000
F0_HZ = 170.0
PEAK = 120

# AOSP Effect IDs
CLICK = 0
DOUBLE_CLICK = 1
TICK = 2
THUD = 3
POP = 4
HEAVY_CLICK = 5
TEXTURE_TICK = 21

# AOSP CompositePrimitive IDs
P_CLICK = 1
P_THUD = 2
P_SPIN = 3
P_QUICK_RISE = 4
P_SLOW_RISE = 5
P_QUICK_FALL = 6
P_LIGHT_TICK = 7
P_LOW_TICK = 8


def parse_ram_image(path: str) -> List[List[int]]:
    with open(path, "rb") as f:
        data = f.read()

    (checksum,) = struct.unpack(">H", data[0:2])
    if sum(data[2:]) & 0xFFFF != checksum:
        raise ValueError(f"{path}: checksum mismatch")

    (base_addr,) = struct.unpack(">H", data[2:4])
    (first_start,) = struct.unpack(">H", data[5:7])
    count = (first_start - base_addr - 1) // 4

    # File offset of SRAM address base_addr
    file_base = 4

    waves = []
    for i in range(count):
        start, end = struct.unpack(">HH", data[5 + i * 4 : 9 + i * 4])
        lo = start - base_addr + file_base
        hi = end - base_addr + file_base + 1
        if lo < 0 or hi > len(data) or lo >= hi:
            raise ValueError(f"{path}: wave {i + 1} out of bounds")
        waves.append([b - 256 if b > 127 else b for b in data[lo:hi]])

    return waves


def scale(wave: List[int], factor: float) -> List[int]:
    return [int(round(x * factor)) for x in wave]


def silence(ms: float) -> List[int]:
    return [0] * int(round(ms * PLAY_RATE_HZ / 1000))


def synth(duration_ms: float, envelope, brake: bool = True, freq_mod=None) -> List[int]:
    n = int(round(duration_ms * PLAY_RATE_HZ / 1000))
    out = []
    phase = 0.0
    amp = 0.0
    for i in range(n):
        t = i / n
        f = F0_HZ * (freq_mod(t) if freq_mod else 1.0)
        amp = envelope(t)
        out.append(int(round(PEAK * amp * math.sin(phase))))
        phase += 2 * math.pi * f / PLAY_RATE_HZ

    if brake and amp > 0.05:
        # Finish the current half cycle, then drive one anti-phase cycle with
        # a decaying amplitude to stop the LRA instead of letting it ring.
        period = int(round(PLAY_RATE_HZ / F0_HZ))
        for i in range(period):
            a = amp * 0.6 * (1 - i / period)
            out.append(int(round(-PEAK * a * math.sin(phase))))
            phase += 2 * math.pi * F0_HZ / PLAY_RATE_HZ

    # Clamp and trim trailing silence
    out = [max(-127, min(127, x)) for x in out]
    while out and out[-1] == 0:
        out.pop()
    return out


def build(ram: List[List[int]], tiktap: List[List[int]]):
    if len(ram) < 4:
        raise ValueError("haptic_ram.bin: expected at least 4 waves")
    if len(tiktap) < 9:
        raise ValueError("tiktap_wave.bin: expected at least 9 waves")

    # haptic_ram.bin waves:
    #   1: 16.8 ms, two strong cycles with brake
    #   2: 10.4 ms, click with brake
    #   3:  8.0 ms, sharp single cycle tick
    #   4:  5.9 ms, plain f0 cycle, used by the driver for long vibrations
    # tiktap_wave.bin waves 1..11 go from ~100 Hz (29.5 ms) to ~330 Hz
    # (11.2 ms), all braked.
    #
    # The stock HAL doesn't support effects, so the stock framework plays
    # its fallback patterns with on() at full amplitude, and the driver picks
    # a RAM wave from the duration (duration_time = 20, 30, 60 ms).
    # Measured on stock W1WRS36.39-115-4 (cmd overlay lookup + dumpsys):
    #   CLICK         virtualKey  20 ms         -> wave 2
    #   TICK          clockTick   10 ms         -> wave 3
    #   TEXTURE_TICK  clockTick   10 ms         -> wave 3
    #   HEAVY_CLICK   longPress   30 ms         -> wave 1
    #   DOUBLE_CLICK  30, 100, 30               -> wave 1, wave 1 130 ms later
    w_strong = ram[0]
    w_click = ram[1]
    w_tick = ram[2]
    w_thud = tiktap[0]
    w_low = tiktap[3]
    w_pop = tiktap[8]

    waves: Dict[str, List[int]] = {
        "click": w_click,
        "double_click": w_strong + silence(130 - len(w_strong) * 1000 / PLAY_RATE_HZ) + w_strong,
        "tick": w_tick,
        "thud": w_thud,
        "pop": w_pop,
        "heavy_click": w_strong,
        "texture_tick": w_tick,
        "light_tick": scale(w_tick, 0.55),
        "low_tick": scale(w_low, 0.7),
        "quick_rise": synth(80, lambda t: 0.15 + 0.85 * t * t),
        "slow_rise": synth(250, lambda t: 0.1 + 0.9 * t),
        "quick_fall": synth(60, lambda t: (1 - t) ** 2, brake=False),
        "spin": synth(
            150,
            lambda t: 0.3 + 0.7 * math.sin(math.pi * t),
            freq_mod=lambda t: 1.0 + 0.08 * math.sin(4 * math.pi * t),
        ),
    }

    effects: List[Tuple[int, str]] = [
        (CLICK, "click"),
        (DOUBLE_CLICK, "double_click"),
        (TICK, "tick"),
        (THUD, "thud"),
        (POP, "pop"),
        (HEAVY_CLICK, "heavy_click"),
        (TEXTURE_TICK, "texture_tick"),
    ]

    primitives: List[Tuple[int, str]] = [
        (P_CLICK, "click"),
        (P_THUD, "thud"),
        (P_SPIN, "spin"),
        (P_QUICK_RISE, "quick_rise"),
        (P_SLOW_RISE, "slow_rise"),
        (P_QUICK_FALL, "quick_fall"),
        (P_LIGHT_TICK, "light_tick"),
        (P_LOW_TICK, "low_tick"),
    ]

    return waves, effects, primitives


def write_array(o, name: str, wave: List[int]):
    o.write(f"static const int8_t wave_{name}[] = {{\n")
    per_line = 16
    for i in range(0, len(wave), per_line):
        chunk = wave[i : i + per_line]
        o.write("        " + " ".join(f"{x:>4}," for x in chunk) + "\n")
    o.write("};\n\n")


def write_table(o, table_name: str, entries: List[Tuple[int, str]]):
    o.write(f"static const struct effect_stream {table_name}[] = {{\n")
    for effect_id, name in entries:
        o.write(
            "        {\n"
            f"                .effect_id = {effect_id},\n"
            f"                .length = ARRAY_SIZE(wave_{name}),\n"
            f"                .play_rate_hz = {PLAY_RATE_HZ},\n"
            f"                .data = wave_{name},\n"
            "        },\n"
        )
    o.write("};\n\n")


def main():
    if len(sys.argv) != 4:
        raise SystemExit(f"usage: {sys.argv[0]} <haptic_ram.bin> <tiktap_wave.bin> <out.h>")

    ram = parse_ram_image(sys.argv[1])
    tiktap = parse_ram_image(sys.argv[2])
    waves, effects, primitives = build(ram, tiktap)

    used = sorted({n for _, n in effects} | {n for _, n in primitives})

    with open(sys.argv[3], "w", encoding="utf-8") as o:
        o.write(
            "/*\n"
            " * SPDX-FileCopyrightText: The LineageOS Project\n"
            " * SPDX-License-Identifier: Apache-2.0\n"
            " */\n\n"
            "/*\n"
            " * Generated by tools/generate_tiktap_effects.py from\n"
            " * /vendor/firmware/haptic_ram.bin and /vendor/firmware/tiktap_wave.bin.\n"
            " * Do not edit manually.\n"
            " */\n\n"
            "#pragma once\n\n"
            f"#define TIKTAP_EFFECT_F0_HZ_X10 {int(F0_HZ * 10)}\n\n"
        )
        for name in used:
            write_array(o, name, waves[name])
        write_table(o, "effects", effects)
        write_table(o, "primitives", primitives)

    for name in used:
        print(f"{name}: {len(waves[name])} samples, {len(waves[name]) * 1000 / PLAY_RATE_HZ:.1f} ms")


if __name__ == "__main__":
    main()
