/*
 * SPDX-FileCopyrightText: The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>

// struct effect_stream and get_effect_stream(), shared with the QTI HAL
#include "effect.h"

// LRA resonant frequency (Hz * 10) the effect waveforms were designed for
uint32_t get_effect_design_f0_x10();
