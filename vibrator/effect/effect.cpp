/*
 * SPDX-FileCopyrightText: The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

#include "tiktap_effect.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(*(a)))

#include "generated_effect.h"

#define PRIMITIVE_ID_MASK 0x8000

const struct effect_stream* get_effect_stream(uint32_t effect_id) {
    const struct effect_stream* table = effects;
    size_t count = ARRAY_SIZE(effects);

    if ((effect_id & PRIMITIVE_ID_MASK) != 0) {
        effect_id &= ~PRIMITIVE_ID_MASK;
        table = primitives;
        count = ARRAY_SIZE(primitives);
    }

    for (size_t i = 0; i < count; i++) {
        if (table[i].effect_id == effect_id) return &table[i];
    }

    return nullptr;
}

uint32_t get_effect_design_f0_x10() {
    return TIKTAP_EFFECT_F0_HZ_X10;
}
