/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */
/*
 * bsp_memory.h spike shim. The real Platform/bsp_memory.h places large buffers
 * in an H7 ".ram_d1_bss" (AXI SRAM) section; the F427 has a single SRAM and its
 * linker script has no such section. Neutralise the placement so the buffers
 * land in ordinary .bss (keeping the 32-byte alignment). Production will either
 * add a .ram_d1_bss -> SRAM mapping to the F427 linker or make the real header
 * F427-aware; this shim keeps the spectrum spike self-contained.
 */
#pragma once

#define BSP_RAM_D1_BSS
#define BSP_RAM_D1_BSS_32 __attribute__((aligned(32)))
