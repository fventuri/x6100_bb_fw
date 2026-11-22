/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */
/*
 * flow_uplink (F427, no-HAL) -- OEM X6100 USART6 flow-packet uplink.
 * ============================================================================
 * The other half of the OEM host protocol (M4a is the I2C3 downlink): the BASE
 * streams a 4136-byte "flow" packet to the R16 Linux host every ~35 ms over
 * USART6 @ 1.5 Mbaud via DMA2 Stream 6, carrying the live 100 kHz raw RX I/Q
 * (512 complex f32) + telemetry, hardware-CRC32 stamped. The host GUI builds the
 * waterfall/panadapter from the I/Q and DERIVES the S-meter from it. Ported from
 * the proven reference ~/X6100/x6100-bbfw/firmwares/phase4-linux/firmware.c.
 *
 * The RX datapath is on DMA1 + I2C3, so USART6/DMA2-S6/CRC/PC6/PC7 are free.
 */
#ifndef M4_FLOW_UPLINK_H
#define M4_FLOW_UPLINK_H

/* Bring up PC6/PC7 AF8, USART6 @ 1.5 Mbaud, DMA2 Stream 6 and the CRC engine.
 * Call once after i2s_start(). */
void Flow_Uplink_Init(void);

/* Build one flow packet from the live raw I/Q + LO center and kick its DMA
 * burst. Register on the scheduler at ~35 ms (OEM cadence). Runs in main-loop
 * context; the DMA send is one-shot (~28 us on the wire) and non-blocking. */
void Flow_Uplink_Tick(void);

#endif /* M4_FLOW_UPLINK_H */
