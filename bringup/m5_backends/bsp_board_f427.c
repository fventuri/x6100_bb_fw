/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */
/*
 * bsp_board (F427, no-HAL) -- X6100 named board-level discrete signals for the
 * clean App surface (bsp_board.h). Replaces the X6200 Platform/bsp_board.c (wrong
 * GPIO/board). This is the M5-0 scaffold: every TX/PA-affecting signal is forced
 * to its SAFE / RX / OFF state so adopting app_radio.c for RX cannot assert TX or
 * key the PA. The real X6100 GPIO mapping (T/R PE8/PG11 TX vs PE9 RX + 5T/5R
 * rails, PA-bias enable, ACC TRX, status LEDs) lands in M5b, once app_protect and
 * the T/R sequencing are in place. See docs/pa_control.md.
 */
#include "bsp_board.h"

#include "bsp.h"   /* CMSIS core (NVIC_SystemReset, __disable_irq) + device regs */

/* ---- inputs: report the safe/idle reading (no TX request, no mic) ---------- */
bool Bsp_Board_IsPowerButtonPressed(void) { return false; }
bool Bsp_Board_IsExtMicPttPressed(void)   { return false; }
bool Bsp_Board_IsCwHostKeyDown(void)      { return false; }  /* active-low idle  */
bool Bsp_Board_IsExtMicConnected(void)    { return false; }

/* ---- power / indicators: no-op in M5-0 (owned by the host-link power path) -- */
void Bsp_Board_SetLinuxPower(bool enabled) { (void)enabled; }
void Bsp_Board_SetPowerHold(bool enabled)  { (void)enabled; }
void Bsp_Board_SetStatusLedGreen(bool on)  { (void)on; }
void Bsp_Board_SetStatusLedRed(bool on)    { (void)on; }

/* ---- TX / PA discretes: HARD-SAFE for M5-0 -- never assert TX or bias ------- *
 * app_radio only drives these in TX mode, which M5-0 never enters; keeping them
 * no-ops is belt-and-suspenders so a stray call cannot energize the PA chain. */
void Bsp_Board_SetAccTrx(bool on)     { (void)on; }
void Bsp_Board_SetPABias(bool on)     { (void)on; }  /* PA transistors stay OFF  */
void Bsp_Board_ConnectRxADC(bool on)  { (void)on; }
void Bsp_Board_SetTxRelay(bool on)    { (void)on; }  /* antenna relay stays RX   */

/* Final power cut: the OEM drops the rails and resets. With the rail GPIOs not
 * yet mapped (M5b), do the safe subset -- mask interrupts and reset the MCU. */
void Bsp_Board_FinalPowerCut(void)
{
    __disable_irq();
    NVIC_SystemReset();
    for (;;) { }   /* unreachable; satisfies __attribute__((noreturn)) */
}
