/* SPDX-License-Identifier: GPL-3.0-or-later */
/* M1 spike shim: route logging to the boot UART. ERROR/WARN only (the real
 * LOG_DEBUG lines use %f, which newlib-nano printf does not format). */
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "bsp.h"   /* bsp_uprintf */

#define LOG_ERROR(...) do { bsp_uprintf("[E][" LOG_TAG "] "); \
                            bsp_uprintf(__VA_ARGS__); bsp_uprintf("\r\n"); } while (0)
#define LOG_WARN(...)  do { bsp_uprintf("[W][" LOG_TAG "] "); \
                            bsp_uprintf(__VA_ARGS__); bsp_uprintf("\r\n"); } while (0)
#define LOG_INFO(...)  ((void)0)
#define LOG_DEBUG(...) ((void)0)
#define LOG_TRACE(...) ((void)0)
