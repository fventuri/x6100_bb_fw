#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-or-later
# Run an x6100_bb_fw image in Renode headless and capture the USART1 console.
#
#   ./run.sh                       # run bringup/m0_boot.elf, 3 s virtual time
#   ./run.sh <path/to/firmware.elf>
#   SECONDS_RUN=5 ./run.sh bringup/m0_boot.elf
#
# Needs the Renode portable build; override its location with RENODE=/path/renode.
set -e
cd "$(dirname "$0")"
REPO_ROOT=$(cd .. && pwd)

RENODE=${RENODE:-/home/franco/X6100/tools-renode/renode_1.17.0-portable/renode}
FW_ARG=${1:-bringup/m0_boot.elf}
case "$FW_ARG" in
    /*) FW="$FW_ARG" ;;
    *)  FW="$REPO_ROOT/$FW_ARG" ;;
esac
RUNFOR=${SECONDS_RUN:-3}

if [ ! -x "$RENODE" ]; then
    echo "Renode not found at '$RENODE'. Set RENODE=/path/to/renode." >&2
    exit 1
fi
if [ ! -f "$FW" ]; then
    echo "firmware not found: $FW (build it first)" >&2
    exit 1
fi

UARTLOG=$(mktemp /tmp/x6100-usart1.XXXXXX.log)
SCRIPT=$(mktemp /tmp/x6100-run.XXXXXX.resc)
{
    echo 'mach create "x6100"'
    echo "machine LoadPlatformDescription @$REPO_ROOT/renode/stm32f427.repl"
    echo "sysbus LoadELF @$FW"
    # Application vector table at 0x08020000 (128 KiB IAP bootloader below).
    echo "cpu VectorTableOffset 0x08020000"
    echo "sysbus.usart1 CreateFileBackend @$UARTLOG true"
    echo "emulation RunFor \"$RUNFOR\""
    echo "quit"
} > "$SCRIPT"

"$RENODE" --console --disable-xwt -e "include @$SCRIPT" >/dev/null 2>&1 || true
rm -f "$SCRIPT"

echo "=== USART1 ($FW_ARG, ${RUNFOR}s virtual) ==="
cat "$UARTLOG" 2>/dev/null || echo "(no USART1 output captured)"
rm -f "$UARTLOG"
