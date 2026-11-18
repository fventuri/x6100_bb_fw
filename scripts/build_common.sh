#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-or-later
# Shared build settings for x6100_bb_fw firmware images.
# Source this, set LINK_LD, then call: build_firmware <out-name> <src...>
#
# STM32F427, 32-bit, hard float. NO STM32 HAL: uses ST's STM32CubeF4 only for
# the CMSIS device headers and the header-only STM32 Low-Level (LL) inline API
# (-DUSE_FULL_LL_DRIVER). No LL/HAL .c sources are compiled -- every LL function
# used is __STATIC_INLINE and the clock tree is set with direct register writes.

REPO_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
PLATFORM_DIR="$REPO_ROOT/Platform/f427"

CUBE=${CUBE:-$HOME/st/STM32CubeF4}
CMSIS_CORE="$CUBE/Drivers/CMSIS/Include"
CMSIS_DEV="$CUBE/Drivers/CMSIS/Device/ST/STM32F4xx/Include"
LL_INC="$CUBE/Drivers/STM32F4xx_HAL_Driver/Inc"

CC=${CC:-arm-none-eabi-gcc}
OBJCOPY=${OBJCOPY:-arm-none-eabi-objcopy}
NM=${NM:-arm-none-eabi-nm}

# HSE_VALUE = board 25 MHz crystal. HCLK_MHZ selects core clock (250 default / 168).
DEFS="-DSTM32F427xx -DUSE_FULL_LL_DRIVER -DHSE_VALUE=25000000 -DHCLK_MHZ=${HCLK_MHZ:-250}"
INCS="-I$CMSIS_CORE -I$CMSIS_DEV -I$LL_INC -I$PLATFORM_DIR"
CFLAGS="-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Os \
        -ffreestanding -fno-common -Wall -Wextra \
        -ffunction-sections -fdata-sections $DEFS $INCS"
LINK_LD=${LINK_LD:-$PLATFORM_DIR/stm32f427.ld}
LDFLAGS="-T $LINK_LD -Wl,--gc-sections -Wl,-Map,firmware.map \
         --specs=nano.specs --specs=nosys.specs -nostartfiles"

FOUNDATION_SRC="$PLATFORM_DIR/startup.c $PLATFORM_DIR/bsp.c $PLATFORM_DIR/i2c.c"

check_cube()
{
    if [ ! -f "$CMSIS_DEV/stm32f427xx.h" ]; then
        echo "STM32CubeF4 not found under '$CUBE'. Set CUBE=/path/to/STM32CubeF4." >&2
        exit 1
    fi
}

# build_firmware <out-basename> <extra .c files / flags...>
build_firmware()
{
    check_cube
    out="$1"; shift
    $CC $CFLAGS $LDFLAGS "$@" $FOUNDATION_SRC -o "$out.elf"
    $OBJCOPY -O binary "$out.elf" "$out.bin"
    echo "built $out.elf / $out.bin:"
    ls -l "$out.bin"
    echo "first 2 words (little-endian MSP, reset_vector):"
    od -An -tx4 -N8 "$out.bin"
    if $NM "$out.elf" | grep -q uldivmod; then
        echo "WARNING: __aeabi_uldivmod present (64-bit divide) -- check the code" >&2
    fi
}

# xgf_encrypt <basename>: AES-256-ECB (Xiegu IAP key) for flashing hardware.
# Franco runs the flasher; this only produces the image.
xgf_encrypt()
{
    out="$1"
    XGF_KEY=435158494547552d584945594920202059414e4855414e204d4f554c49414e5a
    python3 -I - "$out" <<'PY'
import sys
b = sys.argv[1]
d = open(b + ".bin", "rb").read()
pad = (-len(d)) % 16
open(b + ".pad.bin", "wb").write(d + b"\xFF" * pad)
PY
    openssl enc -aes-256-ecb -nopad -K "$XGF_KEY" -in "$out.pad.bin" -out "$out.xgf"
    echo "built $out.xgf (encrypted; flash THIS to hardware):"
    ls -l "$out.xgf"
}
