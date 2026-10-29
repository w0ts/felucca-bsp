#!/bin/sh
# SPDX-License-Identifier: MPL-2.0
# Copyright (C) 2026 Kerem Kilic (Ellic Studio)
set -eu
cd "$(dirname "$0")/.."
USB_TEST_DIR=$(mktemp -d)
trap 'rm -rf "$USB_TEST_DIR"' EXIT HUP INT TERM
for mode in host host-audio host-name device loader; do
    case "$mode" in
        host) flags="" ;;
        host-audio) flags="-DTEST_UAC=1" ;;
        device) flags="-DTEST_HOST=0" ;;
        loader) flags="-DFELUCCA_LOADER=1" ;;
        host-name) flags="-DFELUCCA_USB_HOST_NAME=1" ;;
    esac
    # Word splitting is intentional for the fixed compiler options above.
    "${CC:-cc}" -std=c11 -O1 -Wall -Wextra -Werror -Wno-unused-function \
        -Wno-unused-variable -Wno-missing-field-initializers \
        -fsanitize=address,undefined -Ichip $flags tests/usb_host_test.c -o "$USB_TEST_DIR/$mode"
    "$USB_TEST_DIR/$mode"
done
# USB audio (tests/uac_test.c): the default configuration (byte for byte the descriptors before FELUCCA_UAC_CH and
# FELUCCA_UAC_24), the input alone, and the channel counts and sample sizes around it, with and without the console;
# on the host's clock (FELUCCA_UAC_HOST_CLOCK, TEST_HC: minutes of a model host and crystal) by default, in 4 channels
# 24-bit, and in 8 without the output; the output with its feedback (FELUCCA_UAC_OUT_FEEDBACK, TEST_FB: minutes of a
# model host that follows it) by default, in 4 channels 24-bit, and with the input on the host's clock too; the
# output in 24-bit too (FELUCCA_UAC_OUT_24, TEST_O24) adaptive, with the feedback, and with all of them; the input
# and the output as two functions (FELUCCA_UAC_SPLIT, TEST_SPLIT), named or not, alone, and with all of the above
for flags in "" "-DTEST_OUT=0" "-DTEST_CH=4 -DTEST_24=1" "-DTEST_CH=4 -DTEST_24=1 -DTEST_CDC=1" "-DTEST_CH=1 -DTEST_OUT=0" \
             "-DTEST_CH=2 -DTEST_24=1" "-DTEST_CH=8" "-DTEST_HC=1" "-DTEST_HC=1 -DTEST_CH=4 -DTEST_24=1" \
             "-DTEST_HC=1 -DTEST_CH=8 -DTEST_OUT=0" "-DTEST_FB=1" "-DTEST_FB=1 -DTEST_CH=4 -DTEST_24=1" \
             "-DTEST_FB=1 -DTEST_HC=1 -DTEST_CH=4 -DTEST_24=1" "-DTEST_O24=1" "-DTEST_FB=1 -DTEST_O24=1" \
             "-DTEST_FB=1 -DTEST_O24=1 -DTEST_HC=1 -DTEST_CH=4 -DTEST_24=1" "-DTEST_SPLIT=1" \
             "-DTEST_SPLIT=1 -DTEST_NAMES=1" "-DTEST_SPLIT=1 -DTEST_OUT=0" "-DTEST_SPLIT=1 -DTEST_NAMEFN=1" \
             "-DTEST_SPLIT=1 -DTEST_NAMEFN=1 -DTEST_OUT=0 -DTEST_CH=4" "-DTEST_NAMEFN=1" \
             "-DTEST_SPLIT=1 -DTEST_NAMES=1 -DTEST_FB=1 -DTEST_O24=1 -DTEST_HC=1 -DTEST_CH=4 -DTEST_24=1"; do
    # Word splitting is intentional for the fixed compiler options above.
    "${CC:-cc}" -std=c11 -O1 -Wall -Wextra -Werror -Wno-unused-function \
        -Wno-unused-variable -Wno-missing-field-initializers \
        -fsanitize=address,undefined -Ichip $flags tests/uac_test.c -o "$USB_TEST_DIR/uac" -lm
    "$USB_TEST_DIR/uac"
done
