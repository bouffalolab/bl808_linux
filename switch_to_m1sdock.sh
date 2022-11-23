#!/bin/bash
set -e

SHELL_DIR=$(cd "$(dirname "$0")"; pwd)

case "$0" in
*switch_to_m1sdock.sh)
echo " "
echo "===================== switch to M1sDock start ======================="
git apply ${SHELL_DIR}/patch/m1sdock/m1sdock_uart_pin_def.patch
mv ${SHELL_DIR}/{switch_to_m1sdock.sh,switch_back_from_m1sdock.sh}
echo "===================== switch to M1sDock done  ======================="
    ;;
*switch_back_from_m1sdock.sh)
echo " "
echo "================= switch back from M1sDock start ===================="
git apply --reverse ${SHELL_DIR}/patch/m1sdock/m1sdock_uart_pin_def.patch
mv ${SHELL_DIR}/{switch_back_from_m1sdock.sh,switch_to_m1sdock.sh}
echo "================= switch back from M1sDock done  ===================="
    ;;
esac
