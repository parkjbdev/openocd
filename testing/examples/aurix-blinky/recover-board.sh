#!/bin/bash
# SPDX-License-Identifier: GPL-2.0-or-later
# Recover an AURIX Lite Kit from a dead/unresponsive state back to a running
# blinky, using only this repository's own OpenOCD build (no vendor tooling,
# no other repo required). See RECOVERY.md in this directory for the full
# runbook this script automates; read it before using this on a board you
# have not recovered before.
#
# USAGE
#   recover-board.sh list
#   recover-board.sh <tc2|tc3|tc4> diagnose
#   recover-board.sh <tc2|tc3|tc4> recover
#
#   tc2 = TC275 Lite Kit (TC2x), tc3 = TC375 Lite Kit (TC3x),
#   tc4 = TC4D7 Lite Kit (TC4x)
#
# "diagnose" only issues read-only commands: adapter connect + core examine +
# a single register read, no reset, no halt, no flash read. Use it first to
# confirm which board is actually in trouble before reflashing anything.
# Caveat: "read-only" describes the commands this script sends, not a
# guarantee about the chip -- if the debug subsystem's IO client does not
# come up cleanly on the very first connect attempt, the low-level
# reconnect-retry logic that runs underneath those commands can itself
# disturb a running core (see the IMPORTANT note below). diagnose is safe
# precisely because it stops after one examine instead of going on to erase
# and program flash -- it is not risk-free on a board that is already
# borderline.
#
# "recover" runs this repo's proven one-line recovery command for that board
# (erase + program + verify + reset run, exactly what each board's own
# testing/examples/aurix-blinky/<family>/README documents and what the
# hardware regression suite itself uses to program the board every run):
#
#   program <family>-blinky.elf verify reset exit
#
# Boards are always selected by USB serial, never by USB location/port
# number: locations are not a stable identity for these probes -- a
# power-cycle has been observed to swap which location two different boards
# enumerate at, while both keep the same VID:PID (058b:0043). See
# ab-test/boards.sh in the top-level tricore tree for the incident this
# lesson comes from.
#
# One board at a time. This script never touches more than the one board
# named on the command line, even if others are attached.
#
# IMPORTANT -- if "diagnose" or "recover" fails with an "[OCMTS] UNWEDGE:
# exhausted N reset+reconnect cycles" error, STOP. Do not run this script
# again against the same board hoping a retry will succeed -- it will very
# likely fail the same way again and can turn a board that was still running
# fine into one showing the red fault LED (this has happened twice while
# developing this script: once against a board that still had residual state
# from a different debug-tool session, and once as a direct repeat of that
# same scenario). This signature means the on-chip debug subsystem's IO
# client did not come up cleanly on the very first connect attempt; it is
# not something a same-tool retry fixes. See RECOVERY.md's "If recovery
# fails" section for the escalation path (physical power-cycle first, then
# the vendor-stack fallback if one is available).

set -u

SELF="$(readlink -f "$0")"
AURIX_BLINKY_DIR="$(dirname "$SELF")"
REPO="$(readlink -f "$AURIX_BLINKY_DIR/../../..")"
BIN="$REPO/src/openocd"
TCL="$REPO/tcl"

# board table: name -> "serial board_cfg elf_relpath label"
board_info() {
    case "$1" in
        tc2) echo "LK7U8NLP board/infineon/kit_a1g_tc275_lite.cfg tc27x/tc27x-blinky.elf 'TC275 Lite Kit (TC2x)'" ;;
        tc3) echo "LKAGI1QM board/infineon/kit_a2g_tc375_lite.cfg tc375/tc375-blinky.elf 'TC375 Lite Kit (TC3x)'" ;;
        tc4) echo "LKBM1OL0 board/infineon/kit_a3g_tc4d7_lite.cfg tc4d7/tc4d7-blinky.elf 'TC4D7 Lite Kit (TC4x)'" ;;
        *) return 1 ;;
    esac
}

# List attached Infineon probes (VID:PID 058b:0043) with USB location and
# serial, straight from sysfs -- no lsusb dependency. Read-only, touches no
# DAP.
list_boards() {
    printf "%-8s %-22s %s\n" "loc" "product" "serial"
    for d in /sys/bus/usb/devices/*/; do
        [ -f "$d/idVendor" ] || continue
        [ "$(cat "$d/idVendor" 2>/dev/null)" = "058b" ] || continue
        [ "$(cat "$d/idProduct" 2>/dev/null)" = "0043" ] || continue
        [ -f "$d/interface" ] && continue
        printf "%-8s %-22s %s\n" \
            "$(basename "$d")" \
            "$(cat "$d/product" 2>/dev/null)" \
            "$(cat "$d/serial" 2>/dev/null)"
    done
}

serial_present() {
    for d in /sys/bus/usb/devices/*/; do
        [ -f "$d/serial" ] || continue
        [ "$(cat "$d/serial" 2>/dev/null)" = "$1" ] && return 0
    done
    return 1
}

usage() {
    echo "usage: $0 list"
    echo "       $0 <tc2|tc3|tc4> <diagnose|recover>"
    exit 2
}

[ $# -ge 1 ] || usage

if [ "$1" = "list" ]; then
    list_boards
    exit 0
fi

[ $# -eq 2 ] || usage
BOARD="$1"
ACTION="$2"

read -r SERIAL BOARD_CFG ELF_REL LABEL <<<"$(board_info "$BOARD")" || usage
LABEL=$(eval echo "$LABEL")
ELF="$AURIX_BLINKY_DIR/$ELF_REL"

[ -x "$BIN" ] || { echo "recover-board.sh: no OpenOCD build at $BIN -- build this repo first"; exit 1; }

echo "recover-board.sh: target is $LABEL, serial $SERIAL"
if ! serial_present "$SERIAL"; then
    echo "recover-board.sh: FAILED -- no attached probe has serial $SERIAL right now"
    echo "  Currently attached:"
    list_boards | sed 's/^/    /'
    exit 1
fi

case "$ACTION" in
    diagnose)
        [ -f "$ELF" ] || echo "recover-board.sh: note -- $ELF not found (not needed for diagnose)"
        LOG=$(mktemp -p "$AURIX_BLINKY_DIR" recover-diagnose.XXXXXX.log)
        echo "recover-board.sh: read-only connect + examine, log: $LOG"
        "$BIN" -s "$TCL" -f "$BOARD_CFG" \
            -c "adapter serial $SERIAL" \
            -c init -c "reg pc" -c shutdown \
            >"$LOG" 2>&1
        rc=$?
        if [ $rc -eq 0 ] && grep -q "Examination succeed" "$LOG"; then
            echo "recover-board.sh: connect OK, core(s) examined cleanly"
            grep -E "Tricore version|Examination succeed|pc " "$LOG" | sed 's/^/  /'
        else
            echo "recover-board.sh: connect FAILED or incomplete -- see $LOG"
            grep -E "Error|OCMTS|UNWEDGE|Warn" "$LOG" | sed 's/^/  /'
            if grep -q "UNWEDGE" "$LOG"; then
                echo "  Do NOT run this script again against $BOARD right now -- see"
                echo "  RECOVERY.md's 'If recovery fails' section for the escalation path."
            fi
        fi
        exit $rc
        ;;
    recover)
        [ -f "$ELF" ] || { echo "recover-board.sh: missing $ELF -- build it first (see that family's README)"; exit 1; }
        LOG=$(mktemp -p "$AURIX_BLINKY_DIR" recover-flash.XXXXXX.log)
        echo "recover-board.sh: programming $ELF_REL, log: $LOG"
        "$BIN" -s "$TCL" -f "$BOARD_CFG" \
            -c "adapter serial $SERIAL" \
            -c "program $ELF verify reset exit" \
            >"$LOG" 2>&1
        rc=$?
        if [ $rc -eq 0 ] && grep -q "Verified OK" "$LOG"; then
            echo "recover-board.sh: SUCCESS -- programmed, verified, reset run issued"
            echo "  Check the board now, and again after ~60s: LEDs should be blinking."
        else
            echo "recover-board.sh: FAILED -- see $LOG"
            grep -E "Error|Protection error|OCMTS|UNWEDGE" "$LOG" | sed 's/^/  /'
            if grep -q "UNWEDGE" "$LOG"; then
                echo "  Do NOT run this script again right now -- repeating this same"
                echo "  command will very likely fail the same way again."
            fi
            echo "  See RECOVERY.md's 'If recovery fails' section for the escalation path."
        fi
        exit $rc
        ;;
    *)
        usage
        ;;
esac
