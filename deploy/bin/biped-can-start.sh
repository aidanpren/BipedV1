#!/usr/bin/env bash
# Resolve the CANable's stable device path, then BECOME slcand.
#
# The `exec` at the bottom matters: this script replaces itself with slcand, so
# the process systemd supervises IS slcand. No wrapper process in the middle to
# confuse Restart=, and a SIGTERM from `systemctl stop` lands on the daemon
# itself rather than on a shell that would have to forward it.

set -euo pipefail

[ -r /etc/default/biped ] && . /etc/default/biped
iface="${CAN_IFACE:-can0}"
speed="${CAN_SPEED_INDEX:-6}"
wait_s="${CAN_DEVICE_WAIT:-20}"
pinned="${CAN_DEVICE:-}"

# ---------------------------------------------------------------------------
# Device resolution.
#
# /dev/ttyACM0 is NOT a stable name. It is assigned in enumeration order, so a
# reboot with a second CDC-ACM device attached, or a replug, moves the CANable
# to ttyACM1 — and slcand would then happily attach to whatever else landed on
# ttyACM0 and produce a silent, dead bus.
#
# /dev/serial/by-id/ is created by systemd's stock udev rules from the USB
# descriptor (vendor, product, serial number). It is stable across replug and
# across reboot, and it needs NO custom udev rule of our own — which also means
# there is no rule of ours that can silently stop matching after a firmware
# update changes a VID/PID.
# ---------------------------------------------------------------------------
resolve_device() {
    local link target candidates=()

    if [ -n "${pinned}" ]; then
        printf '%s\n' "${pinned}"
        return 0
    fi

    for link in /dev/serial/by-id/*; do
        [ -e "${link}" ] || continue          # glob did not match: skip the literal
        target="$(readlink -f "${link}")"
        # Only CDC-ACM devices. The CANable's slcan firmware enumerates as one;
        # this filters out FTDI/CP210x adapters (ttyUSB*) that are not it.
        case "${target}" in
            /dev/ttyACM*) candidates+=("${link}") ;;
        esac
    done

    if [ "${#candidates[@]}" -eq 1 ]; then
        printf '%s\n' "${candidates[0]}"
        return 0
    fi

    if [ "${#candidates[@]}" -gt 1 ]; then
        # More than one. Try to pick the CANable by name before giving up.
        local narrowed=()
        for link in "${candidates[@]}"; do
            case "${link,,}" in
                *canable*|*cantact*|*slcan*) narrowed+=("${link}") ;;
            esac
        done
        if [ "${#narrowed[@]}" -eq 1 ]; then
            printf '%s\n' "${narrowed[0]}"
            return 0
        fi
        # Ambiguous. REFUSE rather than guess: guessing here means attaching a
        # CAN line discipline to the wrong device, which fails silently.
        echo "ERROR: ${#candidates[@]} CDC-ACM devices present, cannot tell which is the CANable:" >&2
        printf '  %s\n' "${candidates[@]}" >&2
        echo "Pin the right one in /etc/default/biped as CAN_DEVICE=<path>." >&2
        return 1
    fi

    echo "ERROR: no CDC-ACM serial device found under /dev/serial/by-id/." >&2
    echo "Is the CANable plugged in? Check: ls -l /dev/serial/by-id/ ; lsusb" >&2
    return 1
}

# Wait for USB enumeration. At boot systemd starts units long before the USB
# tree has settled, so without this the service loses a race it could simply
# have waited out — and Restart=on-failure would paper over it as a confusing
# "it works on the second try".
deadline=$(( SECONDS + wait_s ))
while :; do
    if device="$(resolve_device 2>/dev/null)" && [ -e "${device}" ]; then
        break
    fi
    if [ "${SECONDS}" -ge "${deadline}" ]; then
        echo "ERROR: no CAN device after ${wait_s}s. Diagnosis follows:" >&2
        resolve_device >/dev/null || true    # re-run, this time letting it speak
        exit 1
    fi
    sleep 0.5
done

echo "slcand: using ${device} -> ${iface} at speed index s${speed}"

# Flags, all load-bearing:
#   -o  send the open command 'O\r' to the adapter
#   -c  send the close command on a CLEAN exit (does NOT fire on SIGKILL —
#       which is exactly why biped-can-cleanup.sh exists)
#   -f  read status flags, resetting any latched error state on the adapter
#   -s6 bitrate index (see CAN_SPEED_INDEX in /etc/default/biped)
#   -F  STAY IN FOREGROUND. Without it slcand daemonizes, systemd sees its main
#       process exit immediately, and Type=exec marks the unit failed and
#       restarts it forever. If you ever see that loop, check `slcand -h` on
#       this build for -F.
exec slcand -o -c -f -s"${speed}" -F "${device}" "${iface}"
