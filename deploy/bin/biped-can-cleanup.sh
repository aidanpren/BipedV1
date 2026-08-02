#!/usr/bin/env bash
# Remove any leftover slcand daemon and any zombie CAN netdev, so that the
# service can always start from a known-clean state.
#
# THE ORDER IS THE ENTIRE POINT OF THIS SCRIPT. Daemon first, netdev second.
#
# Why: slcand attaches the N_SLCAN line discipline to the CANable's tty fd, and
# the kernel registers the netdev against THAT LINE DISCIPLINE, not against the
# daemon's process. So:
#   * kill slcand uncleanly (or yank the USB), and the netdev survives with no
#     tty behind it — a carrier-less `can0` squatting the name. `ip link set up`
#     then TIMES OUT (`failed to send bitrate command 'C\rS6\r'` in dmesg,
#     because the driver writes that string to a serial port that is gone), and
#     a fresh slcand fails at the rename with `SIOCSIFNAME: File exists`.
#   * delete the netdev while slcand is still alive and it simply recreates it,
#     which is why a bare `ip link delete can0` looks like it did nothing.
# Interface names are a kernel-global namespace, so "just use can1" steps around
# the corpse rather than removing it. That is how this project drifted from can0
# to can1 to can2 on the bench.
#
# NOT run under `set -e`: every command in here is EXPECTED to fail in the
# normal case (nothing to kill, nothing to delete). Failing to clean up a mess
# that isn't there must not fail the service.

set -uo pipefail

[ -r /etc/default/biped ] && . /etc/default/biped
iface="${CAN_IFACE:-can0}"

# `-x` matches the process NAME exactly, NOT the command line. Deliberate:
# `pkill -f slcand` matches any command line CONTAINING the string, which on
# this project has previously included the killing shell itself and the unit
# that invoked it. -x cannot self-match, because this script is not named
# `slcand`.
if pkill -x slcand; then
    echo "cleanup: killed a running slcand"
    # The line discipline is torn down asynchronously on process exit. Deleting
    # the netdev before that completes is the race this sleep closes.
    sleep 0.3
fi

if [ -d "/sys/class/net/${iface}" ]; then
    echo "cleanup: removing existing netdev ${iface}"
    ip link set "${iface}" down 2>/dev/null || true
    ip link delete "${iface}" 2>/dev/null || true
fi

# Always succeed. ExecStartPre failing would abort the start.
exit 0
