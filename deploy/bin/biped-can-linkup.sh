#!/usr/bin/env bash
# Bring the CAN netdev up, once slcand has registered and renamed it.
#
# Runs as ExecStartPost. That placement is load-bearing for the WHOLE boot
# ordering: systemd holds a unit in `activating` until every ExecStartPost has
# finished, and units ordered `After=` wait for activation to complete. So
# biped-stack.service's `After=biped-can.service` does not merely mean "after
# slcand was exec'd" — it means "after can0 is UP". Without this script here,
# odrive_bridge could open a socket on an interface that exists but is down.

set -euo pipefail

[ -r /etc/default/biped ] && . /etc/default/biped
iface="${CAN_IFACE:-can0}"

# slcand registers the netdev as `slcan0` and then RENAMES it via SIOCSIFNAME.
# Both steps happen after exec, so the name we want does not exist yet at the
# instant ExecStartPost fires. Poll for it; 5 s is generous for a local ioctl.
for _ in $(seq 1 100); do
    [ -d "/sys/class/net/${iface}" ] && break
    sleep 0.05
done

if [ ! -d "/sys/class/net/${iface}" ]; then
    echo "ERROR: ${iface} never appeared. slcand started but did not register" >&2
    echo "the netdev — most likely the rename failed because a zombie already" >&2
    echo "owns the name. Check: ip -details link show ${iface}; dmesg | tail" >&2
    exit 1
fi

# Default txqueuelen on an slcan device is small, and CAN frames are tiny and
# bursty. A full queue shows up in userspace as ENOBUFS from a socket send —
# i.e. as dropped torque commands, not as an error you would think to look for.
ip link set dev "${iface}" txqueuelen 1000

# NOTE: no `type can bitrate ...` here. That is for NATIVE CAN controllers
# (an MCP2515 HAT would need it). On slcan the bitrate was already set by
# slcand's -s flag over the serial link, and passing it here would error.
ip link set dev "${iface}" up

echo "can: ${iface} is up"
ip -brief link show "${iface}"
