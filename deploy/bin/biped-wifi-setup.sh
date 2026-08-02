#!/usr/bin/env bash
# ONE-TIME: create (or update) the robot's access-point profile.
#
#     sudo /usr/local/lib/biped/biped-wifi-setup.sh
#
# Separate from install.sh on purpose: it needs a passphrase, and it should not
# re-run on every `git pull`.
#
# ---------------------------------------------------------------------------
# THE PASSPHRASE IS NEVER STORED IN THIS REPO. It goes straight into
# NetworkManager's own connection store (/etc/NetworkManager/system-connections,
# root-only, mode 600), which is the thing already designed to hold it. It is
# read with `read -rs` so it is never echoed, and never written to any file we
# track.
#
# ONE HONEST CAVEAT: `nmcli con modify ... wifi-sec.psk "$psk"` does put the
# passphrase in nmcli's ARGV, so for the few milliseconds that process lives it
# is visible in `ps` to any other user on the box. On a single-user robot that
# is not a threat worth adding fragility for — the alternative is hand-writing
# NetworkManager's keyfile format, which is easier to get subtly wrong than a
# transient `ps` entry is to exploit. Stated rather than glossed over, because
# "we never expose it" would have been a claim this code does not honour.
#
# WHY WPA2 AND NOT AN OPEN NETWORK: this is a safety control, not hygiene.
# rosbridge binds 0.0.0.0:9090 and exposes /set_mode. On an open AP anyone in
# range can call it, put the robot in TELEOP and energize the motors. The
# passphrase is what stops a stranger arming your robot.
# ---------------------------------------------------------------------------

set -euo pipefail

[ -r /etc/default/biped ] && . /etc/default/biped
iface="${WIFI_IFACE:-wlan0}"
ap_con="${WIFI_AP_CON:-biped-ap}"
ap_ssid="${WIFI_AP_SSID:-BipedV1}"

if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: must run as root (use sudo)." >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# YOU ARE PROBABLY ABOUT TO CUT YOUR OWN CONNECTION.
# If you are ssh'd in over WiFi, activating the AP drops the network you are
# sitting on — the same shape of mistake as pkill'ing your own shell. Say so
# BEFORE doing anything, while it is still cheap to back out.
# ---------------------------------------------------------------------------
if [ -n "${SSH_CONNECTION:-}" ]; then
    ssh_ip="$(printf '%s' "${SSH_CONNECTION}" | awk '{print $3}')"
    if ip -brief addr show "${iface}" 2>/dev/null | grep -qF "${ssh_ip%/*}"; then
        echo "WARNING: you are ssh'd in over ${iface}, the interface this"
        echo "         script reconfigures. Activating the AP will DROP this"
        echo "         session. Use Ethernet or a console, or expect to"
        echo "         reconnect by joining '${ap_ssid}'."
        echo
    fi
fi

# --- preflight: can this radio even be an AP? ------------------------------
# Asked here rather than discovered later, because a driver that cannot do AP
# mode fails at `nmcli con up` with a generic activation error that reads like
# a config mistake.
if ! command -v nmcli >/dev/null 2>&1; then
    echo "MISSING: nmcli. This design assumes NetworkManager." >&2
    exit 1
fi
if ! ip link show "${iface}" >/dev/null 2>&1; then
    echo "ERROR: no interface '${iface}'. Set WIFI_IFACE in /etc/default/biped." >&2
    exit 1
fi
if command -v iw >/dev/null 2>&1; then
    if iw list 2>/dev/null | grep -qi '^\s*\* AP$'; then
        echo "ok: radio reports AP mode support"
    else
        echo "WARNING: 'iw list' does not report AP mode for this radio." >&2
        echo "         Continuing, but activation may fail." >&2
    fi
else
    echo "note: iw not installed, skipping the AP-capability check"
    echo "      (sudo apt install iw) — worth doing before you debug a failure"
fi

# Regulatory domain. An unset domain silently restricts channels and is a
# known cause of an AP that creates itself and then carries nobody.
reg="$(iw reg get 2>/dev/null | awk '/country/{print $2; exit}' || true)"
case "${reg}" in
    ""|00:*) echo "WARNING: wireless regulatory domain is unset (${reg:-none})."
             echo "         Set it, e.g.:  sudo iw reg set US" ;;
    *)       echo "ok: regulatory domain ${reg%:*}" ;;
esac

# --- passphrase -------------------------------------------------------------
echo
echo "AP SSID:      ${ap_ssid}"
echo "AP profile:   ${ap_con}"
echo "Interface:    ${iface}"
echo
# -s: not echoed to the terminal, and never placed on a command line.
read -rsp 'WPA2 passphrase (8-63 chars, empty to keep existing): ' psk
echo

if [ -z "${psk}" ]; then
    if nmcli -t -f NAME con show | grep -qx "${ap_con}"; then
        echo "keeping the existing passphrase on '${ap_con}'"
    else
        echo "ERROR: no existing '${ap_con}' profile, so a passphrase is required." >&2
        exit 1
    fi
elif [ "${#psk}" -lt 8 ] || [ "${#psk}" -gt 63 ]; then
    echo "ERROR: WPA2 passphrase must be 8-63 characters." >&2
    exit 1
fi

# --- create or update the profile ------------------------------------------
if nmcli -t -f NAME con show | grep -qx "${ap_con}"; then
    echo "updating existing profile '${ap_con}'"
else
    echo "creating profile '${ap_con}'"
    nmcli con add type wifi ifname "${iface}" con-name "${ap_con}" \
        ssid "${ap_ssid}" >/dev/null
fi

nmcli con modify "${ap_con}" \
    802-11-wireless.mode ap \
    802-11-wireless.ssid "${ap_ssid}" \
    `# 2.4 GHz: range and phone compatibility matter here, throughput does not` \
    `# — the dashboard is a few KB.` \
    802-11-wireless.band bg \
    ipv4.method shared \
    ipv6.method ignore \
    `# autoconnect NO is load-bearing. biped-wifi-mode.sh decides which network` \
    `# is up. If NetworkManager also had an opinion the two would fight, and` \
    `# the symptom would be an interface that flaps with no single cause to` \
    `# point at. Exactly one thing decides.` \
    connection.autoconnect no

if [ -n "${psk}" ]; then
    nmcli con modify "${ap_con}" \
        wifi-sec.key-mgmt wpa-psk \
        wifi-sec.psk "${psk}"
fi
unset psk

echo
echo "profile ready. It is NOT active yet."
echo
echo "  bring it up:    sudo /usr/local/lib/biped/biped-wifi-mode.sh ap"
echo "  back to home:   sudo /usr/local/lib/biped/biped-wifi-mode.sh home"
echo "  what's on now:  /usr/local/lib/biped/biped-wifi-mode.sh status"
echo
echo "Once active, the robot is at 10.42.0.1 (NetworkManager's shared subnet):"
echo "  dashboard   http://10.42.0.1:8000/"
echo "  rosbridge   ws://10.42.0.1:9090"
echo "  ssh         ssh ${SUDO_USER:-aidanpren}@10.42.0.1"
echo "biped.local should also resolve if avahi-daemon is running."
