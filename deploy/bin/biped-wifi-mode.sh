#!/usr/bin/env bash
# Decide (or force) whether the Pi is on home WiFi or is its own access point.
#
#     biped-wifi-mode.sh status   # read-only: what is up right now
#     biped-wifi-mode.sh auto     # the boot/timer decision
#     biped-wifi-mode.sh home     # force: join home WiFi
#     biped-wifi-mode.sh ap       # force: become the access point
#
# ---------------------------------------------------------------------------
# WHY THIS SCRIPT EXISTS AT ALL: the Pi 5 has ONE radio, and it cannot usefully
# be an access point and a home-WiFi client at the same time. So this is a
# either/or decision, and something has to make it. NetworkManager's own
# autoconnect-priority fallback is not used, because then TWO things would have
# opinions about which network is up and a flapping interface would have no
# single cause to point at. The AP profile is created with autoconnect=no
# (see biped-wifi-setup.sh) precisely so this script is the only decider.
#
# THE POLICY, and why it is asymmetric:
#   * Boot with home WiFi in range  -> join it. You get git pull, apt, ssh.
#   * Boot with home WiFi absent    -> become the AP. You get a dashboard.
#   * Already on the AP             -> STAY, by default. See below.
#
# Returning to home automatically is OFF by default (WIFI_RETURN_TO_HOME), for
# two reasons, and the second is the real one:
#
#   1. Most drivers CANNOT SCAN while operating as an AP on the same radio.
#      "Notice home WiFi came back" is therefore not a passive observation —
#      it requires tearing the AP down to look, which is a real outage.
#   2. That outage would land at the worst possible moment. If you are driving
#      in the field with a phone on the AP and you wander into home-WiFi range,
#      the dashboard would vanish mid-drive. Staying on the AP too long costs
#      you a `git pull` you will notice; dropping it costs you your telemetry
#      while the robot is live.
#
# So the default is "sticky AP", and switching back is a deliberate act:
# `biped-wifi-mode.sh home`, or a reboot at your desk.
# ---------------------------------------------------------------------------

set -uo pipefail

[ -r /etc/default/biped ] && . /etc/default/biped
iface="${WIFI_IFACE:-wlan0}"
ap_con="${WIFI_AP_CON:-biped-ap}"
home_con="${WIFI_HOME_CON:-}"
return_home="${WIFI_RETURN_TO_HOME:-false}"
confirm_needed="${WIFI_HOME_CONFIRM_CHECKS:-3}"

# /run is tmpfs: this counter is deliberately cleared by a reboot, because the
# hysteresis it implements is about "have I seen home WiFi repeatedly in THIS
# session", not across power cycles.
state_dir=/run/biped
seen_file="${state_dir}/home-seen-count"

log() { echo "wifi: $*"; }

active_con() {
    nmcli -t -f GENERAL.CONNECTION device show "${iface}" 2>/dev/null \
        | cut -d: -f2-
}

# Any wireless profile that is not our AP counts as a "home" candidate. Keeping
# this discovered rather than configured means a new WiFi network you join
# normally, with nmcli or the desktop, is picked up with no extra step.
home_profiles() {
    if [ -n "${home_con}" ]; then
        printf '%s\n' "${home_con}"
        return
    fi
    nmcli -t -f NAME,TYPE con show 2>/dev/null \
        | awk -F: -v ap="${ap_con}" '$2 == "802-11-wireless" && $1 != ap {print $1}'
}

home_ssid_visible() {
    # --rescan yes forces a fresh scan; a cached list would let a network that
    # vanished ten minutes ago still look present.
    local ssids
    ssids="$(nmcli -t -f SSID device wifi list --rescan yes 2>/dev/null | sort -u)"
    [ -n "${ssids}" ] || return 1
    while IFS= read -r prof; do
        [ -n "${prof}" ] || continue
        local ssid
        ssid="$(nmcli -t -f 802-11-wireless.ssid con show "${prof}" 2>/dev/null | cut -d: -f2-)"
        [ -n "${ssid}" ] || continue
        if printf '%s\n' "${ssids}" | grep -qxF "${ssid}"; then
            return 0
        fi
    done < <(home_profiles)
    return 1
}

up_ap() {
    log "activating access point '${ap_con}'"
    nmcli con up "${ap_con}" >/dev/null || { log "FAILED to bring up ${ap_con}"; return 1; }
    rm -f "${seen_file}"
    log "AP up. dashboard: http://10.42.0.1:8000/"
}

up_home() {
    local prof
    while IFS= read -r prof; do
        [ -n "${prof}" ] || continue
        log "trying home profile '${prof}'"
        if nmcli con up "${prof}" >/dev/null 2>&1; then
            rm -f "${seen_file}"
            log "joined '${prof}'"
            return 0
        fi
    done < <(home_profiles)
    log "no home profile could be activated"
    return 1
}

cmd_status() {
    local con
    con="$(active_con)"
    echo "interface : ${iface}"
    echo "connection: ${con:-<none>}"
    if [ "${con}" = "${ap_con}" ]; then
        echo "role      : ACCESS POINT"
        echo "dashboard : http://10.42.0.1:8000/"
    elif [ -n "${con}" ]; then
        echo "role      : client (home WiFi)"
    else
        echo "role      : disconnected"
    fi
    echo "addresses :"
    ip -brief addr show "${iface}" 2>/dev/null | sed 's/^/  /'
    echo "return-to-home: ${return_home}"
}

cmd_auto() {
    local con
    con="$(active_con)"

    if [ "${con}" = "${ap_con}" ]; then
        if [ "${return_home,,}" != "true" ]; then
            log "on AP; sticky (WIFI_RETURN_TO_HOME=${return_home}). Nothing to do."
            return 0
        fi
        # Opt-in path. Note this DROPS the AP to scan — see the header.
        log "on AP; checking whether home WiFi is back (this briefly drops the AP)"
        local count=0
        [ -r "${seen_file}" ] && count="$(cat "${seen_file}" 2>/dev/null || echo 0)"
        if home_ssid_visible; then
            count=$(( count + 1 ))
            mkdir -p "${state_dir}"; echo "${count}" > "${seen_file}"
            log "home WiFi seen (${count}/${confirm_needed})"
            if [ "${count}" -ge "${confirm_needed}" ]; then
                up_home || up_ap
            fi
        else
            rm -f "${seen_file}"
        fi
        return 0
    fi

    if [ -n "${con}" ]; then
        log "already on '${con}'. Nothing to do."
        return 0
    fi

    # Disconnected. Prefer home, fall back to the AP.
    log "not connected; looking for home WiFi"
    if home_ssid_visible && up_home; then
        return 0
    fi
    log "no home WiFi — falling back to the access point"
    up_ap
}

case "${1:-status}" in
    status) cmd_status ;;
    auto)   cmd_auto ;;
    ap)     up_ap ;;
    home)   up_home ;;
    -h|--help)
        sed -n '2,8p' "$0" | sed 's/^# \?//' ;;
    *)
        echo "usage: $(basename "$0") {status|auto|home|ap}" >&2
        exit 2 ;;
esac
