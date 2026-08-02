#!/usr/bin/env bash
# Install the BipedV1 systemd units on the Pi.  Run ON THE PI, as root:
#
#     sudo ~/BipedV1/deploy/install.sh
#
# It is INSTALL-ONLY by default: it does not enable anything, so running it
# changes nothing about what happens at your next boot. That is deliberate.
# "The robot starts its motor stack the moment it has power" is a decision that
# deserves its own explicit command, taken after you have watched the services
# come up by hand. Pass --enable when you are ready, or run the systemctl
# commands this script prints.
#
# Idempotent: safe to re-run after every `git pull`.

set -euo pipefail

LIBDIR=/usr/local/lib/biped
UNITDIR=/etc/systemd/system
DEFAULTS=/etc/default/biped

DO_ENABLE=0
BIPED_USER=""
BIPED_WS=""

usage() {
    cat <<'EOF'
usage: sudo deploy/install.sh [--enable] [--user NAME] [--ws PATH]

  --enable      also `systemctl enable` both units (start at boot)
  --user NAME   account the ROS stack runs as   (default: the invoking user)
  --ws PATH     colcon workspace root           (default: parent of deploy/)
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --enable)  DO_ENABLE=1 ;;
        --user)    BIPED_USER="${2:?--user needs a value}"; shift ;;
        --ws)      BIPED_WS="${2:?--ws needs a value}"; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done

# --- who and where ----------------------------------------------------------
if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: must run as root (use sudo)." >&2
    exit 1
fi

# SUDO_USER is the account that invoked sudo — i.e. the human's account, not
# root. That is the account the ROS stack should run as, so it can reach the
# same built workspace and the same ~/.ros logs you read interactively.
if [ -z "${BIPED_USER}" ]; then
    BIPED_USER="${SUDO_USER:-}"
fi
if [ -z "${BIPED_USER}" ] || [ "${BIPED_USER}" = "root" ]; then
    echo "ERROR: could not determine a non-root user to run the stack as." >&2
    echo "Pass it explicitly:  sudo deploy/install.sh --user aidanpren" >&2
    exit 1
fi
if ! id "${BIPED_USER}" >/dev/null 2>&1; then
    echo "ERROR: no such user: ${BIPED_USER}" >&2
    exit 1
fi

HERE="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
if [ -z "${BIPED_WS}" ]; then
    BIPED_WS="$(dirname "${HERE}")"
fi

echo "installing BipedV1 services"
echo "  workspace : ${BIPED_WS}"
echo "  run as    : ${BIPED_USER}"
echo

# --- preflight --------------------------------------------------------------
# Every check below is a failure that would otherwise show up much later as a
# confusing runtime error in the journal.
fail=0

if ! command -v slcand >/dev/null 2>&1; then
    echo "MISSING: slcand. Install it:  sudo apt install can-utils" >&2
    fail=1
elif ! slcand -h 2>&1 | grep -q -- '-F'; then
    # Not fatal — the flag may just be undocumented in this build — but if it
    # is genuinely absent, slcand daemonizes, systemd sees the main process
    # exit at once, and the unit restart-loops. Worth knowing up front.
    echo "WARNING: this slcand build does not document -F (foreground)." >&2
    echo "         If biped-can.service restart-loops, that is why." >&2
fi

if [ ! -d "${BIPED_WS}/src" ]; then
    echo "MISSING: ${BIPED_WS}/src — that does not look like the workspace." >&2
    fail=1
fi

if [ ! -r "${BIPED_WS}/install/setup.bash" ]; then
    echo "MISSING: ${BIPED_WS}/install/setup.bash — workspace not built." >&2
    echo "         Run as ${BIPED_USER} (NOT root, or root will own build/):" >&2
    echo "           cd ${BIPED_WS} && colcon build --symlink-install" >&2
    fail=1
fi

if [ ! -r /opt/ros/jazzy/setup.bash ]; then
    echo "MISSING: /opt/ros/jazzy/setup.bash." >&2
    echo "         If ROS lives elsewhere, set ROS_DISTRO_SETUP in ${DEFAULTS}." >&2
    fail=1
fi

[ "${fail}" -eq 0 ] || { echo; echo "preflight failed; nothing installed." >&2; exit 1; }

# --- helper scripts ---------------------------------------------------------
install -d -m 0755 "${LIBDIR}"
for f in biped-can-cleanup.sh biped-can-start.sh biped-can-linkup.sh \
         biped-stack.sh biped-wifi-setup.sh biped-wifi-mode.sh; do
    install -m 0755 "${HERE}/bin/${f}" "${LIBDIR}/${f}"
    echo "  installed ${LIBDIR}/${f}"
done

# --- defaults file ----------------------------------------------------------
# NEVER overwrite an existing one. This is the file the operator edits in place
# on the robot (pinned CAN_DEVICE, ROS_DOMAIN_ID, debug launch args); silently
# reverting those on a re-run would be the worst kind of surprise.
install -d -m 0755 /etc/default
if [ -e "${DEFAULTS}" ]; then
    echo "  kept existing ${DEFAULTS} (not overwritten)"
    echo "    compare against the shipped template if something looks stale:"
    echo "      diff ${DEFAULTS} ${HERE}/etc/default/biped"
else
    install -m 0644 "${HERE}/etc/default/biped" "${DEFAULTS}"
    # Point it at the workspace we actually found, rather than the path that
    # happened to be typed into the template.
    sed -i "s|^BIPED_WS=.*|BIPED_WS=${BIPED_WS}|" "${DEFAULTS}"
    echo "  installed ${DEFAULTS}  (BIPED_WS=${BIPED_WS})"
fi

# --- units ------------------------------------------------------------------
for u in biped-can.service biped-wifi.service biped-wifi.timer; do
    install -m 0644 "${HERE}/systemd/${u}" "${UNITDIR}/${u}"
    echo "  installed ${UNITDIR}/${u}"
done

# biped-stack.service is a TEMPLATE: User=, Group=, WorkingDirectory= and
# Environment=HOME= cannot come from EnvironmentFile= (systemd resolves them
# before it reads that file), so they must be literal in the installed unit.
sed -e "s|@BIPED_USER@|${BIPED_USER}|g" \
    -e "s|@BIPED_WS@|${BIPED_WS}|g" \
    "${HERE}/systemd/biped-stack.service" > "${UNITDIR}/biped-stack.service"
chmod 0644 "${UNITDIR}/biped-stack.service"
echo "  installed ${UNITDIR}/biped-stack.service  (User=${BIPED_USER})"

if grep -q '@BIPED_' "${UNITDIR}/biped-stack.service"; then
    echo "ERROR: unsubstituted @PLACEHOLDER@ left in the installed unit." >&2
    exit 1
fi

systemctl daemon-reload
echo "  systemctl daemon-reload"

# --- enable (opt-in) --------------------------------------------------------
echo
if [ "${DO_ENABLE}" -eq 1 ]; then
    # biped-wifi is NOT enabled here even with --enable. It needs its one-time
    # setup (and a passphrase) first, and enabling it before that would boot
    # into a WiFi decision with no AP profile to fall back to.
    systemctl enable biped-can.service biped-stack.service
    echo "ENABLED: biped-can, biped-stack. Both start at the next boot."
    echo "WiFi is separate — run biped-wifi-setup.sh, then enable biped-wifi.service."
else
    cat <<EOF
Installed but NOT enabled — nothing about your next boot has changed yet.

Test them by hand first (deploy/README.md, TESTs 7-10):

  sudo systemctl start biped-can
  systemctl status biped-can
  ip -brief link show can0          # expect: can0  UP
  candump can0                      # expect heartbeats 001 021 041 061

  sudo systemctl start biped-stack
  journalctl -fu biped-stack
  ros2 topic list                   # from another shell, after sourcing ROS

WiFi is a separate, ONE-TIME setup (it asks for a WPA2 passphrase, which is
stored by NetworkManager and never written into this repo):

  sudo ${LIBDIR}/biped-wifi-setup.sh
  ${LIBDIR}/biped-wifi-mode.sh status

Then, and only then:

  sudo systemctl enable biped-can biped-stack
  sudo systemctl enable biped-wifi.service   # the boot-time WiFi decision
  sudo systemctl enable biped-wifi.timer     # optional: recover if WiFi drops
EOF
fi
