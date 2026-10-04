#!/bin/bash
# Shared helpers for the hardware tests.
#
# WARNING: every script that sources this DISCONNECTS your Wi-Fi, puts the
# interface into monitor mode, and needs root. It always restores managed mode
# and re-associates on exit (trap), but do not run these over a remote session.
#
#   IFACE=wlp3s0 CON="My Network" ./monitor_root_test.sh
#
# CON is auto-detected from the active NetworkManager profile when omitted.

IFACE="${IFACE:-wlp3s0}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"          # project root (holds the radar)
FIXTURES="$HERE/../fixtures"
OUT="${OUT:-$HERE/out}"
export RSSI_RADAR_ROOT="$ROOT"
export RSSI_RADAR_IFACE="$IFACE"

if [ -z "${CON:-}" ]; then
  CON="$(nmcli -t -f NAME,DEVICE con show --active 2>/dev/null \
         | awk -F: -v d="$IFACE" '$2==d {print $1; exit}')"
fi
if [ -z "${CON:-}" ]; then
  echo "ERROR: could not detect the active connection for $IFACE." >&2
  echo "       Re-run with:  CON=\"<profile name>\" $0" >&2
  exit 1
fi

mkdir -p "$OUT"

# Bring the card back to a usable state and re-associate. Safe to call twice.
restore_iface() {
  echo "=== RESTORING $IFACE ==="
  sudo -n nmcli device set "$IFACE" managed yes >/dev/null 2>&1
  sudo -n ip link set "$IFACE" down                 >/dev/null 2>&1
  sudo -n iw  dev "$IFACE" set type managed         >/dev/null 2>&1
  sudo -n ip link set "$IFACE" up                   >/dev/null 2>&1
  sudo -n nmcli connection up "$CON"                >/dev/null 2>&1
  local i
  for i in $(seq 1 20); do
    if iw dev "$IFACE" link 2>/dev/null | grep -q "Connected to"; then
      echo "    re-associated after ${i}s"
      iw dev "$IFACE" link | sed -n '2p' | sed 's/^/    /'
      echo "    NM-MANAGED: $(nmcli -g GENERAL.NM-MANAGED dev show "$IFACE" 2>/dev/null)"
      echo "    type: $(iw dev "$IFACE" info 2>/dev/null | awk '/type/{print $2}')"
      return 0
    fi
    sleep 1
  done
  echo "    !!! FAILED TO REASSOCIATE — reconnect manually" >&2
  return 1
}

require_root() {
  sudo -n true 2>/dev/null || {
    echo "ERROR: these tests need passwordless sudo for ip/iw/nmcli." >&2
    exit 1
  }
}