#!/usr/bin/env bash
# =============================================================================
# setup_wifi.sh — Switch RPi WiFi to a new network
# Run this while still connected to the CURRENT network.
# The RPi will reboot and reconnect on the new network.
# =============================================================================
set -euo pipefail

SSID="${1:-}"
PASSWORD="${2:-}"

if [[ -z "$SSID" || -z "$PASSWORD" ]]; then
    echo "Usage: bash setup_wifi.sh <SSID> <password>"
    echo "  e.g. bash setup_wifi.sh 'realme 5i' '123456790'"
    exit 1
fi

echo "Adding WiFi: '$SSID'"

# Try nmcli first (NetworkManager — RPi OS Bookworm/Trixie)
if command -v nmcli &>/dev/null; then
    # Delete existing connection with same SSID if any
    nmcli connection delete "$SSID" 2>/dev/null || true

    nmcli connection add type wifi \
        ssid "$SSID" \
        wifi-sec.key-mgmt wpa-psk \
        wifi-sec.psk "$PASSWORD" \
        connection.autoconnect yes \
        connection.autoconnect-priority 10

    echo "✓ WiFi '$SSID' added via nmcli (priority 10 — will connect on reboot)"

# Fallback: wpa_supplicant
elif [ -f /etc/wpa_supplicant/wpa_supplicant.conf ]; then
    wpa_passphrase "$SSID" "$PASSWORD" | sudo tee -a /etc/wpa_supplicant/wpa_supplicant.conf > /dev/null
    echo "✓ WiFi '$SSID' added via wpa_supplicant"
else
    echo "ERROR: Neither nmcli nor wpa_supplicant found."
    exit 1
fi

echo ""
echo "⚠  RPi will reboot now and connect to '$SSID'."
echo "   SSH in again from a device on the '$SSID' network."
echo ""
sudo reboot
