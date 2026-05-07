#!/usr/bin/env bash
# =============================================================================
# install_rpi.sh — Minimal RPi setup (NO ROS 2 needed!)
# Installs: roslibpy + adafruit PCA9685 libs + systemd service
# Time: ~2 minutes (vs 20+ min for ROS 2)
# =============================================================================
set -euo pipefail

REPO_URL="https://github.com/Raj-49/dexter_rpi_bridge.git"
REPO_DIR="$HOME/dexter_rpi_bridge"
SERVICE_NAME="dexter-rpi-bridge"
SERVICE_FILE="$REPO_DIR/systemd/$SERVICE_NAME.service"
ENV_FILE="$HOME/dexter_bridge.env"

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║     Dexter RPi Bridge — Minimal Installer           ║"
echo "║     No ROS 2 required! Just pip + Python.           ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "  User: $(whoami) | OS: $(lsb_release -sd) | Arch: $(uname -m)"
echo ""

# ── Step 1: System deps ───────────────────────────────────────────────────────
echo "[1/5] Installing system dependencies..."
sudo apt-get update -qq
sudo apt-get install -y -qq python3-pip i2c-tools git curl
echo "  Done."

# ── Step 2: Enable I2C @ 400kHz Fast Mode ────────────────────────────────────
echo "[2/5] Enabling I2C at 400kHz Fast Mode..."
for cfg in /boot/firmware/config.txt /boot/config.txt; do
    if [ -f "$cfg" ]; then
        # Enable I2C
        grep -q "^dtparam=i2c_arm=on" "$cfg" || \
            echo "dtparam=i2c_arm=on" | sudo tee -a "$cfg" > /dev/null
        # Set 400kHz Fast Mode (4x faster I2C — reduces 14-servo write from 6.3ms to 1.6ms)
        # PCA9685 and all RPi3/4/5 fully support 400kHz
        if grep -q "i2c_arm_baudrate" "$cfg"; then
            # Update existing baudrate line to 400000
            sudo sed -i 's/dtparam=i2c_arm_baudrate=[0-9]*/dtparam=i2c_arm_baudrate=400000/' "$cfg"
        else
            echo "dtparam=i2c_arm_baudrate=400000" | sudo tee -a "$cfg" > /dev/null
        fi
        echo "  I2C config written to $cfg"
        break
    fi
done
grep -q "^i2c-dev" /etc/modules 2>/dev/null || echo "i2c-dev" | sudo tee -a /etc/modules > /dev/null
sudo modprobe i2c-dev 2>/dev/null || true
echo "  I2C enabled @ 400kHz (reboot required to activate speed change)."

# ── Step 3: Python libraries ──────────────────────────────────────────────────
echo "[3/5] Installing Python libraries (roslibpy + PCA9685)..."
pip3 install --break-system-packages \
    roslibpy \
    adafruit-circuitpython-pca9685 \
    adafruit-blinka \
    smbus2
echo "  Libraries installed."

# ── Step 4: Clone / update the bridge code ────────────────────────────────────
echo "[4/5] Setting up bridge code..."
if [ -d "$REPO_DIR/.git" ]; then
    echo "  Repo exists — pulling latest..."
    git -C "$REPO_DIR" pull
else
    git clone "$REPO_URL" "$REPO_DIR"
fi
echo "  Code ready at $REPO_DIR"

# ── Step 5: Create config file + install service ──────────────────────────────
echo "[5/5] Installing systemd service..."

# Create env file if it doesn't exist
if [ ! -f "$ENV_FILE" ]; then
    cat > "$ENV_FILE" << 'ENVEOF'
# Dexter Bridge Configuration
# Set LAPTOP_IP to the IP of the machine running rosbridge_server
# Find laptop IP with: hostname -I  (run on the laptop)
LAPTOP_IP=
ROSBRIDGE_PORT=9090
ENVEOF
    echo "  ⚠  Config file created at $ENV_FILE"
    echo "  ⚠  YOU MUST SET LAPTOP_IP before starting the service!"
    echo "  ⚠  Run: nano $ENV_FILE"
fi

# Install and enable service
sudo cp "$SERVICE_FILE" "/etc/systemd/system/$SERVICE_NAME.service"
sudo systemctl daemon-reload
sudo systemctl enable "$SERVICE_NAME"
echo "  Service installed and enabled."

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  Installation Complete!                             ║"
echo "╠══════════════════════════════════════════════════════╣"
echo "║  NEXT STEPS:                                        ║"
echo "║  1. Find laptop IP on this WiFi: hostname -I        ║"
echo "║     (run this on the LAPTOP, not RPi)               ║"
echo "║  2. Set it: nano ~/dexter_bridge.env                ║"
echo "║     Add: LAPTOP_IP=<your_laptop_ip>                 ║"
echo "║  3. Test I2C: python3 $REPO_DIR/scripts/i2c_test.py ║"
echo "║  4. Start: sudo systemctl start dexter-rpi-bridge   ║"
echo "║  5. Logs:  journalctl -u dexter-rpi-bridge -f       ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "⚠  Reboot recommended to fully activate I2C."
read -rp "Reboot now? [y/N]: " ans
[[ "$ans" =~ ^[Yy]$ ]] && sudo reboot || true
