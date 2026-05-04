#!/usr/bin/env bash
# =============================================================================
# install_rpi.sh
# =============================================================================
# One-shot setup script. Run this on a FRESH Raspberry Pi OS Lite (64-bit)
# installation via SSH.
#
# What it does:
#   1. Updates the OS
#   2. Enables I2C hardware interface
#   3. Installs ROS 2 Jazzy (base, no desktop)
#   4. Installs Python I2C + PCA9685 libraries
#   5. Clones this repo and builds the ROS 2 workspace
#   6. Installs the systemd auto-start service
#
# Usage:
#   ssh pi@dexter-rpi.local
#   bash <(curl -fsSL https://raw.githubusercontent.com/Raj-49/dexter_rpi_bridge/main/scripts/install_rpi.sh)
#
# Or copy and run manually:
#   chmod +x scripts/install_rpi.sh && ./scripts/install_rpi.sh
# =============================================================================

set -euo pipefail

REPO_URL="https://github.com/Raj-49/dexter_rpi_bridge.git"
WS_DIR="/home/pi/dexter_rpi_ws"
SRC_DIR="$WS_DIR/src"
SERVICE_NAME="dexter-rpi-bridge"

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║        Dexter RPi Bridge — One-Shot Installer        ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

# ── Step 1: System update ─────────────────────────────────────────────────────
echo "[1/6] Updating system packages..."
sudo apt-get update -qq
sudo apt-get upgrade -y -qq
sudo apt-get install -y -qq \
    curl gnupg lsb-release git python3-pip \
    i2c-tools build-essential

# ── Step 2: Enable I2C ───────────────────────────────────────────────────────
echo "[2/6] Enabling I2C interface..."
# Enable i2c-dev without interactive raspi-config
if ! grep -q "^dtparam=i2c_arm=on" /boot/firmware/config.txt 2>/dev/null; then
    echo "dtparam=i2c_arm=on" | sudo tee -a /boot/firmware/config.txt
fi
if ! grep -q "^i2c-dev" /etc/modules 2>/dev/null; then
    echo "i2c-dev" | sudo tee -a /etc/modules
fi
sudo modprobe i2c-dev 2>/dev/null || true
echo "  I2C enabled (effective after reboot)."

# ── Step 3: Install ROS 2 Jazzy ───────────────────────────────────────────────
echo "[3/6] Installing ROS 2 Jazzy..."
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) \
        signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt-get update -qq
    sudo apt-get install -y -qq ros-jazzy-ros-base python3-colcon-common-extensions
    echo "  ROS 2 Jazzy installed."
else
    echo "  ROS 2 Jazzy already installed — skipping."
fi

# Add ROS 2 source to .bashrc
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
fi

# ── Step 4: Install Python hardware libraries ─────────────────────────────────
echo "[4/6] Installing Python I2C + PCA9685 libraries..."
pip3 install --break-system-packages \
    adafruit-circuitpython-pca9685 \
    adafruit-blinka \
    smbus2
echo "  Python libraries installed."

# ── Step 5: Clone repo + build workspace ─────────────────────────────────────
echo "[5/6] Setting up ROS 2 workspace..."
mkdir -p "$SRC_DIR"

if [ -d "$SRC_DIR/dexter_rpi_bridge" ]; then
    echo "  Repo already cloned — pulling latest..."
    git -C "$SRC_DIR/dexter_rpi_bridge" pull
else
    git clone "$REPO_URL" "$SRC_DIR/dexter_rpi_bridge"
fi

# Build the workspace
source /opt/ros/jazzy/setup.bash
cd "$WS_DIR"
colcon build --symlink-install --packages-select dexter_rpi_bridge

# Add workspace to .bashrc
if ! grep -q "source $WS_DIR/install/setup.bash" ~/.bashrc; then
    echo "source $WS_DIR/install/setup.bash" >> ~/.bashrc
fi
echo "  Workspace built at $WS_DIR"

# ── Step 6: Install systemd service ──────────────────────────────────────────
echo "[6/6] Installing systemd service..."
sudo cp "$SRC_DIR/dexter_rpi_bridge/systemd/$SERVICE_NAME.service" \
        "/etc/systemd/system/$SERVICE_NAME.service"
sudo systemctl daemon-reload
sudo systemctl enable "$SERVICE_NAME"
echo "  Service '$SERVICE_NAME' installed and enabled."

# ── Done ─────────────────────────────────────────────────────────────────────
echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║              Installation Complete!                  ║"
echo "╠══════════════════════════════════════════════════════╣"
echo "║  Next steps:                                         ║"
echo "║  1. Wire PCA9685 boards (see README.md)              ║"
echo "║  2. sudo reboot                                      ║"
echo "║  3. After reboot, run: python3 scripts/i2c_test.py   ║"
echo "║  4. Start service: sudo systemctl start $SERVICE_NAME║"
echo "║  5. Check logs: journalctl -u $SERVICE_NAME -f       ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "⚠  A REBOOT IS REQUIRED to activate I2C."
read -rp "Reboot now? [y/N]: " ans
if [[ "$ans" =~ ^[Yy]$ ]]; then
    sudo reboot
fi
