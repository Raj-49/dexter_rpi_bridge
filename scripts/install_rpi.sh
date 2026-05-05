#!/usr/bin/env bash
# =============================================================================
# install_rpi.sh — Works on Ubuntu 24.04 (apt) AND Raspberry Pi OS/Debian (robostack)
# =============================================================================
set -euo pipefail

REPO_URL="https://github.com/Raj-49/dexter_rpi_bridge.git"
WS_DIR="$HOME/dexter_rpi_ws"
SRC_DIR="$WS_DIR/src"
SERVICE_NAME="dexter-rpi-bridge"
START_SCRIPT="$HOME/start_dexter_bridge.sh"

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║        Dexter RPi Bridge — One-Shot Installer        ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

OS_ID=$(lsb_release -si 2>/dev/null | tr '[:upper:]' '[:lower:]' || echo "unknown")
OS_CODENAME=$(lsb_release -sc 2>/dev/null | tr '[:upper:]' '[:lower:]' || echo "unknown")
ARCH=$(uname -m)
echo "  OS: $OS_ID ($OS_CODENAME) | Arch: $ARCH"
echo ""

# ── Step 1: System update ─────────────────────────────────────────────────────
echo "[1/7] Updating system packages..."
sudo apt-get update -qq
sudo apt-get upgrade -y -qq
sudo apt-get install -y -qq curl gnupg lsb-release git python3-pip i2c-tools build-essential
echo "  Done."

# ── Step 2: Enable I2C ────────────────────────────────────────────────────────
echo "[2/7] Enabling I2C interface..."
for cfg in /boot/firmware/config.txt /boot/config.txt; do
    if [ -f "$cfg" ]; then
        grep -q "^dtparam=i2c_arm=on" "$cfg" || echo "dtparam=i2c_arm=on" | sudo tee -a "$cfg"
        break
    fi
done
grep -q "^i2c-dev" /etc/modules 2>/dev/null || echo "i2c-dev" | sudo tee -a /etc/modules
sudo modprobe i2c-dev 2>/dev/null || true
echo "  I2C enabled."

# ── Step 3: Install ROS 2 ─────────────────────────────────────────────────────
if [[ "$OS_ID" == "ubuntu" && "$OS_CODENAME" == "noble" ]]; then
    echo "[3/7] Installing ROS 2 Jazzy via apt (Ubuntu 24.04)..."
    if [ ! -f /opt/ros/jazzy/setup.bash ]; then
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
            -o /usr/share/keyrings/ros-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
            http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
            | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
        sudo apt-get update -qq
        sudo apt-get install -y -qq ros-jazzy-ros-base python3-colcon-common-extensions
    fi
    grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc 2>/dev/null || \
        echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

    # Write start-script for systemd
    cat > "$START_SCRIPT" << 'EOF'
#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
source $HOME/dexter_rpi_ws/install/setup.bash
exec ros2 run dexter_rpi_bridge hardware_node
EOF

else
    echo "[3/7] Raspberry Pi OS / Debian detected — installing ROS 2 Jazzy via robostack..."
    MAMBA_ARCH="linux-aarch64"
    [[ "$ARCH" == "x86_64" ]] && MAMBA_ARCH="linux-64"

    if [ ! -f "$HOME/bin/micromamba" ]; then
        echo "  Downloading micromamba ($MAMBA_ARCH)..."
        mkdir -p "$HOME/bin"
        curl -Ls "https://micro.mamba.pm/api/micromamba/$MAMBA_ARCH/latest" \
            | tar -xvj -C /tmp bin/micromamba
        mv /tmp/bin/micromamba "$HOME/bin/micromamba"
        chmod +x "$HOME/bin/micromamba"
    fi

    "$HOME/bin/micromamba" shell init --shell bash --root-prefix "$HOME/micromamba" 2>/dev/null || true
    # shellcheck disable=SC1090
    source ~/.bashrc 2>/dev/null || true

    if ! "$HOME/bin/micromamba" env list 2>/dev/null | grep -q "ros2_jazzy"; then
        echo "  Creating ros2_jazzy environment (this takes 10-20 min on first run)..."
        "$HOME/bin/micromamba" create -n ros2_jazzy \
            -c robostack-staging \
            -c conda-forge \
            ros-jazzy-ros-base \
            colcon-common-extensions \
            python=3.12 \
            --yes
    else
        echo "  ros2_jazzy environment already exists."
    fi

    # Write start-script for systemd
    MAMBA_BIN="$HOME/bin/micromamba"
    cat > "$START_SCRIPT" << EOF
#!/usr/bin/env bash
eval "\$($MAMBA_BIN shell hook --shell bash)"
micromamba activate ros2_jazzy
source $WS_DIR/install/setup.bash
exec ros2 run dexter_rpi_bridge hardware_node
EOF
fi

chmod +x "$START_SCRIPT"
echo "  Start script written to $START_SCRIPT"

# ── Step 4: Python hardware libraries ─────────────────────────────────────────
echo "[4/7] Installing Python I2C + PCA9685 libraries..."
if [[ "$OS_ID" == "ubuntu" && "$OS_CODENAME" == "noble" ]]; then
    pip3 install --break-system-packages adafruit-circuitpython-pca9685 adafruit-blinka smbus2
else
    eval "$("$HOME/bin/micromamba" shell hook --shell bash)"
    micromamba activate ros2_jazzy
    pip install adafruit-circuitpython-pca9685 adafruit-blinka smbus2
fi
echo "  Python libraries installed."

# ── Step 5: Clone + build workspace ──────────────────────────────────────────
echo "[5/7] Setting up ROS 2 workspace..."
mkdir -p "$SRC_DIR"
if [ -d "$SRC_DIR/dexter_rpi_bridge" ]; then
    echo "  Repo exists — pulling latest..."
    git -C "$SRC_DIR/dexter_rpi_bridge" pull
else
    git clone "$REPO_URL" "$SRC_DIR/dexter_rpi_bridge"
fi

echo "  Building package..."
if [[ "$OS_ID" == "ubuntu" && "$OS_CODENAME" == "noble" ]]; then
    (source /opt/ros/jazzy/setup.bash && cd "$WS_DIR" && \
        colcon build --symlink-install --packages-select dexter_rpi_bridge)
else
    (eval "$("$HOME/bin/micromamba" shell hook --shell bash)" && \
        micromamba activate ros2_jazzy && \
        cd "$WS_DIR" && \
        colcon build --symlink-install --packages-select dexter_rpi_bridge)
fi

grep -q "source $WS_DIR/install/setup.bash" ~/.bashrc 2>/dev/null || \
    echo "source $WS_DIR/install/setup.bash" >> ~/.bashrc
echo "  Workspace built."

# ── Step 6: Install systemd service ──────────────────────────────────────────
echo "[6/7] Installing systemd service..."
sudo cp "$SRC_DIR/dexter_rpi_bridge/systemd/$SERVICE_NAME.service" \
        "/etc/systemd/system/$SERVICE_NAME.service"
sudo systemctl daemon-reload
sudo systemctl enable "$SERVICE_NAME"
echo "  Service enabled."

# ── Step 7: Start the service ─────────────────────────────────────────────────
echo "[7/7] Starting bridge service..."
sudo systemctl restart "$SERVICE_NAME"
sleep 3
if sudo systemctl is-active --quiet "$SERVICE_NAME"; then
    echo "  ✓ Service is RUNNING"
else
    echo "  ✗ Service failed — check: journalctl -u $SERVICE_NAME -n 30"
fi

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  Installation Complete! Reboot recommended for I2C. ║"
echo "╚══════════════════════════════════════════════════════╝"
read -rp "Reboot now? [y/N]: " ans
[[ "$ans" =~ ^[Yy]$ ]] && sudo reboot || true
