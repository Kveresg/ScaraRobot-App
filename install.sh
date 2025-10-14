#!/bin/bash
set -e

APP_NAME="ScaraRobotApp"
INSTALL_DIR="/usr/local/bin"
SERVICE_FILE="/etc/systemd/system/${APP_NAME}.service"
EXECUTABLE="dist/${APP_NAME}"

# -------------------------------
# Helper Functions
# -------------------------------

detect_pkg_manager() {
    if command -v apt &>/dev/null; then
        echo "apt"
    elif command -v dnf &>/dev/null; then
        echo "dnf"
    elif command -v yum &>/dev/null; then
        echo "yum"
    elif command -v pacman &>/dev/null; then
        echo "pacman"
    elif command -v zypper &>/dev/null; then
        echo "zypper"
    else
        echo ""
    fi
}

install_python() {
    PKG_MANAGER=$(detect_pkg_manager)
    if [ -z "$PKG_MANAGER" ]; then
        echo "⚠️  No supported package manager found."
        echo "Please install Python 3 manually and re-run this installer."
        exit 1
    fi

    echo "Installing Python 3 using $PKG_MANAGER..."
    case $PKG_MANAGER in
        apt) sudo apt update && sudo apt install -y python3 ;;
        dnf) sudo dnf install -y python3 ;;
        yum) sudo yum install -y python3 ;;
        pacman) sudo pacman -Sy --noconfirm python ;;
        zypper) sudo zypper install -y python3 ;;
    esac
}

ask_yes_no() {
    local prompt="$1"
    local default="$2"
    local response
    while true; do
        read -p "$prompt [y/n] (default: $default): " response
        response="${response:-$default}"
        case "$response" in
            [Yy]*) return 0 ;;
            [Nn]*) return 1 ;;
            *) echo "Please answer y or n." ;;
        esac
    done
}

create_service_file() {
    local username
    username=$(logname 2>/dev/null || echo "$USER")
    cat <<EOF | sudo tee "$SERVICE_FILE" >/dev/null
[Unit]
Description=$APP_NAME Background Service
After=network.target

[Service]
ExecStart=$INSTALL_DIR/$APP_NAME
Restart=always
User=$username
WorkingDirectory=$INSTALL_DIR

[Install]
WantedBy=multi-user.target
EOF
}

# -------------------------------
# Installation Process
# -------------------------------

echo "🛠️  Starting $APP_NAME installation..."
sleep 0.5

# 1️⃣ Check Python
if ! command -v python3 &>/dev/null; then
    echo "❌ Python 3 not found."
    if ask_yes_no "Would you like to install Python 3 automatically?" "y"; then
        install_python
    else
        echo "Installation aborted — Python 3 is required."
        exit 1
    fi
else
    echo "✅ Python 3 detected: $(python3 -V)"
fi

# 2️⃣ Copy executable
echo "📦 Installing executable..."
if [ ! -f "$EXECUTABLE" ]; then
    echo "❌ Executable not found at $EXECUTABLE"
    echo "Please build your app first (e.g., with PyInstaller)."
    exit 1
fi

sudo cp "$EXECUTABLE" "$INSTALL_DIR/$APP_NAME"
sudo chmod +x "$INSTALL_DIR/$APP_NAME"

# 3️⃣ Optional systemd service
if ask_yes_no "Would you like to install $APP_NAME as a systemd service?" "y"; then
    echo "🧩 Creating systemd service..."
    create_service_file
    sudo systemctl daemon-reload

    if ask_yes_no "Enable and start the service automatically?" "y"; then
        sudo systemctl enable "$APP_NAME"
        sudo systemctl start "$APP_NAME"
        echo "✅ Service started and enabled."
    else
        echo "⚙️  Service installed but not started or enabled."
    fi
else
    echo "⏩ Skipping service installation."
fi

echo ""
echo "🎉 $APP_NAME installation complete!"
echo "Executable: $INSTALL_DIR/$APP_NAME"
echo "Service file: $SERVICE_FILE (if installed)"
echo "You can manage the service with: sudo systemctl {start|stop|status|enable|disable} $APP_NAME"
echo "Thank you for using $APP_NAME!"