#!/bin/bash
APP_NAME="ScaraRobotApp"
INSTALL_DIR="/usr/local/bin"
SERVICE_FILE="/etc/systemd/system/${APP_NAME}.service"

echo "🧹 Uninstalling $APP_NAME..."

if systemctl list-units --full -all | grep -q "$APP_NAME.service"; then
    sudo systemctl stop "$APP_NAME" 2>/dev/null || true
    sudo systemctl disable "$APP_NAME" 2>/dev/null || true
    sudo rm -f "$SERVICE_FILE"
    sudo systemctl daemon-reload
fi

sudo rm -f "$INSTALL_DIR/$APP_NAME"

echo "✅ Uninstalled successfully."