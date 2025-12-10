#!/bin/bash
# Install udev rules for CAN interface management
# Author: Wesley Cui
# Date: 2025-11-14

set -e

readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;36m'
readonly NC='\033[0m' # No Color

error() {
  echo -e "[${RED}ERROR${NC}] $1" >&2
  exit 1
}

warning() {
  echo -e "[${YELLOW}WARNING${NC}] $1"
}

info() {
  echo -e "[${GREEN}INFO${NC}] $1"
}

title() {
  echo -e "\n${BLUE}========================================${NC}"
  echo -e "${BLUE}$1${NC}"
  echo -e "${BLUE}========================================${NC}\n"
}

# Check if running as root
if [ "$EUID" -ne 0 ]; then
  error "Please run with sudo"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

title "CAN Interface Udev Rules Installation"

# Detect if system has mttcan
HAS_MTTCAN=false
if [ -d "/sys/class/net" ]; then
  for iface in /sys/class/net/*; do
    if [ -L "$iface" ]; then
      DRIVER=$(basename "$(readlink -f "$iface")" | grep -o "mttcan" || true)
      if [ "$DRIVER" = "mttcan" ]; then
        HAS_MTTCAN=true
        MTTCAN_IFACE=$(basename "$iface")
        break
      fi
    fi
  done
fi

# Also check via driver info
if [ "$HAS_MTTCAN" = false ]; then
  for iface in $(ip -br link show type can 2>/dev/null | awk '{print $1}'); do
    DRIVER=$(ethtool -i "$iface" 2>/dev/null | grep -i "driver" | awk '{print $2}' || true)
    if [ "$DRIVER" = "mttcan" ]; then
      HAS_MTTCAN=true
      MTTCAN_IFACE="$iface"
      break
    fi
  done
fi

# Display installation plan
if [ "$HAS_MTTCAN" = true ]; then
  info "Detected Jetson platform with mttcan interface: $MTTCAN_IFACE"
  info "Will install two udev rules:"
  echo "  1. 80-mttcan-rename.rules      - Rename mttcan to can99 (high priority)"
  echo "  2. 90-gs-usb-auto-config.rules - Auto-configure gs_usb devices"
else
  info "No mttcan interface detected (non-Jetson platform)"
  info "Will install one udev rule:"
  echo "  1. 90-gs-usb-auto-config.rules - Auto-configure gs_usb devices"
  warning "Skipping 80-mttcan-rename.rules (not needed on this platform)"
fi
echo ""

# Backup existing rules
info "Backing up existing CAN-related udev rules..."
BACKUP_DIR="/etc/udev/rules.d/backup_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$BACKUP_DIR"

if ls /etc/udev/rules.d/*can*.rules 1> /dev/null 2>&1; then
  cp /etc/udev/rules.d/*can*.rules "$BACKUP_DIR/" || true
  info "Backup saved to: $BACKUP_DIR"
else
  info "No existing CAN rules to backup"
fi

# Install new rules
info "Installing new udev rules..."

# Install mttcan rule only if platform has mttcan
if [ "$HAS_MTTCAN" = true ]; then
  cat > /etc/udev/rules.d/80-mttcan-rename.rules << 'EOF'
# Rule 1: Rename Orin/Jetson built-in mttcan to can99
# This rule has higher priority (80 < 90) and runs first
#
# Purpose: Prevent mttcan from occupying can0, freeing it for USB CAN devices
# Applicable: Jetson Orin, Jetson Xavier, and other devices with mttcan
#
# Author: Wesley Cui
# Date: 2025-11-14

SUBSYSTEM=="net", \
    DRIVERS=="mttcan", \
    KERNEL=="can*", \
    NAME="can99"

EOF
  chmod 644 /etc/udev/rules.d/80-mttcan-rename.rules
  info "Installed: 80-mttcan-rename.rules"
else
  # Remove mttcan rule if it exists (for migration from old installation)
  if [ -f "/etc/udev/rules.d/80-mttcan-rename.rules" ]; then
    mv /etc/udev/rules.d/80-mttcan-rename.rules "$BACKUP_DIR/" 2>/dev/null || true
    info "Removed unnecessary 80-mttcan-rename.rules (moved to backup)"
  fi
fi

# Always install gs_usb rule
cat > /etc/udev/rules.d/90-gs-usb-auto-config.rules << 'EOF'
# Rule 2: Auto-configure all gs_usb CAN devices
# This rule applies to all gs_usb devices (Piper, CANable, etc.)
#
# What it does:
# - Renames first gs_usb device to can0
# - Sets bitrate to 1Mbps (1000000)
# - Brings up the interface automatically
#
# Note: If you have multiple gs_usb devices, consider using serial numbers
# to distinguish them (see commented example below)
#
# Author: Wesley Cui
# Date: 2025-11-14

# Auto-configure first gs_usb device as can0
ACTION=="add", \
    SUBSYSTEM=="net", \
    KERNEL=="can*", \
    DRIVERS=="gs_usb", \
    NAME="can0", \
    RUN+="/bin/sh -c 'sleep 0.5; /sbin/ip link set can0 type can bitrate 1000000; /sbin/ip link set can0 up'"

# Example: Configure specific device by serial number (commented out)
# Uncomment and modify if you need to distinguish multiple gs_usb devices
#
# ACTION=="add", \
#     SUBSYSTEM=="net", \
#     KERNEL=="can*", \
#     ATTRS{idVendor}=="1d50", \
#     ATTRS{idProduct}=="606f", \
#     ATTRS{serial}=="YOUR_SERIAL_HERE", \
#     DRIVERS=="gs_usb", \
#     NAME="piper-can", \
#     RUN+="/bin/sh -c 'sleep 0.5; /sbin/ip link set piper-can type can bitrate 1000000; /sbin/ip link set piper-can up'"

EOF
chmod 644 /etc/udev/rules.d/90-gs-usb-auto-config.rules
info "Installed: 90-gs-usb-auto-config.rules"

info "Rules installed successfully"

# Ask about removing old rules
echo ""
if [ -f "/etc/udev/rules.d/90-can-usb.rules" ]; then
  warning "Found old rule: /etc/udev/rules.d/90-can-usb.rules"
  read -p "Do you want to remove it? (y/n): " -n 1 -r
  echo
  if [[ $REPLY =~ ^[Yy]$ ]]; then
    mv /etc/udev/rules.d/90-can-usb.rules "$BACKUP_DIR/"
    info "Old rule moved to backup directory"
  else
    warning "Old rule kept, but may conflict with new rules"
  fi
fi

# Ask about systemd service
echo ""
if systemctl is-enabled agilex_can_init.service &> /dev/null; then
  warning "Found systemd service: agilex_can_init.service"
  echo "This service is now redundant (udev rules will handle CAN setup)"
  read -p "Do you want to disable it? (y/n): " -n 1 -r
  echo
  if [[ $REPLY =~ ^[Yy]$ ]]; then
    systemctl stop agilex_can_init.service
    systemctl disable agilex_can_init.service
    info "Service disabled (files kept for rollback)"
  else
    warning "Service still enabled, may conflict with udev rules"
  fi
fi

# Reload udev rules
info "Reloading udev rules..."
udevadm control --reload-rules
udevadm trigger --subsystem-match=net

title "Installation Complete!"

info "Next steps:"
echo "  1. Unplug and replug your USB CAN adapter"
echo "  2. Check interfaces: ip link show | grep can"
if [ "$HAS_MTTCAN" = true ]; then
  echo "  3. Expected result:"
  echo "     - can99 (mttcan, state DOWN) - Jetson built-in"
  echo "     - can0 (state UP, bitrate 1000000) - USB CAN"
else
  echo "  3. Expected result:"
  echo "     - can0 (state UP, bitrate 1000000) - USB CAN"
fi
echo ""

info "To test with Piper:"
echo "  ros2 launch piper_hardware ..."
echo ""

info "To rollback (if something goes wrong):"
echo "  sudo cp $BACKUP_DIR/*.rules /etc/udev/rules.d/"
echo "  sudo systemctl enable agilex_can_init.service"
echo "  sudo systemctl start agilex_can_init.service"
echo "  sudo udevadm control --reload-rules"
echo ""

info "For troubleshooting, check:"
echo "  journalctl -b | grep -i can"
echo "  dmesg | grep -i can"
echo ""

