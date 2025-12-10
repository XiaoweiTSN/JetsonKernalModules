#!/bin/bash
# Piper 机械臂开机自启动一键部署脚本
# Author: Wesley Cui
# Date: 2025-12-10
#
# 功能：
#   1. 编译 piper_boot_init 二进制文件
#   2. 复制二进制文件到 /opt/piper_tools/bin/
#   3. 复制 service 文件到 /etc/systemd/system/
#   4. reload 并 enable service
#   5. 执行 install_udev_rules.sh

set -e

readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;36m'
readonly NC='\033[0m'

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

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 检查是否以 root 权限运行
check_root() {
    if [ "$EUID" -ne 0 ]; then
        error "Please run with sudo"
    fi
}

# 步骤1: 编译
build_binary() {
    title "Step 1/5: Build piper_boot_init"
    
    cd "$SCRIPT_DIR"
    
    # 创建 build 目录
    if [ -d "build" ]; then
        info "Cleaning old build directory..."
        rm -rf build
    fi
    
    mkdir -p build
    cd build
    
    info "Running cmake..."
    cmake .. -DCMAKE_BUILD_TYPE=Release
    
    info "Running make..."
    make -j$(nproc)
    
    if [ ! -f "piper_boot_init" ]; then
        error "Build failed: piper_boot_init not found"
    fi
    
    info "Build successful!"
    cd "$SCRIPT_DIR"
}

# 步骤2: 安装二进制文件
install_binary() {
    title "Step 2/5: Install binary"
    
    local BIN_DIR="/opt/piper_tools/bin"
    
    info "Creating directory $BIN_DIR..."
    mkdir -p "$BIN_DIR"
    
    info "Copying binary..."
    cp "$SCRIPT_DIR/build/piper_boot_init" "$BIN_DIR/"
    chmod +x "$BIN_DIR/piper_boot_init"
    
    info "Binary installed to $BIN_DIR/piper_boot_init"
}

# 步骤3: 安装 service 文件
install_service() {
    title "Step 3/5: Install systemd service"
    
    local SERVICE_FILE="$SCRIPT_DIR/piper_boot_init.service"
    local TARGET_DIR="/etc/systemd/system"
    
    if [ ! -f "$SERVICE_FILE" ]; then
        error "Service file not found: $SERVICE_FILE"
    fi
    
    info "Copying service file to $TARGET_DIR..."
    cp "$SERVICE_FILE" "$TARGET_DIR/"
    chmod 644 "$TARGET_DIR/piper_boot_init.service"
    
    info "Service file installed"
}

# 步骤4: reload 并 enable service
enable_service() {
    title "Step 4/5: Enable systemd service"
    
    info "Reloading systemd configuration..."
    systemctl daemon-reload
    
    info "Enabling piper_boot_init.service..."
    systemctl enable piper_boot_init.service
    
    info "Service enabled (will start automatically on next boot)"
    
    # 显示 service 状态
    echo ""
    systemctl status piper_boot_init.service --no-pager || true
}

# 步骤5: 安装 udev 规则
install_udev() {
    title "Step 5/5: Install udev rules"
    
    local UDEV_SCRIPT="$SCRIPT_DIR/install_udev_rules.sh"
    
    if [ ! -f "$UDEV_SCRIPT" ]; then
        error "Udev script not found: $UDEV_SCRIPT"
    fi
    
    info "Running install_udev_rules.sh..."
    chmod +x "$UDEV_SCRIPT"
    bash "$UDEV_SCRIPT"
}

# 显示完成信息
show_summary() {
    title "Deployment Complete!"
    
    info "Completed:"
    echo "  - Built piper_boot_init"
    echo "  - Installed to /opt/piper_tools/bin/piper_boot_init"
    echo "  - Installed systemd service"
    echo "  - Enabled auto-start on boot"
    echo "  - Installed CAN udev rules"
    echo ""
    
    info "View logs:"
    echo "  journalctl -u piper_boot_init.service -f"
    echo ""
    
    info "Manual test (optional):"
    echo "  sudo /opt/piper_tools/bin/piper_boot_init"
    echo ""
    
    info "Manage service:"
    echo "  sudo systemctl status piper_boot_init.service"
    echo "  sudo systemctl start piper_boot_init.service"
    echo "  sudo systemctl stop piper_boot_init.service"
    echo "  sudo systemctl disable piper_boot_init.service"
    echo ""
}

# 主流程
main() {
    title "Piper Boot Init Deployment Script"
    
    check_root
    build_binary
    install_binary
    install_service
    enable_service
    install_udev
    show_summary
}

main "$@"
