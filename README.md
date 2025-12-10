# Piper 机械臂开机自启动初始化程序

## 功能

- 开机后自动连接 CAN 总线
- 使能电机
- 执行回零（Homing）
- 完成后退出，由 ROS2 或其他程序接管控制

## 文件结构

```
├── piper_boot_init.cpp      # 主程序
├── piper_boot_init.service  # systemd 服务文件
├── deploy.sh                # 一键部署脚本
├── install_udev_rules.sh    # CAN udev 规则安装
├── CMakeLists.txt           # CMake 配置
├── include/                 # SDK 头文件
└── lib/                     # SDK 库文件 (aarch64/x86_64)
```

## 部署

```bash
sudo ./deploy.sh
```

## 管理

```bash
# 状态
sudo systemctl status piper_boot_init.service

# 日志
journalctl -u piper_boot_init.service -f

# 禁用
sudo systemctl disable piper_boot_init.service
```

## 手动编译

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

