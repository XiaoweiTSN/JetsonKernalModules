# Jetson Orin 自定义内核模块安装包

> **本仓库包含两个分支：**
>
> | 分支 | 功能 | 链接 |
> |------|------|------|
> | **`main`**（当前） | 内核模块安装（RealSense、CAN、CH341 串口） | — |
> | **`piper-auto-start`** | Piper 机械臂开机自动上电回零（systemd 服务） | [查看分支](https://github.com/XiaoweiTSN/JetsonKernalModules/tree/piper-auto-start) |

---

## 概述

本脚本用于在 **Jetson Orin + JetPack 6** 系统中安装自定义编译的内核模块。支持以下内核版本的自动检测与安装：

- **`5.15.148-tegra`** — JetPack 6.1 的 Linux R36.4.3 普通内核
- **`5.15.148-rt-tegra`** — `5.15.148-tegra` 对应的实时内核版本
- **`5.15.185-tegra`** — JetPack 6.2.2 的 Linux R36.5.0 普通内核
- **`5.15.199-tegra`** — JetPack 6.2.3 的 Linux R36.5.2 普通内核

如果系统同时安装了 `5.15.148-tegra` 和 `5.15.148-rt-tegra` 两套内核，脚本会一次性给两套 `/lib/modules/` 都安装对应模块，并分别执行 `depmod`。模块加载只针对当前正在运行的内核执行。

`5.15.148-tegra`、`5.15.148-rt-tegra`、`5.15.185-tegra` 和 `5.15.199-tegra` 目录均包含 **8 个内核模块**。

### RealSense 相机支持（6 个模块）

| 模块文件名 | 功能描述 | 安装路径（相对于 `/lib/modules/<kernel>/kernel/`） |
|---|---|---|
| `uvcvideo.ko` | USB 视频类驱动，已修改以支持 RealSense 深度格式和元数据 | `drivers/media/usb/uvc/` |
| `hid-sensor-accel-3d.ko` | RealSense 加速度传感器驱动 | `drivers/iio/accel/` |
| `hid-sensor-gyro-3d.ko` | RealSense 陀螺仪传感器驱动 | `drivers/iio/gyro/` |
| `hid-sensor-iio-common.ko` | HID 传感器 IIO 公共支持模块 | `drivers/iio/common/hid-sensors/` |
| `hid-sensor-hub.ko` | HID Sensor Hub 驱动，协调多个传感器 | `drivers/hid/` |
| `hid-sensor-trigger.ko` | 传感器触发器机制支持 | `drivers/iio/common/hid-sensors/` |

### USB-CAN 通信（1 个模块）

| 模块文件名 | 功能描述 | 安装路径 |
|---|---|---|
| `gs_usb.ko` | USB-CAN 通信驱动，适用于 Piper 机械臂等 CAN 设备 | `drivers/net/can/usb/` |

### USB 串口驱动（1 个模块）

JetPack 6 系统已内置 `usbserial`、`cp210x`、`ftdi_sio`、`cdc-acm` 等常用串口驱动，唯独缺少 CH341：

| 模块文件名 | 功能描述 | 常见硬件 | 安装路径 |
|---|---|---|---|
| `ch341.ko` | CH340/CH341 USB 转串口驱动 | 廉价 USB-TTL 转换器、Arduino Nano 克隆版 | `drivers/usb/serial/` |

---

## 环境要求

- Jetson Orin 系列（Orin NX / Orin Nano / AGX Orin）
- JetPack 6.x 系统
- 内核版本为 `5.15.148-tegra`、`5.15.148-rt-tegra`、`5.15.185-tegra` 或 `5.15.199-tegra`
- 执行脚本时需具有 `sudo` 权限

---

## 使用方法

### 1. 校验文件完整性与解压

```bash
sha256sum -c install-modules.tar.gz.sha256
# 正确结果应为：install-modules.tar.gz: OK
```

解压：

```bash
tar -xzf install-modules.tar.gz
cd install-modules
```

### 2. 执行安装脚本

```bash
sudo ./install-jetson-modules.sh
```

脚本将自动执行以下步骤：

1. 通过 `uname -r` 检测当前内核版本；
2. 自动选择对应版本的模块目录（`5.15.148-tegra/`、`5.15.148-rt-tegra/`、`5.15.185-tegra/` 或 `5.15.199-tegra/`）；
3. 如果系统同时存在 `5.15.148-tegra` 和 `5.15.148-rt-tegra`，同时安装两套模块；
4. 检查所有模块文件完整性；
5. 将模块拷贝至系统目录 `/lib/modules/<kernel>/kernel/...`；
6. 对每套已安装模块运行 `depmod` 更新模块依赖关系；
7. 按正确的依赖顺序加载当前运行内核的模块；
8. 输出安装结果摘要。

### 3. 重启并验证

```bash
sudo reboot
```

重启后验证模块是否生效：

```bash
lsmod | grep -E 'uvcvideo|hid_sensor|gs_usb|ch341'
```

验证 RealSense 相机：

```bash
realsense-viewer
```

验证串口设备识别（插入 USB 串口设备后）：

```bash
ls /dev/ttyUSB* /dev/ttyACM*
dmesg | tail -20
```

---

## 包内目录结构

```
install-modules/
├── install-jetson-modules.sh    # 自动检测安装脚本
├── 5.15.148-tegra/              # JetPack 6.1 的 Linux R36.4.3 普通内核模块
│   ├── uvcvideo.ko
│   ├── hid-sensor-accel-3d.ko
│   ├── hid-sensor-gyro-3d.ko
│   ├── hid-sensor-iio-common.ko
│   ├── hid-sensor-hub.ko
│   ├── hid-sensor-trigger.ko
│   ├── gs_usb.ko
│   └── ch341.ko
├── 5.15.148-rt-tegra/           # 5.15.148 RT 内核模块
│   ├── uvcvideo.ko
│   ├── hid-sensor-accel-3d.ko
│   ├── hid-sensor-gyro-3d.ko
│   ├── hid-sensor-iio-common.ko
│   ├── hid-sensor-hub.ko
│   ├── hid-sensor-trigger.ko
│   ├── gs_usb.ko
│   └── ch341.ko
├── 5.15.185-tegra/              # JetPack 6.2.2 的 Linux R36.5.0 普通内核模块
│   ├── (同上，共 8 个模块)
│   └── ...
└── 5.15.199-tegra/             # JetPack 6.2.3 的 Linux R36.5.2 普通内核模块
    ├── (同上，共 8 个模块)
    └── ...
```

仓库根目录还保留了 `rt-extra-modules-5.15.148-rt-tegra.tar.gz`，里面是可能用到的 RT 备用模块。当前安装脚本只使用 `install-modules/5.15.148-rt-tegra/` 里的 8 个常用模块，不会自动解压或安装这个备用压缩包；如果以后需要补充其他 RT 模块，可以手动解压后从中取用。

> 说明：`install-modules/` 下的目录是脚本直接安装时读取的“平铺模块目录”；而 `extra-modules-5.15.185-tegra/`、`extra-modules-5.15.199-tegra.tar.gz`、`rt-extra-modules-5.15.148-rt-tegra.tar.gz` 这类文件则是按 rootfs / `lib/modules/<kernel>/...` 结构打包的额外模块包，适合做补充分发和归档，不是安装脚本的直接输入目录。`5.15.185-tegra` 的 extra 目录目前用于存放对应内核版本的补充模块树。

---

## 其他分支说明

### `piper-auto-start` — Piper 机械臂开机自启动

提供一个 systemd 服务，在 Jetson 开机后自动完成 Piper 机械臂的初始化流程：连接 CAN 总线 → 使能电机 → 执行回零（Homing）。完成后退出，由 ROS2 或其他程序接管控制。

**适用场景：** 需要 Piper 机械臂上电即就绪的自动化部署环境。

```bash
git checkout piper-auto-start
# 一键部署：sudo ./deploy.sh
# 管理服务：sudo systemctl status piper_boot_init.service
```

---

## 参考链接

- **RealSense 内核模块补丁：** [jetsonhacks/jetson-orin-librealsense](https://github.com/jetsonhacks/jetson-orin-librealsense)
- **gs_usb 模块参考：**
  - [NVIDIA Jetson 自定义内核指南](https://docs.nvidia.com/jetson/archives/r36.2/DeveloperGuide/SD/Kernel/BringYourOwnKernel.html)
  - [NVIDIA Forum - JetPack 6 缺失 gs_usb 讨论](https://forums.developer.nvidia.com/t/missing-gs-usb-kernel-module-for-jetpack-6)
- **Piper 机械臂 SDK：** [agilexrobotics/piper_sdk](https://github.com/agilexrobotics/piper_sdk)
