# Jetson Orin 自定义内核模块安装说明

> **注意**：本仓库的 `jetson-headless-display` 分支包含 Jetson 无头显示配置功能（解决 RustDesk/VNC 无显示器问题），与主分支内核模块功能无关，[点击此处查看该分支](https://github.com/XiaoweiTSN/JetsonKernalModules/tree/jetson-headless-display)。

本脚本用于在 **Jetson Orin + JetPack 6（内核版本 5.15.148-tegra）** 系统中安装 7 个自定义编译的内核模块。

- **6 个模块**增强了对 **Intel® RealSense™ 相机** 的支持；
- **1 个模块** `gs_usb.ko` 由 **NVIDIA 官方文档推荐**，用于 **Piper 机械臂** 的 USB-CAN 通信接口。

这些模块基于 NVIDIA 官方内核进行定制，并保留了原系统的内核架构。

---

## 模块说明

| 模块文件名                | 功能描述                                                                                   | 安装路径                                                                                     |
|---------------------------|--------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------|
| `uvcvideo.ko`             | USB 视频类驱动，已修改以支持 RealSense 特有的深度视频格式和元数据。                         | `/lib/modules/5.15.148-tegra/kernel/drivers/media/usb/uvc/uvcvideo.ko`                       |
| `hid-sensor-accel-3d.ko`  | RealSense 加速度传感器驱动模块。                                                           | `/lib/modules/5.15.148-tegra/kernel/drivers/iio/accel/hid-sensor-accel-3d.ko`                |
| `hid-sensor-gyro-3d.ko`   | RealSense 陀螺仪传感器驱动模块。                                                           | `/lib/modules/5.15.148-tegra/kernel/drivers/iio/gyro/hid-sensor-gyro-3d.ko`                 |
| `hid-sensor-iio-common.ko`| HID 传感器的 IIO 公共支持模块。                                                             | `/lib/modules/5.15.148-tegra/kernel/drivers/iio/common/hid-sensors/hid-sensor-iio-common.ko`|
| `hid-sensor-hub.ko`       | HID Sensor Hub 驱动，用于协调多个 RealSense 传感器。                                        | `/lib/modules/5.15.148-tegra/kernel/drivers/hid/hid-sensor-hub.ko`                           |
| `hid-sensor-trigger.ko`   | 用于 RealSense 传感器的触发器机制支持。                                                    | `/lib/modules/5.15.148-tegra/kernel/drivers/iio/common/hid-sensors/hid-sensor-trigger.ko`   |
| `gs_usb.ko`               | USB-CAN 通信驱动模块，适用于 Piper 等使用 CAN 通讯的设备。                                  | `/lib/modules/5.15.148-tegra/kernel/drivers/net/can/usb/gs_usb.ko`                           |

---

## 环境要求

- Jetson Orin 或其它 JetPack 6.2 系统；
- 内核版本必须为 `5.15.148-tegra`；
- 当前目录中包含上述 `.ko` 模块文件；
- 执行脚本时需具有 `sudo` 权限。

---

## 使用方法

### 1. 校验文件完整性与解压

模块及安装脚本均打包在压缩文件中，包含 SHA256 校验码：

```
install-modules.tar.gz
install-modules.tar.gz.sha256
```

校验压缩包完整性：

```bash
sha256sum -c install-modules.tar.gz.sha256
# 正确结果应为：install-modules.tar.gz: OK
```

解压模块目录：

```bash
tar -xzf install-modules.tar.gz
cd install-modules
```

---

### 2. 检查环境

```bash
uname -r
# 应输出：5.15.148-tegra

ls *.ko
# 应包含上文列出的所有七个 .ko 模块文件
```

---

### 3. 执行安装脚本

```bash
sudo ./install-realsense-modules.sh
```

脚本将执行以下步骤：

- 检查文件完整性与 sudo 权限；
- 自动创建目标模块目录；
- 拷贝内核模块至系统路径；
- 运行 `depmod` 更新依赖；
- 卸载已加载旧模块，加载新模块；
- 显示执行过程与成功信息。

---

### 4. 重启设备并验证是否生效
```bash
sudo reboot
```
等待重启完成
```bash
lsmod | grep -E 'uvcvideo|hid_sensor|gs_usb'
```

也可使用 `realsense-viewer` 验证相机识别是否恢复。

---

## 参考链接

- **RealSense 相关模块与补丁：**  
  [jetsonhacks/jetson-orin-librealsense](https://github.com/jetsonhacks/jetson-orin-librealsense)

- **gs_usb 模块参考与官方说明：**  
  - [NVIDIA Jetson 自定义内核指南](https://docs.nvidia.com/jetson/archives/r36.2/DeveloperGuide/SD/Kernel/BringYourOwnKernel.html)  
  - [NVIDIA Developer Forum - JetPack 6 缺失 gs_usb 讨论](https://forums.developer.nvidia.com/t/missing-gs-usb-kernel-module-for-jetpack-6)
