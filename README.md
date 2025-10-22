# Jetson 无头显示配置（Headless Display）


### 此包解决什么问题？

在 Jetson Orin NX/Xavier/Nano 等设备上运行 **RustDesk**、**VNC** 或任何需要 **GPU 加速的图形界面程序** 时，如果**没有插入物理显示器**（HDMI/DP），系统会出现以下问题：

- **RustDesk 显示 "No displays" / 黑屏**
- **Xorg 无法启动 NVIDIA GPU 驱动**
- **OpenGL / CUDA / TensorRT GUI 程序无法正常渲染**
- 默认 framebuffer 分辨率只有 **2048×1152**，无法调整
- 设备无法在"无头模式（headless）"下远程访问桌面

**核心需求**：让 Jetson 在没有插显示器的情况下，也能像插了显示器一样启动完整图形界面和 GPU。

---

## 解决方案


本方案利用 **NVIDIA 驱动的 CustomEDID + ConnectedMonitor 机制**，通过伪造一个虚拟显示器的 EDID（Extended Display Identification Data）文件，让 Xorg 和 GPU 驱动"误以为"显示器始终插着，从而正常初始化图形输出。



### 工作原理

本配置使用的 EDID 文件来自 **Redmi 27 寸显示器**（原生支持 2560×1440@100Hz）。即使没有插显示器，Xorg 仍然创建了 **2560×1440** 的虚拟显示面板，RustDesk/VNC 可正常使用，GPU 完全启用。

---

## 使用方法

### 包含文件

- `edid_1440p.bin` - 从 **Redmi 27 寸显示器**导出的 EDID 文件（原生支持 2560×1440@100Hz）
- `xorg.conf` - Xorg 配置文件，指定使用虚拟 EDID

### 安装步骤

#### 步骤 1：备份原有配置（如果存在）

```bash
# 备份原有的 xorg.conf（如果存在）
sudo cp /etc/X11/xorg.conf /etc/X11/xorg.conf.backup 2>/dev/null || echo "没有找到原配置文件，跳过备份"
```

#### 步骤 2：复制新配置文件

```bash
# 进入本目录
cd ~/jetson-headless-diaplay

# 复制 EDID 文件到系统目录
sudo cp edid_1440p.bin /etc/X11/edid_1440p.bin

# 复制 Xorg 配置文件到系统目录
sudo cp xorg.conf /etc/X11/xorg.conf

# 确保文件权限正确
sudo chmod 644 /etc/X11/edid_1440p.bin
sudo chmod 644 /etc/X11/xorg.conf
```

#### 步骤 3：重启显示服务或系统

**选项 A：仅重启显示管理器**（推荐快速测试）

```bash
sudo systemctl restart gdm
```

**选项 B：重启整个系统**（推荐首次配置）

```bash
sudo reboot
```

---

## 验证效果

### 1. 检查虚拟显示器是否创建成功

```bash
xrandr
```

**预期输出示例**：

```
Screen 0: minimum 8 x 8, current 4096 x 2304, maximum 32767 x 32767
DP-0 connected primary 4096x2304+0+0 (normal left inverted right x axis y axis) 597mm x 336mm
   2560x1440     59.56*+  99.95    75.03  
   1920x1080     60.00    59.94    50.00  
   1600x900      60.00  
   1440x900      59.89  
   1280x1024     75.02    60.02  
   1280x720      59.94    50.00  
   1024x768      75.03    60.00  
   800x600       75.00    72.19    60.32  
   720x576       50.00  
   720x480       59.94  
   640x480       75.00    59.94    59.93
```

看到 `DP-0 connected` 和 `2560x1440` 表示配置成功。

### 2. 检查 GPU 渲染是否可用

```bash
glxinfo | grep "renderer"
```

**预期输出**：

```
OpenGL renderer string: NVIDIA Tegra Orin (nvgpu)/integrated
```

看到 `NVIDIA Tegra Orin` 表示 GPU 正常加载。

### 3. 测试远程桌面软件

- **RustDesk**：应该可以正常显示桌面画面，不再显示 "No displays"
- **VNC**（TigerVNC/RealVNC）：可以正常连接并显示桌面
- **OpenGL 程序**：`glxgears` 等测试程序可以正常运行

```bash
glxgears
```

如果窗口正常显示并流畅旋转齿轮，说明配置完全成功。

---

## 常见问题

### 问题 1：插入真实显示器后，依然使用虚拟 EDID

**现象**：插入物理显示器后，系统仍然使用 `/etc/X11/edid_1440p.bin` 中的分辨率，而不是显示器的原生分辨率。

**原因**：`xorg.conf` 中的 `CustomEDID` 参数强制覆盖了真实显示器的 EDID。

**解决方法**：

**方案 A：临时恢复原配置**（插入显示器时使用）

```bash
# 重命名配置文件，使其失效
sudo mv /etc/X11/xorg.conf /etc/X11/xorg.conf.headless

# 重启显示服务
sudo systemctl restart gdm

# 使用完毕后，恢复无头配置
sudo mv /etc/X11/xorg.conf.headless /etc/X11/xorg.conf
sudo systemctl restart gdm
```

**方案 B：修改配置文件，允许自动检测**

编辑 `/etc/X11/xorg.conf`，注释掉 `CustomEDID` 行：

```bash
sudo nano /etc/X11/xorg.conf
```

找到：

```
Option      "CustomEDID" "DFP-0:/etc/X11/edid_1440p.bin"
```

修改为：

```
# Option      "CustomEDID" "DFP-0:/etc/X11/edid_1440p.bin"  # 注释掉，允许使用真实显示器
```

然后重启显示服务：

```bash
sudo systemctl restart gdm
```

### 问题 2：重启后黑屏或无法进入图形界面

**解决方法**：

1. 按 `Ctrl + Alt + F3` 切换到 TTY3 终端
2. 登录账户
3. 恢复备份的配置：

```bash
sudo rm /etc/X11/xorg.conf
sudo cp /etc/X11/xorg.conf.backup /etc/X11/xorg.conf
sudo systemctl restart gdm
```

### 问题 3：分辨率不符合需求

**解决方法**：

编辑 `/etc/X11/xorg.conf`，修改 `PreferredMode` 和 `Modes` 参数：

```bash
sudo nano /etc/X11/xorg.conf
```

找到 `Monitor` 和 `Screen` 部分，修改为所需分辨率：

```
Section "Monitor"
    Identifier "Monitor0"
    Option "PreferredMode" "1920x1080"  # 修改默认分辨率
EndSection

Section "Screen"
    SubSection "Display"
        Modes "1920x1080" "2560x1440" "3840x2160"  # 可选分辨率列表
    EndSubSection
EndSection
```

保存后重启显示服务。

---

## 额外功能

### 自动化安装脚本（可选）

如果需要在多台设备上部署，可以创建自动安装脚本：

```bash
#!/bin/bash
# 自动安装无头显示配置

# 备份原配置
sudo cp /etc/X11/xorg.conf /etc/X11/xorg.conf.backup 2>/dev/null

# 复制文件
sudo cp edid_1440p.bin /etc/X11/edid_1440p.bin
sudo cp xorg.conf /etc/X11/xorg.conf

# 设置权限
sudo chmod 644 /etc/X11/edid_1440p.bin
sudo chmod 644 /etc/X11/xorg.conf

# 重启显示服务
sudo systemctl restart gdm

echo "无头显示配置安装完成！"
```

### 完全恢复到默认配置

```bash
# 删除自定义配置
sudo rm /etc/X11/xorg.conf
sudo rm /etc/X11/edid_1440p.bin

# 重启显示服务
sudo systemctl restart gdm
```


