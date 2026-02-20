# Gazebo & ROS2 6-Axis Robotic Arm Learning Notes

## Day 1: Setup ROS2 and Gazebo Environment

**Date**: 2026-02-20

### 核心概念与操作
1. **Gazebo 安装层级**: 
   - 现代 Gazebo（过去被称为 Ignition）通常与 ROS2 一起安装，它的可执行文件路径一般位于 `/opt/ros/$ROS_DISTRO/opt/gz_tools_vendor/bin/gz`。
   - 启动仿真界面的核心命令是：`gz sim`
2. **Apple Silicon 虚拟机里的图形问题（闪屏/崩溃）**:
   - **原因**: 虚拟机的 OpenGL 硬件加速（Hardware Acceleration）可能无法完全兼容 Gazebo 的最新渲染引擎配置。
   - **解决方案**: 强制使用 CPU 进行软件渲染。虽然略微增加 CPU 负担，但在基础模型搭建和学习阶段完全足够，且能保证界面的绝对稳定。
3. **关键环境变量**:
   - `export LIBGL_ALWAYS_SOFTWARE=1`: 禁用 OpenGL 硬件加速，彻底解决闪屏问题。

### 操作命令回顾
```bash
# 检查当前的 Ubuntu 发行版和 ROS2 版本 (如 Jazzy)
cat /etc/os-release
ls /opt/ros/

# 永久应用解决虚拟机闪屏的环境变量（写入 ~/.bashrc）
echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
source ~/.bashrc

# 启动 Gazebo 仿真环境
gz sim
```

### 调试经验
- 遇到 `gz sim` 启动后持续黑屏闪烁，以及控制台出现 X11 渲染警告时，首要排查步骤是**降低渲染要求**。硬件资源的分配（尤其是 GPU）在 ARM Ubuntu 虚拟机中经常是很多奇怪图形 bug 的根源。

---
