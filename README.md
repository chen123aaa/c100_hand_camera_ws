# C100 手上相机工作空间

这个工作空间用于调试 WHEELTEC C100 USB 相机，并把它作为机械臂末端的手上相机发布到 ROS2 话题。

## 目录结构

- `src/c100_hand_camera/`：ROS2 包，包名为 `c100_hand_camera`。
- `scripts/c100_preview.py`：OpenCV 预览脚本，用来直接看 C100 画面。
- `scripts/list_video_devices.sh`：列出 Linux 视频设备和 V4L2 基本信息。
- `captures/`：预览脚本按 `s` 保存的截图目录。
- `calibration/`：预留给后续 C100 标定结果。

## 编译

```bash
cd /home/raybot/c100_hand_camera_ws
colcon build --symlink-install
source install/setup.bash
```

## 检测相机设备

机器人上可能已经接了其他相机，所以不要默认使用 `/dev/video0`。先在插入 C100 前后分别运行检测命令，对比新增的 `/dev/videoX`：

```bash
ros2 run c100_hand_camera c100_detect_devices
```

如果想进一步确认哪个设备能实际读到图像帧：

```bash
ros2 run c100_hand_camera c100_detect_devices --probe
```

启动手上相机节点时必须显式指定设备路径，例如 `device:=/dev/video2`。不传 `device` 会直接退出，避免误打开机器人上的其他相机。

## 启动 ROS2 发布节点

把 `/dev/video2` 替换成检测到的 C100 设备：

```bash
ros2 launch c100_hand_camera c100_hand_camera.launch.py device:=/dev/video2
```

默认发布话题：

- `/c100_hand_camera/image_raw`
- `/c100_hand_camera/camera_info`

默认坐标系：

- `c100_hand_camera_link`

## OpenCV 预览

只想先看画面时，可以不用 ROS 节点，直接运行预览脚本：

```bash
python3 scripts/c100_preview.py --device /dev/video2 --width 640 --height 480
```

如果要尝试 C100 的 1080P MJPEG：

```bash
python3 scripts/c100_preview.py --device /dev/video2 --width 1920 --height 1080 --mjpeg
```

预览窗口按键：

- `q` 或 `Esc`：退出。
- `s`：保存当前画面到 `captures/`。

## 默认内参和畸变

当前配置文件使用资料包给出的 C100 参考参数，配置位置：

```text
src/c100_hand_camera/config/c100_hand_camera.yaml
```

C100 在 `640x480` 下的参考内参：

```text
fx=302.7048929342819
fy=302.2019683382837
cx=298.6791162335703
cy=234.9714757570849
```

默认畸变参数使用资料教程示例：

```text
k1=0.0615721521591663
k2=0.09610650401456461
p1=0.0004141072132062723
p2=0.0008935300620624921
k3=0.02529521259640314
```

这些参数只能用于初步测试。后续如果要做机械臂手眼标定、抓取定位或精确测量，需要按实际使用分辨率重新标定 C100，然后更新 `c100_hand_camera.yaml`。

## 常用检查命令

查看图像话题是否存在：

```bash
ros2 topic list | grep c100_hand_camera
```

查看图像频率：

```bash
ros2 topic hz /c100_hand_camera/image_raw
```

查看相机内参消息：

```bash
ros2 topic echo /c100_hand_camera/camera_info --once
```
