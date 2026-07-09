# C100 手上相机工作空间

这个工作空间用于把 WHEELTEC C100 USB 相机作为机械臂末端手上相机发布到 ROS2 话题。流程全部通过终端完成，图像通过 ROS topic 解析，不提供本地 OpenCV 预览窗口。

## 编译

```bash
cd /home/raybot/c100_hand_camera_ws
colcon build --symlink-install
source install/setup.bash
```

## 检测设备

机器人可能同时连接多个相机，所以启动节点前必须先确认 C100 对应的 `/dev/videoX`。

插入 C100 前后分别运行：

```bash
ros2 run c100_hand_camera c100_detect_devices
```

需要尝试读取一帧来确认设备可用时：

```bash
ros2 run c100_hand_camera c100_detect_devices --probe
```

## 启动相机话题

把 `/dev/video2` 替换为检测到的 C100 设备：

```bash
ros2 launch c100_hand_camera c100_hand_camera.launch.py device:=/dev/video2
```

节点不接受空的 `device` 参数。不显式传入设备路径时会直接退出，避免误打开其他相机。

## 发布内容

默认话题：

```text
/c100_hand_camera/image_raw
/c100_hand_camera/camera_info
```

默认坐标系：

```text
c100_hand_camera_link
```

默认参数配置：

```text
src/c100_hand_camera/config/c100_hand_camera.yaml
```

## 默认内参和畸变

当前先使用资料包里的 C100 参考参数，分辨率为 `640x480`。

内参：

```text
fx=302.7048929342819
fy=302.2019683382837
cx=298.6791162335703
cy=234.9714757570849
```

畸变参数：

```text
k1=0.0615721521591663
k2=0.09610650401456461
p1=0.0004141072132062723
p2=0.0008935300620624921
k3=0.02529521259640314
```

这些参数只适合初步取流和话题联调。后续做手眼标定、抓取定位或精确测量时，需要按实际使用分辨率重新标定 C100，并更新 `c100_hand_camera.yaml`。

## 终端检查

查看话题：

```bash
ros2 topic list | grep c100_hand_camera
```

查看图像频率：

```bash
ros2 topic hz /c100_hand_camera/image_raw
```

查看相机内参：

```bash
ros2 topic echo /c100_hand_camera/camera_info --once
```

查看一帧图像消息头：

```bash
ros2 topic echo /c100_hand_camera/image_raw --once --field header
```
