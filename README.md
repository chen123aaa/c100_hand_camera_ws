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
ros2 launch c100_hand_camera c100_hand_camera.launch.py device:=/dev/video0
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

## 标定

标定节点参照原厂标定程序的 OpenCV 棋盘格流程实现：

```text
ROS 图像 topic
-> cv2.findChessboardCorners()
-> cv2.cornerSubPix()
-> cv2.calibrateCamera()
-> 输出 camera_matrix / distortion_coefficients
```

先启动 C100 图像发布节点：

```bash
ros2 launch c100_hand_camera c100_hand_camera.launch.py device:=/dev/video2
```

另开一个终端，进入工作空间并 source：

```bash
cd /home/raybot/c100_hand_camera_ws
source install/setup.bash
```

启动标定节点：

```bash
ros2 launch c100_hand_camera c100_calibration.launch.py
```

默认订阅：

```text
/c100_hand_camera/image_raw
```

标定调试图像：

```text
/c100_calibration/debug_image
```

这个话题参照原厂源码的 `cv2.drawChessboardCorners()`，检测到棋盘格时会在图像上画出角点和连线；未检测到时会发布带状态文字的原图。

默认标定板参数来自资料包：

```text
内角点: 9x6
方格尺寸: 25mm
采样数量: 25
```

标定时移动棋盘格，让它出现在画面中不同位置和角度。节点检测到棋盘格后，用 ROS service 手动采集当前样本。

手动采集一帧：

```bash
ros2 service call /c100_calibration/capture_sample std_srvs/srv/Trigger {}
```

如果想在采够默认样本数前提前计算：

```bash
ros2 service call /c100_calibration/calibrate std_srvs/srv/Trigger {}
```

重新开始采集：

```bash
ros2 service call /c100_calibration/reset std_srvs/srv/Trigger {}
```

采够样本后节点会计算并保存：

```text
calibration/c100_hand_camera_calibrated.yaml
```

如果要改采样数量或输出路径：

```bash
ros2 launch c100_hand_camera c100_calibration.launch.py target_samples:=30 output_path:=calibration/c100_640x480.yaml
```

生成的 YAML 可以用来更新 `src/c100_hand_camera/config/c100_hand_camera.yaml` 里的 `camera_matrix` 和 `distortion_coefficients`。

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

查看标定调试图像消息头：

```bash
ros2 topic echo /c100_calibration/debug_image --once --field header
```

## AprilTag 36h11 检测测试

当前检测脚本使用系统现有的 OpenCV ArUco/AprilTag 实现，不需要额外安装 Python AprilTag 包。先保持相机节点运行，再打开一个终端：

```bash
cd /home/raybot/c100_hand_camera_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

检测 `ID=0` 的 3cm 卡片：

```bash
ros2 launch c100_hand_camera c100_apriltag.launch.py target_id:=0 tag_size:=0.03
```

检测 `ID=0` 的 5cm 卡片时，需要停止上一个检测进程后改为：

```bash
ros2 launch c100_hand_camera c100_apriltag.launch.py target_id:=0 tag_size:=0.05
```

由于两张卡片都是 `ID=0`，检测器无法仅凭图案自动判断当前是 3cm 还是 5cm。`tag_size` 必须设置为当前放到相机前的那张卡片的实际黑色方框边长；尺寸设置错误会使输出位置按比例错误。不要让两张同 ID、不同尺寸的卡片同时出现在画面中。

检测输出：

```text
/c100_apriltag/detection   JSON 文本，包含 ID、中心像素和相机坐标系位置
/c100_apriltag/pose        geometry_msgs/PoseStamped
/c100_apriltag/debug_image 带边框、坐标轴和距离的调试图像
/tf                        c100_hand_camera_link -> apriltag_0
```

终端查看结果：

```bash
ros2 topic echo /c100_apriltag/detection
ros2 topic echo /c100_apriltag/pose
```

查看标注图像：

```bash
rqt_image_view /c100_apriltag/debug_image
```

位姿精度依赖相机内参。当前工作空间使用的是 C100 的参考内参，只适合检测联调；做精确定位前应使用本机相机重新标定。
