# ROS2 A* Demo

这是从 `ros_motion_planning` 里摘出来的一个独立 ROS2 Jazzy 演示工作区，目标是直接给 `/home/jason/ROS_Flutter_Gui_App` 联调。

当前工作区提供：

- `warehouse` 静态地图
- TurtleBot3 Waffle URDF 模型
- 纯 ROS2 Python A* 规划节点
- 简化机器人运动仿真与路径跟踪
- 模拟 RGB 摄像头画面：`/camera/rgb/image_raw`
- `rosbridge websocket`，默认监听 `0.0.0.0:9090`
- Flutter App 需要的核心 topic：`/map`、`/tf`、`/odom`、`/plan`、`/local_plan`、`/goal_pose`、`/cmd_vel`
- 兼容 App 的 `topology_msgs`

## 目录

```text
ros_planning/
├── src/
│   ├── astar_demo/
│   └── topology_msgs/
└── start_demo.sh
```

## 首次编译

```bash
cd /home/jason/ros_planning
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

## 启动

```bash
cd /home/jason/ros_planning
./start_demo.sh
```

如需同时打开 RViz2：

```bash
./start_demo.sh use_rviz:=true
```

## Flutter App 联调

1. 保证手机和电脑在同一局域网。
2. 启动本工作区：`./start_demo.sh`
3. 电脑上查看局域网 IP：

```bash
hostname -I
```

4. 手机端 `/home/jason/ROS_Flutter_Gui_App` 输入：

- IP: 电脑局域网 IP
- Port: `9090`

5. App 里建议选择 ROS2 / TurtleBot3 相关默认配置；本工程兼容以下关键 topic：

- `map`
- `/camera/rgb/image_raw`
- `/camera/rgb/camera_info`
- `/scan`
- `/plan`
- `/local_plan`
- `/transformed_global_plan`
- `/goal_pose`
- `/initialpose`
- `/cmd_vel`
- `/odom`
- `/wheel/odometry`
- `/local_costmap/costmap`
- `/global_costmap/costmap`
- `/local_costmap/published_footprint`
- `/battery_status`
- `/diagnostics`
- `/map/topology`
- `/map/topology/update`
- `/nav_to_topology_point`

### Flutter 后端摄像头联调

如果使用 `/home/jason/ROS_Flutter_Gui_App/backend` 作为 Flutter App 的后端，摄像头画面走后端的 HTTP/WebSocket 通道：

1. 启动本工作区：`./start_demo.sh`
2. 启动 Flutter GUI 后端，默认 HTTP 端口为 `8080`
3. App 连接电脑局域网 IP，端口填写 `8080`
4. 设置页保持图像话题为默认值：`/camera/rgb/image_raw`
5. 主界面点击相机按钮，即可订阅并显示摄像头画面

摄像头发布参数可在启动时覆盖：

```bash
./start_demo.sh camera_topic:=/camera/rgb/image_raw camera_width:=320 camera_height:=240 camera_publish_rate:=5.0
```

## 已知说明

- 这是轻量演示栈，不依赖 Gazebo，也没有把 ROS1 的 `move_base` / pluginlib 体系整体迁过去。
- 手动控制支持 `vx/vy/vw`，其中 `vy` 以简化平面运动方式处理，主要目的是让 App 摇杆可直接联调。
- 地图编辑页发布的 `/map/update` 和 `/map/topology/update` 已接入内存更新，但默认不自动落盘。
- 当前摄像头是 ROS 端生成的模拟 RGB 图像，用于 App 话题订阅和显示链路联调；后续接入真实 USB/CSI 相机时，只要发布同名 `sensor_msgs/Image` 或 `sensor_msgs/CompressedImage` 即可复用 Flutter 显示链路。
