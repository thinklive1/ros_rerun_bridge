# 容器内 ROS 到容器外 Rerun 可视化指南

## 架构概述

```
┌─────────────────────────────────────────────────┐
│           Docker 容器                            │
│                                                  │
│  ┌──────────────┐      ┌──────────────────┐   │
│  │  ROS Master  │◄────►│  ROS 节点         │   │
│  │  (roscore)   │      │  (发布点云数据)   │   │
│  └──────────────┘      └──────────────────┘   │
│         ▲                       │              │
│         │              发布到 /pointcloud      │
│         │                       │              │
│  ┌──────┴───────────────────────▼──────────┐  │
│  │    Rerun Bridge (记录模式)              │  │
│  │  - 订阅 ROS 话题                        │  │
│  │  - 保存到 .rrd 文件                     │  │
│  └──────────────────┬──────────────────────┘  │
│                     │                          │
│                保存到共享目录                  │
│                     │                          │
└─────────────────────┼──────────────────────────┘
                      │
              Docker Volume 映射
              /data/rerun_recordings
                      │
┌─────────────────────▼──────────────────────────┐
│              宿主机                              │
│                                                  │
│  ┌────────────────────────────────────────┐   │
│  │   Rerun Viewer (GUI)                   │   │
│  │   $ rerun /data/recording.rrd          │   │
│  │                                         │   │
│  │   实时可视化点云数据                    │   │
│  └────────────────────────────────────────┘   │
└──────────────────────────────────────────────────┘
```

---

## 方案 1: 文件共享模式（推荐，最简单）

### 原理
容器内 Rerun Bridge 将数据保存到 `.rrd` 文件，通过 Docker Volume 共享给宿主机，宿主机使用 Rerun Viewer 打开文件。

### 步骤

#### 1. 启动容器时挂载共享目录

```bash
docker run -it \
  -v /path/on/host/rerun_data:/data/rerun_recordings \
  -p 11311:11311 \
  your_ros_container:latest
```

或修改 `docker-compose.yml`:

```yaml
services:
  ros_container:
    image: your_ros_image
    volumes:
      - ./rerun_data:/data/rerun_recordings
    ports:
      - "11311:11311"
```

#### 2. 容器内配置 Rerun Bridge

**创建配置文件** `/catkin_ws/src/rerun_bridge/config/pointcloud_config.yaml`:

```yaml
# 话题到实体路径的映射
topic_to_entity_path:
  /pointcloud: /world/lidar/points
  /camera/image_raw: /world/camera/image
  /tf: /world/tf
  /odom: /world/odometry

# 不需要额外的静态变换
extra_transform3ds: []
extra_pinholes: []

# TF 树配置（根据你的机器人调整）
tf:
  update_rate: 30.0
  tree:
    world:
      base_link:
        lidar: {}
        camera: {}
```

#### 3. 修改 visualizer_node.cpp 使用保存模式

**文件**: `/catkin_ws/src/rerun_bridge/src/rerun_bridge/visualizer_node.cpp`

```cpp
RerunLoggerNode::RerunLoggerNode() {
    // 从环境变量或默认路径保存
    const char* save_path_env = std::getenv("RERUN_SAVE_PATH");
    std::string save_path = save_path_env ? 
                           save_path_env : 
                           "/data/rerun_recordings/recording.rrd";
    
    ROS_INFO("Saving Rerun data to: %s", save_path.c_str());
    _rec.save(save_path);
    
    // ...existing code...
    std::string yaml_path;
    if (_nh.getParam("yaml_path", yaml_path)) {
        ROS_INFO("Read yaml config at %s", yaml_path.c_str());
    }
    _read_yaml_config(yaml_path);
}
```

#### 4. 创建 Launch 文件

**文件**: `/catkin_ws/src/rerun_bridge/launch/pointcloud_bridge.launch`

```xml
<launch>
  <!-- Rerun Bridge 节点 -->
  <node name="rerun_bridge" pkg="rerun_bridge" type="visualizer" output="screen">
    <rosparam param="yaml_path" subst_value="True">
      $(find rerun_bridge)/config/pointcloud_config.yaml
    </rosparam>
  </node>
</launch>
```

#### 5. 容器内启动

```bash
# 终端 1: 启动 ROS Master
roscore

# 终端 2: 启动你的点云发布节点
rosrun your_package pointcloud_publisher

# 终端 3: 启动 Rerun Bridge（设置保存路径）
export RERUN_SAVE_PATH=/data/rerun_recordings/recording_$(date +%s).rrd
roslaunch rerun_bridge pointcloud_bridge.launch

# 验证话题
rostopic list | grep pointcloud
rostopic hz /pointcloud
```

#### 6. 宿主机查看

```bash
# 安装 Rerun（如果未安装）
pip install rerun-sdk

# 实时查看（会自动更新）
rerun /path/on/host/rerun_data/recording_*.rrd

# 或指定具体文件
rerun /path/on/host/rerun_data/recording_1234567890.rrd
```

---

## 方案 2: 网络流模式（实时性更好）

### 原理
在宿主机运行 Rerun Viewer 服务器，容器内 Bridge 通过网络连接发送数据。

### 步骤

#### 1. 宿主机启动 Rerun Viewer 服务器

```bash
# 启动 TCP 服务器模式
rerun --serve --port 9876
```

或启动 Web Viewer:

```bash
# Web 模式（浏览器访问）
rerun --web-viewer --port 9090
```

#### 2. 容器内修改 visualizer_node.cpp

```cpp
RerunLoggerNode::RerunLoggerNode() {
    // 从环境变量读取连接地址
    const char* rerun_addr_env = std::getenv("RERUN_ADDR");
    std::string rerun_addr = rerun_addr_env ? 
                            rerun_addr_env : 
                            "host.docker.internal:9876";  // Docker Desktop
    
    ROS_INFO("Connecting to Rerun at: %s", rerun_addr.c_str());
    auto result = _rec.connect(rerun_addr);
    
    if (result.is_err()) {
        ROS_ERROR("Failed to connect to Rerun: %s", result.error.description.c_str());
        // 降级到文件模式
        std::string fallback = "/tmp/rerun_fallback.rrd";
        ROS_WARN("Falling back to save mode: %s", fallback.c_str());
        _rec.save(fallback);
    }
    
    // ...existing code...
}
```

#### 3. 启动容器时配置网络

```bash
# 方式 1: 使用 host 网络模式（最简单）
docker run -it --network host your_ros_container

# 方式 2: 桥接模式 + 端口映射
docker run -it \
  -e RERUN_ADDR=host.docker.internal:9876 \
  your_ros_container

# 方式 3: docker-compose
```

**docker-compose.yml**:

```yaml
services:
  ros_container:
    image: your_ros_image
    network_mode: host
    environment:
      - RERUN_ADDR=localhost:9876
      - ROS_MASTER_URI=http://localhost:11311
```

#### 4. 容器内启动

```bash
# 设置 Rerun 连接地址
export RERUN_ADDR=host.docker.internal:9876

# 启动 Bridge
roslaunch rerun_bridge pointcloud_bridge.launch
```

---

## 方案 3: ROS 网络跨容器（最灵活）

### 原理
将 ROS Master 暴露给宿主机，宿主机直接运行 Rerun Bridge。

### 步骤

#### 1. 容器启动时暴露 ROS Master

```bash
docker run -it \
  -e ROS_MASTER_URI=http://0.0.0.0:11311 \
  -e ROS_HOSTNAME=192.168.1.100  # 容器 IP
  -p 11311:11311 \
  your_ros_container
```

#### 2. 宿主机配置 ROS 环境

```bash
# 设置指向容器的 ROS Master
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=192.168.1.101  # 宿主机 IP

# 测试连接
rostopic list
```

#### 3. 宿主机运行 Rerun Bridge

```bash
# 编译 Rerun Bridge（如果宿主机没有）
cd /path/to/catkin_ws
catkin_make --pkg rerun_bridge

# 启动 Bridge（会自动启动 Rerun Viewer）
source devel/setup.bash
roslaunch rerun_bridge pointcloud_bridge.launch
```

---

## 完整示例：FAST-LIVO 点云可视化

### 场景
容器内运行 FAST-LIVO，生成点云和相机数据，宿主机用 Rerun 可视化。

### 1. 创建 FAST-LIVO 配置

**文件**: `/catkin_ws/src/rerun_bridge/config/fast_livo_config.yaml`

```yaml
topic_to_entity_path:
  # 点云
  /cloud_registered: /world/lidar/registered_points
  /cloud_effected: /world/lidar/effect_points
  
  # 相机
  /left_camera/image: /world/camera/left/image
  
  # 位姿
  /Odometry: /world/odometry
  
  # IMU
  /livox/imu: /world/sensors/imu

extra_transform3ds: []
extra_pinholes: []

tf:
  update_rate: 30.0
  tree:
    world:
      body:
        lidar: {}
        camera: {}
        imu: {}
```

### 2. 创建启动脚本

**文件**: `/catkin_ws/start_fast_livo_with_rerun.sh`

```bash
#!/bin/bash
set -e

# 配置
RERUN_DIR="/data/rerun_recordings"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
RERUN_FILE="$RERUN_DIR/fast_livo_$TIMESTAMP.rrd"

# 确保目录存在
mkdir -p $RERUN_DIR

# 设置环境
export RERUN_SAVE_PATH=$RERUN_FILE
source /catkin_ws/devel/setup.bash

echo "=== Starting FAST-LIVO with Rerun Bridge ==="
echo "Rerun recording will be saved to: $RERUN_FILE"

# 启动 roscore（后台）
roscore &
ROSCORE_PID=$!
sleep 2

# 启动 FAST-LIVO（后台）
roslaunch fast_livo mapping_avia.launch rviz:=false &
FASTLIVO_PID=$!
sleep 3

# 启动 Rerun Bridge（前台）
roslaunch rerun_bridge fast_livo_bridge.launch

# 清理
echo "Shutting down..."
kill $FASTLIVO_PID $ROSCORE_PID
echo "Recording saved to: $RERUN_FILE"
echo "View on host with: rerun $RERUN_FILE"
```

### 3. 容器内运行

```bash
chmod +x /catkin_ws/start_fast_livo_with_rerun.sh
./start_fast_livo_with_rerun.sh

# 或分别启动
# 终端 1
roscore

# 终端 2
roslaunch fast_livo mapping_avia.launch rviz:=false

# 终端 3
export RERUN_SAVE_PATH=/data/rerun_recordings/recording.rrd
roslaunch rerun_bridge fast_livo_bridge.launch

# 终端 4: 播放 bag（如果有）
rosbag play your_data.bag
```

### 4. 宿主机查看

```bash
# 实时查看（自动刷新）
rerun /path/to/rerun_data/fast_livo_*.rrd

# 或用 Python API
python3 << EOF
import rerun as rr
rr.init("FAST-LIVO Viewer")
rr.connect()  # 或 rr.spawn()
# 加载 .rrd 文件
# rr.load("/path/to/recording.rrd")
EOF
```

---

## 性能优化建议

### 1. 数据传输优化

```yaml
# 降低点云密度
topic_to_entity_path:
  /cloud_registered_downsampled: /world/lidar/points  # 使用降采样后的
```

### 2. 记录频率控制

修改 Bridge 代码添加频率限制:

```cpp
ros::Subscriber _create_point_cloud_subscriber(const std::string& topic) {
    static ros::Time last_log_time = ros::Time(0);
    static const double min_interval = 0.1;  // 10Hz
    
    return _nh.subscribe<sensor_msgs::PointCloud2>(
        topic, 10,
        [this, topic](const sensor_msgs::PointCloud2::ConstPtr& msg) {
            ros::Time now = ros::Time::now();
            if ((now - last_log_time).toSec() < min_interval) {
                return;  // 跳过
            }
            last_log_time = now;
            log_point_cloud(_rec, _resolve_entity_path(topic), msg);
        }
    );
}
```

### 3. 文件大小控制

```bash
# 使用流式保存（限制内存）
export RERUN_FLUSH_TIMEOUT_SEC=1

# 分段保存
while true; do
    TIMESTAMP=$(date +%s)
    export RERUN_SAVE_PATH=/data/rerun_recordings/segment_$TIMESTAMP.rrd
    timeout 60 roslaunch rerun_bridge pointcloud_bridge.launch
done
```

---

## 调试技巧

### 检查 ROS 通信

```bash
# 容器内
rostopic list
rostopic hz /pointcloud
rostopic echo /pointcloud --noarr  # 不显示数组内容

# 检查消息类型
rostopic info /pointcloud
```

### 检查 Rerun 连接

```bash
# 查看 Rerun 进程
ps aux | grep rerun

# 查看保存的文件
ls -lh /data/rerun_recordings/

# 查看文件内容摘要
rerun --cat /data/rerun_recordings/recording.rrd | head -20
```

### 性能监控

```bash
# 容器内监控
rosrun rqt_top rqt_top  # CPU/内存使用
rosrun rqt_graph rqt_graph  # 节点图

# 宿主机监控
htop
iotop  # IO 使用
```

---

## 故障排查

### 问题 1: 容器内 Bridge 无法启动

```bash
# 检查 Rerun SDK 是否正确安装
python3 -c "import rerun; print(rerun.__version__)"

# 检查共享目录权限
ls -la /data/rerun_recordings/
chmod 777 /data/rerun_recordings/
```

### 问题 2: 点云不显示

```bash
# 检查话题类型
rostopic type /pointcloud  # 应该是 sensor_msgs/PointCloud2

# 检查是否有数据
rostopic hz /pointcloud

# 检查配置文件中的路径映射
cat /catkin_ws/src/rerun_bridge/config/pointcloud_config.yaml
```

### 问题 3: 文件无法访问

```bash
# 检查 Docker Volume 挂载
docker inspect <container_id> | grep -A 10 Mounts

# 检查文件权限
ls -la /path/on/host/rerun_data/
```

---

## 总结

| 方案 | 优点 | 缺点 | 适用场景 |
|------|------|------|----------|
| **文件共享** | 简单可靠，无网络依赖 | 非实时，需等待记录完成 | 离线分析，长时间记录 |
| **网络流** | 实时性好，交互式 | 需要网络配置，延迟敏感 | 实时调试，短时间测试 |
| **ROS 网络** | 最灵活，可用所有 ROS 工具 | 配置复杂，网络要求高 | 分布式系统，多机协作 |

**推荐方案**: 
- **开发调试**: 方案 2（网络流）
- **生产记录**: 方案 1（文件共享）
- **多机部署**: 方案 3（ROS 网络）

---

## 参考资源

- Rerun 文档: https://www.rerun.io/docs
- Docker 网络: https://docs.docker.com/network/
- ROS 网络配置: http://wiki.ros.org/ROS/NetworkSetup
- 本项目架构文档: [`RERUN_ROS1_BRIDGE_ARCHITECTURE.md`](RERUN_ROS1_BRIDGE_ARCHITECTURE.md)
