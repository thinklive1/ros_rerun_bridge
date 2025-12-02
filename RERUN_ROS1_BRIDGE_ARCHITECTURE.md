# Rerun-ROS1 Bridge 架构详解

## 项目概述

`rerun_bridge` 是一个将 ROS1 数据流桥接到 Rerun 可视化平台的工具。它能够实时将 ROS 话题、TF 变换树等数据转换为 Rerun 可理解的格式，实现高性能的 3D 可视化。

**项目仓库**: [rerun.io](https://rerun.io)  
**Rerun SDK 版本**: 0.16.0  
**ROS 版本**: ROS1 (Noetic)

---

## 核心架构

### 1. 整体架构图

```
┌──────────────────────────────────────────────────────────────┐
│                         ROS1 生态系统                          │
│  ┌────────────┐  ┌─────────────┐  ┌──────────┐  ┌─────────┐ │
│  │ /camera/   │  │ /lidar/     │  │ /imu/    │  │ /tf     │ │
│  │   image    │  │   points    │  │   data   │  │ (树)    │ │
│  └─────┬──────┘  └──────┬──────┘  └────┬─────┘  └────┬────┘ │
└────────┼─────────────────┼──────────────┼──────────────┼─────┘
         │                 │              │              │
         │    ROS Subscribers (多线程订阅)│              │
         └─────────────────┴──────────────┴──────────────┘
                           │
         ┌─────────────────▼──────────────────┐
         │    RerunLoggerNode (桥接核心)      │
         │  ┌──────────────────────────────┐  │
         │  │   消息回调 & 数据转换层      │  │
         │  │  - log_image()               │  │
         │  │  - log_imu()                 │  │
         │  │  - log_pose_stamped()        │  │
         │  │  - log_tf_message()          │  │
         │  │  - log_odometry()            │  │
         │  │  - log_camera_info()         │  │
         │  └────────────┬─────────────────┘  │
         │               │                     │
         │  ┌────────────▼─────────────────┐  │
         │  │   Rerun RecordingStream      │  │
         │  │   (数据记录和发送)            │  │
         │  └────────────┬─────────────────┘  │
         └───────────────┼─────────────────────┘
                         │
         ┌───────────────▼──────────────────┐
         │      Rerun Viewer (可视化)       │
         │  - 3D 场景渲染                   │
         │  - 时间轴控制                    │
         │  - 实体树浏览                    │
         │  - 多视角显示                    │
         └──────────────────────────────────┘
```

---

## 核心组件详解

### 2. RerunLoggerNode (主节点类)

**文件**: `src/rerun_bridge/visualizer_node.hpp/cpp`

#### 2.1 主要成员变量

```cpp
class RerunLoggerNode {
private:
    // 话题到实体路径的映射
    std::map<std::string, std::string> _topic_to_entity_path;
    
    // 话题到订阅器的映射
    std::map<std::string, ros::Subscriber> _topic_to_subscriber;
    
    // TF 框架到实体路径的映射
    std::map<std::string, std::string> _tf_frame_to_entity_path;
    std::map<std::string, std::string> _tf_frame_to_parent;
    
    // Rerun 录制流（核心接口）
    const rerun::RecordingStream _rec{"rerun_logger_node"};
    
    // ROS 节点句柄
    ros::NodeHandle _nh{"~"};
    
    // TF 相关
    float _tf_fixed_rate;  // TF 更新频率
    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener{_tf_buffer};
};
```

#### 2.2 初始化流程

```cpp
RerunLoggerNode::RerunLoggerNode() {
    // 1. 获取 YAML 配置文件路径
    std::string yaml_path;
    _nh.getParam("yaml_path", yaml_path);
    
    // 2. 读取配置并解析
    _read_yaml_config(yaml_path);
    
    // 3. 启动 Rerun 可视化器
    _rec.spawn().exit_on_failure();
    
    // 4. 创建所有订阅器
    _create_subscribers();
}
```

#### 2.3 运行主循环

```cpp
void RerunLoggerNode::spin() {
    // 使用多线程处理消息（8个线程）
    ros::MultiThreadedSpinner spinner(8);
    
    // 如果设置了 TF 更新频率，定时更新 TF 树
    if (_tf_fixed_rate > 0) {
        ros::Rate rate(_tf_fixed_rate);
        while (ros::ok()) {
            spinner.spinOnce();
            _update_tf();  // 更新 TF 变换
            rate.sleep();
        }
    } else {
        spinner.spin();
    }
}
```

---

### 3. 配置系统 (YAML)

**示例配置**: `launch/spot_example_params.yaml`

```yaml
# 话题到实体路径的映射
topic_to_entity_path:
  /spot/camera/left/image: /odom/body/head/left/left_fisheye
  /spot/camera/left/camera_info: /odom/body/head/left/left_fisheye
  /imu/data: /odom/body/imu

# 额外的 3D 变换（静态变换）
extra_transform3ds: []

# 额外的相机针孔模型
extra_pinholes: []

# TF 树配置
tf:
  update_rate: 30.0  # Hz (设为 0 则直接记录原始 TF 数据)
  
  # TF 树结构定义（用于生成实体路径层次）
  tree:
    odom:              # 根框架
      body:            # 子框架
        camera: {}
        lidar: {}
        imu: {}
```

**配置解析流程**:

```cpp
void RerunLoggerNode::_read_yaml_config(std::string yaml_path) {
    YAML::Node config = YAML::LoadFile(yaml_path);
    
    // 1. 解析话题映射
    for (auto it : config["topic_to_entity_path"]) {
        std::string topic = it.first.as<std::string>();
        std::string entity_path = it.second.as<std::string>();
        _topic_to_entity_path[topic] = entity_path;
    }
    
    // 2. 解析 TF 树
    _tf_fixed_rate = config["tf"]["update_rate"].as<float>();
    _add_tf_tree(config["tf"]["tree"], "", "");
    
    // 3. 记录额外的静态变换和相机参数
    // ...
}
```

---

### 4. ROS 消息类型支持

#### 4.1 支持的消息类型映射

| ROS 消息类型 | Rerun 数据类型 | 转换函数 | 用途 |
|-------------|---------------|---------|------|
| `sensor_msgs::Image` | `rerun::Image` / `rerun::DepthImage` | `log_image()` | RGB/深度图像 |
| `sensor_msgs::Imu` | `rerun::Scalar` (x3) | `log_imu()` | IMU 加速度 |
| `geometry_msgs::PoseStamped` | `rerun::Transform3D` | `log_pose_stamped()` | 位姿 |
| `nav_msgs::Odometry` | `rerun::Transform3D` | `log_odometry()` | 里程计 |
| `tf2_msgs::TFMessage` | `rerun::Transform3D` | `log_tf_message()` | TF 变换 |
| `sensor_msgs::CameraInfo` | `rerun::Pinhole` | `log_camera_info()` | 相机内参 |

---

### 5. 数据转换详解

#### 5.1 图像转换 (log_image)

**文件**: `src/rerun_bridge/rerun_ros_interface.cpp`

```cpp
void log_image(
    const rerun::RecordingStream& rec, 
    const std::string& entity_path,
    const sensor_msgs::Image::ConstPtr& msg
) {
    // 1. 设置时间戳
    rec.set_time_seconds("timestamp", msg->header.stamp.toSec());
    
    // 2. 根据编码类型处理
    if (msg->encoding == "16UC1") {
        // 16位深度图（毫米单位）
        cv::Mat img = cv_bridge::toCvCopy(msg)->image;
        rec.log(
            entity_path,
            rerun::DepthImage({img.rows, img.cols}, 
                             rerun::TensorBuffer::u16(img))
                .with_meter(1000)  // 转换为米
        );
    } else if (msg->encoding == "32FC1") {
        // 32位浮点深度图（米单位）
        cv::Mat img = cv_bridge::toCvCopy(msg)->image;
        rec.log(
            entity_path,
            rerun::DepthImage({img.rows, img.cols}, 
                             rerun::TensorBuffer::f32(img))
                .with_meter(1.0)
        );
    } else {
        // RGB 彩色图像
        cv::Mat img = cv_bridge::toCvCopy(msg, "rgb8")->image;
        rec.log(
            entity_path,
            rerun::Image(tensor_shape(img), 
                        rerun::TensorBuffer::u8(img))
        );
    }
}
```

**零拷贝优化** (`collection_adapters.hpp`):

```cpp
// 通过模板特化实现零拷贝借用 OpenCV 数据
template <typename TElement>
struct rerun::CollectionAdapter<TElement, cv::Mat> {
    Collection<TElement> operator()(const cv::Mat& img) {
        // 借用数据，不复制
        return Collection<TElement>::borrow(
            reinterpret_cast<TElement*>(img.data),
            img.total() * img.channels()
        );
    }
};
```

#### 5.2 IMU 转换 (log_imu)

```cpp
void log_imu(
    const rerun::RecordingStream& rec, 
    const std::string& entity_path,
    const sensor_msgs::Imu::ConstPtr& msg
) {
    rec.set_time_seconds("timestamp", msg->header.stamp.toSec());
    
    // 分别记录三轴加速度为标量
    rec.log(entity_path + "/x", rerun::Scalar(msg->linear_acceleration.x));
    rec.log(entity_path + "/y", rerun::Scalar(msg->linear_acceleration.y));
    rec.log(entity_path + "/z", rerun::Scalar(msg->linear_acceleration.z));
}
```

#### 5.3 位姿转换 (log_pose_stamped)

```cpp
void log_pose_stamped(
    const rerun::RecordingStream& rec, 
    const std::string& entity_path,
    const geometry_msgs::PoseStamped::ConstPtr& msg
) {
    rec.set_time_seconds("timestamp", msg->header.stamp.toSec());
    
    // 记录 3D 变换（位置 + 四元数）
    rec.log(
        entity_path,
        rerun::Transform3D(
            rerun::Vector3D(
                msg->pose.position.x, 
                msg->pose.position.y, 
                msg->pose.position.z
            ),
            rerun::Quaternion::from_wxyz(
                msg->pose.orientation.w,
                msg->pose.orientation.x,
                msg->pose.orientation.y,
                msg->pose.orientation.z
            )
        )
    );
    
    // 额外记录轨迹点（用于轨迹可视化）
    std::string trajectory_path = "/trajectories/" + entity_path;
    rec.log(
        trajectory_path,
        rerun::Points3D({{
            static_cast<float>(msg->pose.position.x),
            static_cast<float>(msg->pose.position.y),
            static_cast<float>(msg->pose.position.z)
        }})
    );
}
```

#### 5.4 TF 树转换 (log_tf_message)

```cpp
void log_tf_message(
    const rerun::RecordingStream& rec,
    const std::map<std::string, std::string>& tf_frame_to_entity_path,
    const tf2_msgs::TFMessage::ConstPtr& msg
) {
    for (const auto& transform : msg->transforms) {
        // 检查是否有对应的实体路径
        if (tf_frame_to_entity_path.find(transform.child_frame_id) 
            == tf_frame_to_entity_path.end()) {
            ROS_WARN("No entity path for frame_id %s", 
                     transform.child_frame_id.c_str());
            continue;
        }
        
        rec.set_time_seconds("timestamp", transform.header.stamp.toSec());
        
        // 记录变换
        rec.log(
            tf_frame_to_entity_path.at(transform.child_frame_id),
            rerun::Transform3D(
                rerun::Vector3D(
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z
                ),
                rerun::Quaternion::from_wxyz(
                    transform.transform.rotation.w,
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z
                )
            )
        );
    }
}
```

#### 5.5 相机内参转换 (log_camera_info)

```cpp
void log_camera_info(
    const rerun::RecordingStream& rec, 
    const std::string& entity_path,
    const sensor_msgs::CameraInfo::ConstPtr& msg
) {
    // Rerun 使用列优先顺序存储 3x3 矩阵
    const std::array<float, 9> image_from_camera = {
        static_cast<float>(msg->K[0]), // fx
        static_cast<float>(msg->K[3]),
        static_cast<float>(msg->K[6]),
        static_cast<float>(msg->K[1]),
        static_cast<float>(msg->K[4]), // fy
        static_cast<float>(msg->K[7]),
        static_cast<float>(msg->K[2]), // cx
        static_cast<float>(msg->K[5]), // cy
        static_cast<float>(msg->K[8]),
    };
    
    rec.log(
        entity_path,
        rerun::Pinhole(image_from_camera)
            .with_resolution(
                static_cast<int>(msg->width), 
                static_cast<int>(msg->height)
            )
    );
}
```

---

### 6. 订阅器工厂模式

**目的**: 根据话题名自动推断消息类型并创建相应的订阅器

```cpp
void RerunLoggerNode::_create_subscribers() {
    for (const auto& [topic, entity_path] : _topic_to_entity_path) {
        ros::master::V_TopicInfo topic_infos;
        ros::master::getTopics(topic_infos);
        
        // 查找话题类型
        for (const auto& info : topic_infos) {
            if (info.name == topic) {
                std::string datatype = info.datatype;
                
                // 根据消息类型创建订阅器
                if (datatype == "sensor_msgs/Image") {
                    _topic_to_subscriber[topic] = 
                        _create_image_subscriber(topic);
                }
                else if (datatype == "sensor_msgs/Imu") {
                    _topic_to_subscriber[topic] = 
                        _create_imu_subscriber(topic);
                }
                else if (datatype == "geometry_msgs/PoseStamped") {
                    _topic_to_subscriber[topic] = 
                        _create_pose_stamped_subscriber(topic);
                }
                // ... 其他类型
                break;
            }
        }
    }
}
```

**订阅器创建示例**:

```cpp
ros::Subscriber RerunLoggerNode::_create_image_subscriber(
    const std::string& topic
) {
    return _nh.subscribe<sensor_msgs::Image>(
        topic, 
        10,  // 队列大小
        [this, topic](const sensor_msgs::Image::ConstPtr& msg) {
            std::string entity_path = _resolve_entity_path(topic);
            log_image(_rec, entity_path, msg);
        }
    );
}
```

---

### 7. TF 树管理

#### 7.1 TF 树解析

```cpp
void RerunLoggerNode::_add_tf_tree(
    const YAML::Node& node, 
    const std::string& parent_entity_path, 
    const std::string& parent_frame
) {
    for (auto it : node) {
        std::string frame = it.first.as<std::string>();
        std::string entity_path = parent_entity_path + "/" + frame;
        
        // 记录框架映射
        _tf_frame_to_entity_path[frame] = entity_path;
        _tf_frame_to_parent[frame] = parent_frame;
        
        // 递归处理子框架
        if (it.second.IsMap()) {
            _add_tf_tree(it.second, entity_path, frame);
        }
    }
}
```

#### 7.2 TF 定时更新

```cpp
void RerunLoggerNode::_update_tf() const {
    ros::Time now = ros::Time::now();
    
    for (const auto& [frame, entity_path] : _tf_frame_to_entity_path) {
        std::string parent_frame = _tf_frame_to_parent.at(frame);
        
        if (parent_frame.empty()) continue;
        
        try {
            // 查询 TF 变换
            geometry_msgs::TransformStamped transform = 
                _tf_buffer.lookupTransform(
                    parent_frame, 
                    frame, 
                    ros::Time(0)  // 获取最新变换
                );
            
            // 记录到 Rerun
            log_transform(_rec, entity_path, transform);
            
        } catch (tf2::TransformException& ex) {
            ROS_WARN("%s", ex.what());
        }
    }
}
```

---

## 8. 实体路径 (Entity Path) 系统

### 8.1 实体路径的概念

Rerun 使用类似文件系统的层次结构组织数据：

```
/world                     # 根
  /odom                    # 里程计框架
    /body                  # 机器人本体
      /camera              # 相机
        /left              # 左相机
          /image           # 图像数据
          /camera_info     # 相机参数
      /lidar               # 激光雷达
        /points            # 点云
      /imu                 # IMU
        /x                 # x 轴加速度
        /y                 # y 轴加速度
        /z                 # z 轴加速度
  /trajectories            # 轨迹
    /odom/body/pose        # 位姿轨迹
```

### 8.2 路径解析

```cpp
std::string RerunLoggerNode::_resolve_entity_path(
    const std::string& topic
) const {
    // 如果配置中有显式映射，使用配置的路径
    if (_topic_to_entity_path.count(topic) > 0) {
        return _topic_to_entity_path.at(topic);
    }
    
    // 否则自动生成路径
    // 将 /camera/left/image 转换为 /topics/camera-left/image
    std::string path = "/topics" + topic;
    std::replace(path.begin(), path.end(), '/', '-');
    return path;
}
```

---

## 9. 编译系统 (CMakeLists.txt)

### 9.1 Rerun SDK 集成

```cmake
# 使用 FetchContent 自动下载 Rerun C++ SDK
include(FetchContent)
FetchContent_Declare(
    rerun_sdk 
    URL https://github.com/rerun-io/rerun/releases/download/0.16.0/rerun_cpp_sdk.zip
)
FetchContent_MakeAvailable(rerun_sdk)
```

### 9.2 依赖关系

```cmake
find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    sensor_msgs
    nav_msgs
    geometry_msgs
    cv_bridge
    tf2_ros
    tf2_msgs
)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
```

### 9.3 构建目标

```cmake
# 库：转换接口
add_library(${PROJECT_NAME} 
    src/rerun_bridge/rerun_ros_interface.cpp
)

# 可执行文件：可视化节点
add_executable(visualizer 
    src/rerun_bridge/visualizer_node.cpp
)

# 链接
target_link_libraries(${PROJECT_NAME} 
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBS} 
    ${YAML_CPP_LIBRARIES} 
    rerun_sdk
)

target_link_libraries(visualizer 
    ${PROJECT_NAME} 
    ${catkin_LIBRARIES} 
    ${YAML_CPP_LIBRARIES} 
    rerun_sdk
)
```

---

## 10. 使用示例

### 10.1 基本启动

```bash
# 终端 1: 启动 Rerun Bridge
roslaunch rerun_bridge spot_example.launch

# 终端 2: 播放 ROS bag（如果有）
rosbag play your_data.bag
```

### 10.2 自定义配置

**创建配置文件** `my_config.yaml`:

```yaml
topic_to_entity_path:
  /camera/image_raw: /robot/camera/image
  /camera/camera_info: /robot/camera
  /imu/data: /robot/sensors/imu
  /odom: /robot/odometry

tf:
  update_rate: 30.0
  tree:
    map:
      odom:
        base_link:
          camera_link: {}
          imu_link: {}
```

**创建 launch 文件** `my_visualizer.launch`:

```xml
<launch>
  <node name="rerun_bridge" pkg="rerun_bridge" type="visualizer" output="screen">
    <rosparam param="yaml_path" subst_value="True">
      $(find my_package)/config/my_config.yaml
    </rosparam>
  </node>
</launch>
```

### 10.3 与 FAST-LIVO 集成

```bash
# 终端 1: 启动 FAST-LIVO
roslaunch fast_livo mapping_avia.launch rviz:=false

# 终端 2: 启动 Rerun 可视化
roslaunch rerun_bridge fast_livo_visualizer.launch

# 终端 3: 播放数据
rosbag play /path/to/your_data.bag
```

**FAST-LIVO 配置示例** (`fast_livo_visualizer.yaml`):

```yaml
topic_to_entity_path:
  /left_camera/image: /world/camera/left/image
  /livox/lidar: /world/lidar/points
  /livox/imu: /world/imu
  /Odometry: /world/odometry

tf:
  update_rate: 30.0
  tree:
    world:
      body:
        camera: {}
        lidar: {}
        imu: {}
```

---

## 11. 核心优势

### 11.1 性能优化

1. **零拷贝图像传输**: 通过 `CollectionAdapter` 直接借用 OpenCV 数据
2. **多线程消息处理**: 8 个线程并行处理 ROS 消息
3. **高效的二进制协议**: Rerun 使用 Apache Arrow 格式

### 11.2 功能特性

1. **实时可视化**: 低延迟的 3D 场景渲染
2. **时间旅行**: 可回放任意时间点的数据
3. **灵活的实体树**: 层次化组织所有数据
4. **多数据源**: 同时支持图像、点云、位姿、IMU 等

### 11.3 易用性

1. **自动类型推断**: 根据话题自动创建订阅器
2. **YAML 配置**: 无需修改代码即可配置
3. **TF 树同步**: 自动追踪和可视化 TF 变换

---

## 12. 常见问题排查

### 12.1 Rerun 窗口无法启动

```bash
# 设置为 headless 模式
export RERUN_HEADLESS=1
roslaunch rerun_bridge spot_example.launch

# 或使用 web viewer
export RERUN_VIEWER_PORT=9090
# 然后在浏览器中打开 http://localhost:9090
```

### 12.2 话题未被订阅

检查配置文件中的话题名是否与实际话题匹配：

```bash
# 列出所有话题
rostopic list

# 检查话题类型
rostopic info /your/topic
```

### 12.3 TF 变换未显示

确保 TF 树配置与实际 TF 树结构一致：

```bash
# 查看 TF 树
rosrun tf view_frames
# 生成 frames.pdf

# 监听 TF 变换
rosrun tf tf_echo parent_frame child_frame
```

---

## 13. 扩展开发

### 13.1 添加新的消息类型

**步骤**:

1. 在 `rerun_ros_interface.hpp` 中声明转换函数
2. 在 `rerun_ros_interface.cpp` 中实现转换
3. 在 `visualizer_node.cpp` 中添加订阅器工厂函数
4. 更新 `_create_subscribers()` 方法

**示例** (添加 `PointCloud2` 支持):

```cpp
// rerun_ros_interface.hpp
void log_point_cloud(
    const rerun::RecordingStream& rec, 
    const std::string& entity_path,
    const sensor_msgs::PointCloud2::ConstPtr& msg
);

// rerun_ros_interface.cpp
void log_point_cloud(...) {
    rec.set_time_seconds("timestamp", msg->header.stamp.toSec());
    
    // 解析点云数据
    std::vector<rerun::Position3D> points;
    // ... 解析逻辑
    
    rec.log(entity_path, rerun::Points3D(points));
}

// visualizer_node.cpp
ros::Subscriber _create_point_cloud_subscriber(const std::string& topic) {
    return _nh.subscribe<sensor_msgs::PointCloud2>(
        topic, 10,
        [this, topic](const sensor_msgs::PointCloud2::ConstPtr& msg) {
            log_point_cloud(_rec, _resolve_entity_path(topic), msg);
        }
    );
}
```

### 13.2 自定义实体路径策略

修改 `_resolve_entity_path()` 方法：

```cpp
std::string RerunLoggerNode::_resolve_entity_path(
    const std::string& topic
) const {
    // 自定义路径生成逻辑
    if (topic.find("/camera") != std::string::npos) {
        return "/sensors/cameras" + topic;
    }
    else if (topic.find("/lidar") != std::string::npos) {
        return "/sensors/lidars" + topic;
    }
    
    return "/default" + topic;
}
```

---

## 14. 总结

**Rerun-ROS1 Bridge** 通过以下机制实现了高效的数据桥接：

1. **RecordingStream API**: Rerun 的核心接口，负责数据记录和发送
2. **消息转换层**: 将 ROS 消息转换为 Rerun 数据类型
3. **订阅器工厂**: 自动创建和管理 ROS 订阅器
4. **实体路径系统**: 层次化组织可视化数据
5. **TF 树同步**: 实时追踪和显示机器人变换树
6. **YAML 配置**: 灵活的配置系统

这种设计使得用户无需修改现有的 ROS 节点，即可将数据流式传输到 Rerun 进行高性能可视化。

---

**参考链接**:
- Rerun 官方文档: https://www.rerun.io/docs
- Rerun GitHub: https://github.com/rerun-io/rerun
- ROS1 Wiki: http://wiki.ros.org
