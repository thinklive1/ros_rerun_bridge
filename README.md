# fastlivo2

1. [项目地址](https://github.com/hku-mars/FAST-LIVO2)
2. [复现经验帖](https://zhuanlan.zhihu.com/p/25774287765)

编译sophus时遇到的问题，github上有[issue](https://github.com/uzh-rpg/rpg_svo/issues/237)

# 容器说明

系统: Ubuntu 20.04
ROS版本: Noetic
包管理: conda
conda环境名: base(python 3.13)
ros项目地址: /catkin_ws
用户: 除了默认的root外，设置了一个普通用户fastlivo,密码同用户名, 装了zsh和相关差距，方便终端操作, root用户的.bashrc写了一行`su fastlivo`自动切换用户，权限不足想回root时可以`exit`

## 桥接节点说明

该节点用于将FAST-LIVO2发布的点云数据桥接到Rerun viewer，因为现在三是在docker容器里连接主机, 所以grpc链接是 `rerun+http://172.17.0.1:9876/proxy`  
目前订阅的topic是 `/cloud_registered`，launch文件在 `rerun_bridge/launch/pointcloud_bridge.launch`,py的ros节点脚本在 `rerun_bridge/scripts/pointcloud_bridge.py`  
`rerun_bridge/src`目录下的c++代码是之前的示例项目代码，后来因为发现旧版本sdk不支持grpc, 且就当前任务来说不如轻量级的python方便，所以废弃



