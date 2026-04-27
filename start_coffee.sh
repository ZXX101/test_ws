#!/bin/bash
# CoffeeProj 启动脚本
# 使用默认参数启动 coffeeProj_pkg

# Source ROS 环境
source /opt/ros/noetic/setup.bash
source /home/zxx/test_ws/devel/setup.bash

# 启动 coffee_proj_node (使用 launch 中的默认参数)
roslaunch coffeeproj_pkg coffee_proj.launch