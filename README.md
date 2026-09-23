# ros2_gazebo
ROS2 Gazebo experiments

# Dependencies
- ROS2 Jazzy & Gazebo
- Ubuntu 24.04 LTS
- Install Gazebo launch vendor library through apt

## Building a Simple Robot world

- folder: `gz_robot`

## Mapping & SLAM toolbox
- In this package, we build a basic robot and use `slam_toolbox` and `nav2` package to build map of
a given custom environment. 
- folder: `my_robot_sim`

## MoveIt2 Demo files
- folder: `moveit_demo`
- This folder contains codes for running moveit2 demo with Panda robot arm on ROS2 Jazzy


## Utlities
- folder: `scripts`

### Converting ROS2 graph into PDF

- file: `ros2_graph_export.py`: Converts ROS2 graph into a pdf file without using "rqt_graph" which is quite resource intensive.
- You should install `graphviz`: 
```
sudo apt install graphviz
python3 ./ros2_graph_export.py
```

### Installing ROS2 Jazzy on Ubuntu 24.04 WSL
- file: `install_ros2_tb3.sh`
