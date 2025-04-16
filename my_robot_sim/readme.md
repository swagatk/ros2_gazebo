# Building Map using Nav2 and SLAM toolbox

## Dependencies
- Ubuntu 24.04 LTS
- [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/ros_installation/)
```
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
```

## Resources/Credits
- [Nav2 Documentation](https://docs.nav2.org/setup_guides/index.html)
- [ROS2 Jazzy Documenation](https://docs.ros.org/en/jazzy/Tutorials.html)

## Download & Install
Download the package into your home directory `~/`:
```
git clone https://github.com/swagatk/ros2_gazebo.git
```

copy the folder `~/ros2_gazebo/my_robot_sim/` to the `src` folder of your local ros2 workspace.
```
cp -r ~/ros2_gazebo/my_robot_sim ~/ros2_ws/src/
cd ~/ros2_ws
```
Install required dependencies for the ros package
```
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
Build the package:
```
colcon build --packages-select my_robot_sim
source install/setup.bash
```
Now launch the application
```
ros2 launch my_robot_sim display.launch.py
```

## Install dependencies
Install the following packages if `rosdep install` does not resolve the dependencies:
```
sudo apt install ros-jazzy-xacro
sudo apt install ros-jazzy-sdformat-urdf
sudo apt install ros-jazzy-ros-gz
sudo apt install ros-<ros2-distro>-gazebo-ros-pkgs

```
## Launch SLAM toolbox
Install `slam_toolbox` package
```
sudo apt install ros-jazzy-slam-toolbox
```

Now execute the following command in a separate terminal:
```
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```
### Launch Nav2

Install `nav2` packages:
```
sudo apt install ros-jazzy-navigation2
sudo apt install ros-jazzy-nav2-bringup
```
Now execute the following command in a separate terminal: 
```
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```