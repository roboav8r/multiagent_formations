# Prerequisites
- ROS2 Humble
- mamba (or conda)
```
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
```

# Setup & Build
```
cd ~
mkdir -p formation_ws/src && cd formation_ws/src
git clone https://github.com/roboav8r/multiagent_formations && cd multiagent_formations
mamba env create -f formations.yml
mamba activate formations
cd ~/formation_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -y --ignore-src
colcon build
source install/setup.bash
```
# Usage
mamba activate formations


```
# Spawn world
cd ~/formation_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch multiagent_formations spawn_testbed.launch.py 

# Spawn robots 
# TODO - make spawn node for random spawning see https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_sim_demos/launch/robot_description_publisher.launch.py
mamba activate formations
cd formation_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run ros_gz_sim create -file $(ros2 pkg prefix --share multiagent_formations)/models/vehicle/model.sdf -name vehicle_1 -x 5.0 -y 5.0 -z 0.5
ros2 run ros_gz_sim create -file $(ros2 pkg prefix --share multiagent_formations)/models/vehicle/model.sdf -name vehicle_2 -x -5.0 -y 5.0 -z 0.5
ros2 run ros_gz_sim create -file $(ros2 pkg prefix --share multiagent_formations)/models/vehicle/model.sdf -name vehicle_3 -x -5.0 -y -5.0 -z 0.5
ros2 run ros_gz_sim create -file $(ros2 pkg prefix --share multiagent_formations)/models/vehicle/model.sdf -name vehicle_4 -x 5.0 -y -5.0 -z 0.5

# Launch the multiagent formation control node

# Teleop vehicle 1
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/model/vehicle_1/cmd_vel
```
