# RLCar_Gazebo
RLCar Gazebo Simulation  

* Build and Source

```
colcon build --symlink-install --packages-select rlcar_description
colcon build --symlink-install --packages-select rlcar_gazebo
colcon build --symlink-install --packages-select rlcar_gazebo_controller
colcon build --symlink-install --packages-select rlcar_gazebo_odometry
colcon build --symlink-install --packages-select rlcar_gazebo_slam
sudo apt install ros-foxy-jackal-simulator 
```

* Desc

```
ros2 launch rlcar_description rlcar_description.launch.py
```

* Gz

```
ros2 launch rlcar_gazebo empty_world.launch.py 
ros2 launch rlcar_gazebo racecourse.launch.py
ros2 launch rlcar_gazebo caffee_world.launch.py 
```

* SLAM 

```
ros2 launch rlcar_gazebo racecourse.launch.py use_rviz:=false
ros2 launch rlcar_gazebo_slam slam_toolbox.launch.py 
```

* Navigation

```
# Case1 - Caffee World
ros2 launch rlcar_gazebo caffee_world.launch.py use_rviz:=false
ros2 launch rlcar_gazebo_navigation caffee_bringup_launch.py 

ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap request:\ {}\

# Case2 - Race Course World
ros2 launch rlcar_gazebo racecourse.launch.py use_rviz:=false
ros2 launch rlcar_gazebo_navigation racecourse_bringup_launch.py
```
