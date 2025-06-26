
# Open a terminal in the repo and source the build file
```sh
source docker_build.sh ros2im
```


# In the terminal in the repo, source the run file
```sh
source docker_run.sh ros2im ros2cont
```
# Launch Gazebo
```sh
ros2 launch rover_gazebo multi_rover.launch.py
```
# Move the robot
For the master
```sh
ros2 topic pub --rate 1 /master/cmd_vel geometry_msgs/msg/Twist "{linear: {x: -2.0, y: 0.0, z: 0.0}}"
```
For the slave
```sh
ros2 topic pub --rate 1 /slave/cmd_vel geometry_msgs/msg/Twist "{linear: {x: -2.0, y: 0.0, z: 0.0}}"
```
# Launch Gazebo+Slam+Nav2
```sh
ros2 launch rover_bringup rover_sim.launch
```


