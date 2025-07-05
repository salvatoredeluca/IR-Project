
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
# Move the robots
For the master
```sh
ros2 topic pub --rate 1 /master/cmd_vel geometry_msgs/msg/Twist "{linear: {x: -2.0, y: 0.0, z: 0.0}}"
```
For the slave
```sh
ros2 topic pub --rate 1 /slave/cmd_vel geometry_msgs/msg/Twist "{linear: {x: -2.0, y: 0.0, z: 0.0}}"
```
# Launch Gazebo+Slam+Nav2
Then on RVIZ you can assign a goal to both master and slave indipendently (by default the topic is /master/goal_pose so to assign goals to the slave you have to change it in /slave/goal_pose)
```sh
ros2 launch rover_bringup rover_sim.launch
```

# Launch the follow task
After launching Gazebo+SLAM+NAV2, make the master follow the desired waypoints

```sh
ros2 run rover_bringup follow_waypoints
```
Then on another terminal paste this to make the slave follow the master
```sh
ros2 run rover_bringup simple_follower
```




