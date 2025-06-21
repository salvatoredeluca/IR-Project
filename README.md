
## Image Compilation and Execution

2.  Open a terminal in the repo and source the build file
```sh
source docker_build.sh <IMAGE_NAME>
```
where <IMAGE_NAME> is a name for the image you want to build.

3. In the terminal in the repo, source the run file
```sh
source docker_run.sh <IMAGE_NAME> <CONTAINER_NAME>
```
where <IMAGE_NAME> is the name of the image you have just built, while <CONTAINER_NAME> is a name for the container hosting the image.

4. Spawn the robot
```sh
ros2 launch rover_gazebo multi_rover.launch.py
```

5. Send velocity messages
For the master type:
```sh
ros2 topic pub --rate 1 /slave/cmd_vel geometry_msgs/msg/Twist "{linear: {x: -2.0, y: 0.0, z: 0.0}}"
```
For the slave type:
```sh
ros2 topic pub --rate 1 /master/cmd_vel geometry_msgs/msg/Twist "{linear: {x: -2.0, y: 0.0, z: 0.0}}"
```



   
   
   
   
   
   
   
   
   
   
   
   
   
   
   

