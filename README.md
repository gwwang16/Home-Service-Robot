##  Home Service Robot

Ubuntu 16.04 + ROS Kinetic

### Getting Started

If you do not have an active ROS workspace, you can create one by:

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Clone this repo into the **src** directory of your workspace:

```
$ cd ~/catkin_ws/src
$ git clone xxx
```

Install dependencies

```
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
$ cd ~/catkin_ws/src/Home-Service-Robot/shellscripts
$ chmod +x home_service.sh
$ chmod +x wall_follower.sh
```

Build the project:

```
$ cd ~/catkin_ws
$ catkin_make
```

Map the environment with wall following algorithm

```
$ cd ~/catkin_ws/src/Home-Service-Robot/shellscripts
$ ./wall_follower.sh
```

Save the map to file 

```
rosrun map_server map_saver -f ~/catkin_ws/src/Home-Service-Robot/homebot/world/my_map
```

Select the start and end positions using

```
rostopic echo /amcl_pose
```

It will display the pose info while you clicking any points on the map with `2D Pose Estimate` button.

Map navigation based on the built map
```
$ cd ~/catkin_ws/src/Home-Service-Robot/shellscripts
$ ./home_service.sh
```
