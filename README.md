# robot_delivery_system

#### Hero controller code:
Please follow the [Hero Controller User Guide](https://store.ctr-electronics.com/content/user-manual/HERO%20User's%20Guide.pdf) for instructions on how to run the code on the Hero Controller board

### Compilation Instructions
1. `git clone` the repo
2. `cd robot_delivery_system` 
3. `catkin_make`

### How to run perception package
1. Open new Termial: `roscore` and `source devel/setup.bash`
2. run `sudo chmod 777 /dev/ttyUSB0` to connect to lidar
3. `roslaunch perception perception.launch`

### How to run 3iRobotics lidar ros package alone (skip if running perception package):
1. Open new Termial: `roscore` and `source devel/setup.bash`
2. run `sudo chmod 777 /dev/ttyUSB0` to connect to lidar
3. run publish_node:
	`rosrun delta_lidar delta_lidar_node` or `roslaunch  delta_lidar delta_lidar.launch`
	(NOTE:please select correct serial in “launch\delta_lidar.launch”)
4. run subscribe_node:
* Open new Termial: `source devel/setup.bash`
* `rosrun delta_lidar delta_lidar_node_client`

### How to run 3iRobotics lidar rviz:
`roslaunch  delta_lidar view_delta_lidar.launch`
