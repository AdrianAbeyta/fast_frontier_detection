# fast_frontier_detection

ROS Package to control the NRG Magni robot for autonomus exploration.

### Runing The FFD Node. 

The implementation of FFD was developed and tested on ROS Melodic. Execution on other ROS versions have not been tested and my not work as intended. For execution of the FFD algorithm please follow the steps below. Note that these instructions assume that the "desktop-full" version of ROS (which includes the Gazebo physics simulator) is installed. If a the "desktop" or "base" version are instead installed, additional effort may be needed to install Gazebo and maybe RVIZ. Note also that the implementation of FFD also requres the magni_robot and negative_obstacle_detection package to run the demonstration. 

### In separate terminal windows launch the following

1. Bringup RVIZ on the Magni robot:

```
roslaunch magni_viz view_demo.launch 
```
2. Run the negative_obstacle_detection node. 

```
rosrun negative_obstacle_detection negative_obstacle_deteion_node
```
3. Run the slam_gmapping node. 

```
rosrun gmapping slam_gmapping scan:=scan_multi
```
4. Launch the move_base package. 

```
roslaunch magni_nav move_base.launch
```

### Start the FFD demo:
Once the demo has started the robot will begin to map and explore the enviornment. The demonstration will complete once the robot has finished constucting a map of the enviornment. 

```
rosrun fast_frontier_detection fast_frontier_detection_node 
```
- Note: Instructions to preform after the FFD demo has started
- In the bottom left courner of RVIZ click add -> By topic ->  and add both /contor pointcloud and /latest_frontiers pointcloud inorder to see the visualized contor and frontiers. 
- In the dispays window unsubscribe (uncheck) all boxes except for Grid,RobotModel,Navigation,and Static Map. (For better visualization of the contors and frontiers increase the size of the pointclouds in their respective display topic.) 

### FFD Node
1. Subscribed Topics:
- odom (nav_msgs::Odometry): Odometry information from the robot. 
- scan (sensor_msgs::LaserScan): Laserscan message from LIDAR sensor. 
- map (nav_msgs::OccupancyGrid): Occupancy grid map. 

2. Published Topics:
- goal (move_base_msgs::MoveBaseGoal): Issued frontier waypoint.
- goal_viz (visualization_msgs::Marker): Visualized waypoint in RVIZ.
