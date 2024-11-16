The planner stack consists of two ROS nodes, GBPlanner and Planner Control Interface (PCI). 

## Input Interface

Different ROS services are provided to call different features of the planner. The following services can be used to interact with the planner control interface:  
- ```planner_control_interface/std_srvs/automatic_planning``` ([std_srvs/Trigger](http://docs.ros.org/en/indigo/api/std_srvs/html/srv/Trigger.html)): Trigger planner. PCI will keep triggering the planner at the end of every planning iteration.
- ```planner_control_interface/std_srvs/single_planning``` ([std_srvs/Trigger](http://docs.ros.org/en/indigo/api/std_srvs/html/srv/Trigger.html)): Trigger only one planning iteration.
- ```planner_control_interface/stop_request``` ([std_srvs/Trigger](http://docs.ros.org/en/indigo/api/std_srvs/html/srv/Trigger.html)): Stop the planner and bring the robot to a hault.
- ```planner_control_interface/std_srvs/homing_trigger``` ([std_srvs/Trigger](http://docs.ros.org/en/indigo/api/std_srvs/html/srv/Trigger.html)): Trigger homing manuever. Planner will return path to the home location after finishing the current planning iteration.
- ```planner_control_interface/std_srvs/go_to_waypoint``` ([std_srvs/Trigger](http://docs.ros.org/en/indigo/api/std_srvs/html/srv/Trigger.html)): Service to plan path to the given waypoint along the global graph. Note: Global graph will only start building when the planner is triggered for the first time.
- ```pci_initialization_trigger``` ([planner_msgs/pci_initialization](https://github.com/ntnu-arl/gbplanner_ros/tree/gbplanner2/planner_msgs/srv/pci_initialization.srv)): Trigger initialization motion.
- ```pci_global``` ([planner_msgs/pci_global](https://github.com/ntnu-arl/gbplanner_ros/tree/gbplanner2/planner_msgs/srv/pci_global.srv)): Trigger global planner to send the robot to specific vertex in the global garph (need to know the id of that vertex).

## Output Interface
Two output interfaces are provided for the PCI: Action-based and Topic-based.
### Action-based
Action: ```pci_output_path``` ([planner_msgs/pathFollowerAction](https://github.com/ntnu-arl/gbplanner_ros/tree/gbplanner2/planner_msgs/action/pathFollowerAction.action))  
PCI will send goal to an action server hosting the above action. PCI expects an estimated time remaining to complete the path exeution from the server as a feedback. Please see the action description for more details of the goal, feedback and result of the action.

To enable this mode set the ```output_type``` in the planner_control_interface_sim_config.yaml to ```kAction``` as shown [here](https://github.com/ntnu-arl/gbplanner_ros/blob/d9ecc1f6ab597a6beb183ef058280ad207dc5bf9/gbplanner/config/smb/planner_control_interface_sim_config.yaml#L4).

The ground robot demo in the [Demo](Demo) section uses the action-based interface.

### Topic-base
In this mode, PCI will publish the calculated path in two formats on the following topics:
- ```command/trajectory``` ([trajectory_msgs/MultiDOFJointTrajectory](http://docs.ros.org/en/noetic/api/trajectory_msgs/html/msg/MultiDOFJointTrajectory.html))
- ```pci_command_path``` ([geometry_msgs/PoseArray](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseArray.html))

To enable this mode set the ```output_type``` in the planner_control_interface_sim_config.yaml to ```kTopic``` as shown [here](https://github.com/ntnu-arl/gbplanner_ros/blob/d9ecc1f6ab597a6beb183ef058280ad207dc5bf9/gbplanner/config/rmf_obelix/planner_control_interface_sim_config.yaml#L4).

The aerial robot demo in the [Demo](Demo) section uses the topic-based interface.


## Subscribed Topics
### GBPlanner:
- ```pose``` ([geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)): Robot pose in PoseWithCovarianceStamped format.
- ```pose_stamped``` ([geometry_msgs/PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html)): Robot pose in PoseStamped format.
- ```odometry``` ([nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)): Robot's odometry.
- ```/robot_status``` ([planner_msgs/RobotStatus](https://github.com/ntnu-arl/gbplanner_ros/tree/gbplanner2/planner_msgs/msg/RobotStatus.msg)): Topic used to get the time remaining before the battery runs out.
- ```planner_control_interface/stop_request``` ([std_msgs/Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html)): Feedback from PCI when the stop service is called.
- ```/gbplanner_node/tsdf_map_in``` ([voxblox_msgs/Layer](https://github.com/ethz-asl/voxblox/blob/master/voxblox_msgs/msg/Layer.msg)): Input voxblox TSDF map if an external map is to be fed to the planner
- ```/gbplanner_node/esdf_map_in``` ([voxblox_msgs/Layer](https://github.com/ethz-asl/voxblox/blob/master/voxblox_msgs/msg/Layer.msg)): Input voxblox ESDF map if an external map is to be fed to the planner

### Planner Control Interface:
- ```pose``` ([geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)): Robot pose in PoseWithCovarianceStamped format.
- ```pose_stamped``` ([geometry_msgs/PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html)): Robot pose in PoseStamped format.
- ```odometry``` ([nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)): Robot's odometry.
- ```/move_base_simple/goal``` ([geometry_msgs/PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html)): Setting the waypoint to go to along the global graph.


## Published Topics
### GBPlanner:
- ```/gbplanner_node/mesh``` ([voxblox_msgs/Mesh](https://github.com/ethz-asl/voxblox/blob/master/voxblox_msgs/msg/Mesh.msg)): Voxblox mesh
- ```/gbplanner_node/occupied_nodes``` ([visualization_msgs/MarkerArray](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html)): Voxblox occupancy map occupied voxels
- ```/gbplanner_node/surface_pointcloud``` ([sensor_msgs/PointCloud2](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html)): Voxblox surface pointcloud
- ```/gbplanner_node/tsdf_map_out``` ([voxblox_msgs/Layer](https://github.com/ethz-asl/voxblox/blob/master/voxblox_msgs/msg/Layer.msg)): TSDF map published by voxblox
- ```/gbplanner_node/tsdf_pointcloud``` ([sensor_msgs/PointCloud2](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html)): Voxblox TSDF map published as a pointcloud
- ```/gbplanner_node/tsdf_slice``` ([sensor_msgs/PointCloud2](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html)): Slice of voxblox TSDF map at a set height published as a pointcloud
- ```/gbplanner_node/esdf_map_out``` ([voxblox_msgs/Layer](https://github.com/ethz-asl/voxblox/blob/master/voxblox_msgs/msg/Layer.msg)): ESDF map published by voxblox
- ```/gbplanner_node/esdf_pointcloud``` ([sensor_msgs/PointCloud2](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html)): Voxblox ESDF map published as a pointcloud
- ```/gbplanner_node/esdf_slice``` ([sensor_msgs/PointCloud2](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html)): Slice of voxblox ESDF map at a set height published as a pointcloud
- ```/pci_command_trajectory_vis``` ([visualization_msgs/MarkerArray](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html)): Visualization of the trajectory to be followed by the robot
- Topics starting with ```vis/``` ([visualization_msgs/MarkerArray](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html)): Visualization markers to visualize various parts of the planner. Please refer to [gbplanner_viz.h](https://github.com/ntnu-arl/gbplanner_ros/blob/gbplanner2/gbplanner/include/gbplanner/gbplanner_rviz.h) for details about each topic.

### Planner Control Interface:
- ```gbplanner/go_to_waypoint_pose_visualization``` ([visualization_msgs/MarkerArray](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html)): Visualization of the waypoint to go to along the global graph.