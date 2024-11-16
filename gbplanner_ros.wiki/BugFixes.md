# Fixes for common bugs/issues
This page contains solutions for common problems that might be encountered during the installation and testing of GBPlanner2. The page will be updated as new bugs/issues are found.

## Issue with reading xacro file on ROS-Melodic
This is a known issue in the community and xacro 1.14.13 fixes this. However, as of March 10, 2022, it is not synced with the public ros repositories. You can either install this version from the source or via the ros-testing repositories. The instructions for doing the same can be found here: [http://wiki.ros.org/TestingRepository](http://wiki.ros.org/TestingRepository)

Further information about the issue:  
https://github.com/ethz-asl/rotors_simulator/issues/695#issue-113445556  
https://github.com/ros/xacro/issues/316#issue-1155057122  
https://github.com/ros/rosdistro/issues/32236

## Protobuf Issues
Due to protobuf mismatch (or missing compiler) you might encounter an error similar to the one below:
```bash
Errors     << voxblox_ros:make /home/pol/Downloads/catkin_ws/logs/voxblox_ros/build.make.004.log
/usr/bin/ld: CMakeFiles/visualize_tsdf.dir/src/visualize_tsdf.cc.o: undefined reference to symbol '_ZN6google8protobuf8internal10LogMessageC1ENS0_8LogLevelEPKci'
/usr/bin/ld: /lib/x86_64-linux-gnu/libprotobuf.so.17: error adding symbols: DSO missing from command line
collect2: error: ld returned 1 exit status
make[2]: *** [CMakeFiles/visualize_tsdf.dir/build.make:469: /home/pol/Downloads/catkin_ws/devel/.private/voxblox_ros/lib/voxblox_ros/visualize_tsdf] Error 1
make[1]: *** [CMakeFiles/Makefile2:1527: CMakeFiles/visualize_tsdf.dir/all] Error 2
make[1]: *** Waiting for unfinished jobs....
/usr/bin/ld: CMakeFiles/voxblox_eval.dir/src/voxblox_eval.cc.o: undefined reference to symbol '_ZN6google8protobuf8internal10LogMessageC1ENS0_8LogLevelEPKci'
/usr/bin/ld: /lib/x86_64-linux-gnu/libprotobuf.so.17: error adding symbols: DSO missing from command line
collect2: error: ld returned 1 exit status
make[2]: *** [CMakeFiles/voxblox_eval.dir/build.make:469: /home/pol/Downloads/catkin_ws/devel/.private/voxblox_ros/lib/voxblox_ros/voxblox_eval] Error 1
make[1]: *** [CMakeFiles/Makefile2:2311: CMakeFiles/voxblox_eval.dir/all] Error 2
make: *** [Makefile:141: all] Error 2
```
Check the issue linked to the above: [#11](https://github.com/ntnu-arl/gbplanner_ros/issues/11)