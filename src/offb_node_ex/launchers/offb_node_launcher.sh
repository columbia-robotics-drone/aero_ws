#!/bin/bash

gnome-terminal	 		\
   	--tab --title "px4 posix_sitl" --command "bash -c \"
cd ~/src/Firmware &&
make posix_sitl_default &&
source ~/catkin_ws/devel/setup.bash &&
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build_posix_sitl_default &&
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd) &&
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo &&
roslaunch px4 posix_sitl.launch;
    exec bash\"" \
	--tab -e --title "mavros"	--command "bash -c \"
roslaunch mavros px4.launch fcu_url:='udp://:14540@127.0.0.1:14557';
	exec bash\""  \
    --tab -e --title "offb_node" --command "bash -c \"
rosrun offb_node_ex offb_node_ex_node;
	exec bash\"" \