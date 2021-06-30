# YDLIDAR ROS Package Download and Build
## Step1: create a ROS workspace, if there is no workspace, othereise Skip to Step2
#### Linux/OS X
	$mkdir -p ~/ydlidar_ros_ws/src
	$cd ~/ydlidar_ros_ws/src
#### Windows
	$md \dev\ydlidar_ros_ws\src
	$cd \dev\ydlidar_ros_ws\src
	
## Step2: clone ydlidar ros package
	$git clone https://github.com/YDLIDAR/ydlidar_ros
	
## Step3: Build [ydlidar_ros](https://github.com/YDLIDAR/ydlidar_ros) package
	$cd ..
	$catkin_make
	
Note: Set ROS Workspace Environment Variables

	$echo "source ~/ydlidar_ros_ws/devel/setup.bash" >> ~/.bashrc
	$source ~/.bashrc
	
## Step4:Configure LiDAR [paramters](../launch/lidar.launch)
	<launch>
  	<node name="ydlidar_node"  pkg="ydlidar_ros"  type="ydlidar_node" output="screen" respawn="false" >
    		<param name="port"         type="string" value="/dev/ydlidar"/>  
    		<param name="baudrate"         type="int" value="230400"/>  
    		<param name="frame_id"     type="string" value="laser_frame"/>
    		<param name="resolution_fixed"    type="bool"   value="true"/>
    		<param name="auto_reconnect"    type="bool"   value="true"/>
    		<param name="reversion"    type="bool"   value="true"/>
    		<param name="angle_min"    type="double" value="-180" />
    		<param name="angle_max"    type="double" value="180" />
    		<param name="range_min"    type="double" value="0.1" />
    		<param name="range_max"    type="double" value="16.0" />
    		<param name="ignore_array" type="string" value="" />
    		<param name="frequency"    type="double" value="10"/>
    		<param name="isTOFLidar"    type="bool"   value="false"/>
 	 </node>
  	<node pkg="tf" type="static_transform_publisher" name="base_link_to_laser4"
    args="0.2245 0.0 0.2 0.0 0.0  0.0 /base_footprint /laser_frame 40" />
	</launch>

Note: How to configure paramters, see [here](paramters.md)
  
## Step5:Create serial port Alias[/dev/ydlidar] 
	$chmod 0777 src/ydlidar_ros/startup/*
	$sudo sh src/ydlidar_ros/startup/initenv.sh
Note: After completing the previous operation, replug the LiDAR again.
  
## Step6:Run ydlidar_ros node
	
	$roslaunch ydlidar_ros lidar.launch

	$rosrun ydlidar_ros ydlidar_client
	 
or 

	$roslaunch ydlidar_ros lidar_view.launch

