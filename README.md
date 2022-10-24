# coordinated_motion ROS package

## Requirements
- ROS -- tested on Melodic, but other versions may work.
- catkin -- used for building the application. 
- turtlebot3
- turtlebot3_msgs

## Build
	cd catkin_ws/src
 	git clone git@github.com:chenmasterandrew/auction_motion.git
 	git clone git@github.com:ROBOTIS-GIT/turtlebot3.git
 	git clone git@github.com:ROBOTIS-GIT/turtlebot3_msgs.git
 	"follow instructions in PA3 to modify the robot model turtlebot3_waffle_pi (same file modified for PA2) and change from world to encoder"
	cd catkin_ws
	catkin_make
 	source catkin_ws/devel/setup.sh
	
## Run
	roslaunch auction_motion auction_motion.launch

## Attribution & Licensing

See LICENSE for further information
