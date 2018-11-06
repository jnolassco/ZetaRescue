# CS354 Zeta Rescue Final Project

## Bob Saydahmat, Garrett Folks, Jason Nolasco, Zamua Nasrawt

YouTube Link:
	- https://youtu.be/AGY_NtcBAio

Project State:
- rescue.py currently contains all core functionality.
- The robot wanders randomly for a fixed amount of time.
	- Occasionally the turtlebot erroneously tries to get "unstuck" and spins.
- AR tags don't seem to be detected even after subscribing to the (supposedly) correct topics.
	- May be an issue with launch file.
- After TIME_LIMIT seconds, the turtlebot returns to the starting position as determined by the initial_pose.yaml file.

Execution Instructions: 
- Clone the repository into ~/catkin_ws/src
	- $ cd ~/catkin_ws/src
	- $ git clone https://github.com/JMU-CS354-F17/final-project-sprague-bots.git
- Move into workspace directory and build package
	- $ cd ~/catkin_ws
	- $ catkin_make
- Configure ROS environment for new packages and update the ROS package list.
	- $ source devel/setup.bash
	- $ rospack profile
- Launch the rescue procedure
	- $ roslaunch zeta_rescue rescue.launch 
- Launch the rescue.py script
	- $ python src/final-project-sprague-bots/scripts/rescue.py

Development Plan:
- Split independent tasks up amongst group mates.
	- Debugging current systems
	- AR Tag observation
	- Taking pictures
	- Path planning and optimization research
	- Path planning and optimization implementation
	- User-friendly reporting
	- System testing
	
