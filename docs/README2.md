# CS354 Zeta Rescue Final Project

## Bob Saydahmat, Garrett Folks, Jason Nolasco, Zamua Nasrawt

YouTube Links:
	- Checkpoint1: https://youtu.be/AGY_NtcBAio
	- Checkpoint2: https://youtu.be/OrSAJ6zLaHA

Current Project State (Checkpoint 2):
- rescue.py contains all core functionality.
- The robot wanders randomly for a fixed amount of time.
	- Occasionally the turtlebot erroneously tries to get "unstuck" and spins.
- Images of victims are taken and markers are made in rviz when "unique" victims are sensed.
- Victims are differentiated by distance (currently 2 meters).

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
	- ~~Debugging current systems~~
	- ~~AR Tag observation~~
	- ~~Taking pictures~~
	- Line up with victim before taking picture
	- Path planning and optimization research
	- Path planning and optimization implementation
	- User-friendly reporting
	- System testing
	
