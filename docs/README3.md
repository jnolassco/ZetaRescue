# CS354 Zeta Rescue Final Project

## Bob Saydahmat, Garrett Folks, Jason Nolasco, Zamua Nasrawt

YouTube Links:
	- Checkpoint1: https://youtu.be/AGY_NtcBAio
	- Checkpoint2: https://youtu.be/OrSAJ6zLaHA
	- Checkpoint3: https://youtu.be/siutKcsbzdQ

Current Project State (Checkpoint 3):
- rescue.py contains all core functionality.
- The robot wanders randomly for a fixed amount of time.
	- Occasionally the turtlebot erroneously tries to get "unstuck" and spins.
- Images of victims are taken and markers are made in rviz when "unique" victims are sensed.
- Victims are differentiated by distance (currently 2 meters).
- The turtlebot attempts to line up with the ar tag, before taking an image and placing a marker on a victim.
	- This is very buggy and inconsistent at the moment.
- Pressing the button prints the array of victims and their coordinates(x,y,z) in the "/map" coordinate frame.
	- Orientation was ommitted purposefully

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
	- ~~Line up with victim before taking picture~~ (inconsistent, needs further debugging)
	- ~~Path planning and optimization research~~
	- ~~User-friendly reporting~~
	- Path planning and optimization implementation
	- System testing
	
