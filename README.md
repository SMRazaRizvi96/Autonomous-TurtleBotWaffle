# Sofar Project

##### In this repository, you can find:

The folder sofar_project which contains the packages, planning, slam_gmapping, sofar_project, turtlebot3, turtlebot3_msgs, turtlebot3_simulations.

The package sofar_project package contains the file sofar.py which contains the code for the modules Rigid Body Detector, Robot Controller, and path planning.
This folder also contains the launch file and the custom message folder.

### Data required from other modules:

    • Final target as a coordinate published on move_base server.

### Installation and Running Procedures:

For running the three modules, first clone the entire repository that will also include the supporting packages; ‘planning’ and ‘gmapping’.

Then install the sparse library (sudo apt-get install libsuitesparse-dev) and openslam_gmapping (sudo apt-get install ros-<ros_distro>-openslam-gmapping).

Install the ROS navigation stack (sudo apt-get install ros-<ros_distro>-navigation).

Open the terminal and run 'catkin_make' inside your ROS workspace

Open the Terminal and go to the 'Scripts' folder of the main package ‘sofar_project’

##### Give running permissions to the code with: 

'chmod +x sofar.py'

##### Run the launch file

' roslaunch sofar_project sofLaunch.launch '


### Contributors:

Aliya Arystanbek
Laiba Zahid
Syed Muhammad Raza Rizvi
