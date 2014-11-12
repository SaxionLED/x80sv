x80sv
=====

Container for x80svn description, simulation and drivers





[![Build Status](https://travis-ci.org/SaxionLED/x80sv.svg)](https://travis-ci.org/SaxionLED/x80sv)


-------------------
quick install notes:
-------------------
- Install ubuntu 14.04-LTS  
_http://www.ubuntu.com/_

- Install ros indigo
_http://www.ros.org/install/_  
_http://wiki.ros.org/indigo/Installation/Ubuntu_

- Setup catkin workspace _/home/[USER]/catkin_ws_  
_http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment_  

- Setup .bashrc  
  - Source the workspace  
    ___source ~/catkin_ws/devel/setup.bash___  
  - Set gazebo model path for offline gazebo models in the model database  
    ___export GAZEBO_MODEL_PATH=~/catkin_ws/src/x80sv/x80sv_simulation/models/___
  - Set automatic cd to catkin_ws  
    ___roscd___  
    ___cd___ ___..___ 

- Git clone skynav and x80sv in the catkin_ws/src folder,  
	_(or clone somewhere else (**~/GIT/**) and create a symbolic link to the files in the catkin_ws/src folder.)_  
  
	- for the navigation software: 
>_$ git clone https://github.com/SaxionLED/skynav.git_  
	
  - for the x80sv robot drivers: 
>_$ git clone https://github.com/SaxionLED/x80sv.git_  
  
- Setup serial ports to right setup and add useraccount to 'dialout' group   
>_$ useradd -G dialout [USER]_    
  
- Export the files from __x80sv/x80sv_driver/udev__ folder to __/etc/udev/rules.d__   
	  for resolving the robot specific USB connections

- RUN
  - To build the project  
>_:~/catkin_workspace $ catkin_make_  
 
  - To build and run the gtests and rostests 
>_:~/catkin_workspace $ catkin_make run_tests_   
 
- Connect the robot and laser via usb,  or run a robot simulator in gazebo  

- RUN
  - For launching the robot drivers   
>_:~/catkin_workspace $ roslaunch x80sv_bringup real_robot.launch_  
 
  - Or launch the full skynav navigation package and the gmapping and x80 drivers for full robot navigational control 
>_:~/catkin_workspace $ roslaunch x80sv_bringup skynav_real_robot.launch_
	
