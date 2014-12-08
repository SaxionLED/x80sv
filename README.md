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

- Setup catkin workspace
_http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment_  

- Setup .bashrc  
  - Source the workspace  
    ___source ~/catkin_ws/devel/setup.bash___  

- Install dependencies:
    - apt-get install ros-indigo-global-planner ros-indigo-gmapping ros-indigo-dwa-local-planner ros-indigo-move-base ros-indigo-controller-manager ros-indigo-base-local-planner ros-indigo-costmap-2d

- Clone skynav and x80sv in the catkin_ws/src folder. (or clone somewhere else (**~/GIT/**) and create a symbolic link to the files in the catkin_ws/src folder.)
  
	- for the navigation software: 
>_$ git clone https://github.com/SaxionLED/skynav.git_  
	
  - for the x80sv robot drivers: 
>_$ git clone https://github.com/SaxionLED/x80sv.git_  
  
- Setup serial ports to right setup and add useraccount to 'dialout' group   
>_$ useradd -G dialout [USER]_    
  
- Export the files from __x80sv/x80sv_driver/udev__ folder to __/etc/udev/rules.d__   
	  for resolving the robot specific USB connections. These include usb to serial for controllerboard
      and the lrs. Also the blink1 status led.

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
	
