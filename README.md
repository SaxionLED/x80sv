x80sv
=====

Container for x80sv description, simulation and drivers


[![Build Status](https://travis-ci.org/SaxionLED/x80sv.svg)](https://travis-ci.org/SaxionLED/x80sv)


-------------------
quick install notes
-------------------

- Install ubuntu 14.04-LTS  
_http://www.ubuntu.com/_

- Install ros indigo
_http://www.ros.org/install/_  

- In your user directory setup a ROS catkin workspace
_http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment_  

- Setup .bashrc to automatically load the workspace:
  - Source the workspace  
    ___source ~/catkin_ws/devel/setup.bash___  

- Install dependencies:
    - apt-get install ros-indigo-global-planner ros-indigo-gmapping ros-indigo-dwa-local-planner ros-indigo-move-base ros-indigo-controller-manager ros-indigo-base-local-planner ros-indigo-costmap-2d

- Clone skynav and x80sv in the catkin_ws/src folder. (or clone somewhere else (**~/GIT/**) and create a symbolic link to the files in the catkin_ws/src folder.)
  
	- for the navigation software: 
>_:~/catkin_ws/src $ git clone https://github.com/SaxionLED/skynav.git_  
	
  - for the x80sv robot drivers: 
>_:~/catkin_ws/src $ git clone https://github.com/SaxionLED/x80sv.git_  

- Build the project  
>_:~/catkin_ws $ catkin_make_  
 
- (Optional) Build and run the gtests and rostests (only if you want to run the testsuite)
>_:~/catkin_ws $ catkin_make run_tests_

- It is convenient to add the following line to your .bashrc:
>_ _
  
- Only the x80sv laptop:
  - Setup serial ports to right setup and add useraccount to 'dialout' group   
>_$ useradd -G dialout [USER]_    
  
  - Copy the files from __x80sv/x80sv_driver/udev__ to __/etc/udev/rules.d__ (if not already present)
      this is required for resolving the robot specific USB connections. These include usb to serial 
      for controllerboard,
      the laser range sensor and the blink1 status led.

----------
Simulation
----------

- Run the simulated version of the robot:
>_:~/catkin_ws $ roslaunch x80sv_bringup sim_world.launch_

- Launch rviz and add views to visualize the robot:
>_:~/catkin_ws $ rviz_

- Launch rqt and use the robot steering plugin to steer the robot:
>_:~/catkin_ws $ rqt_

- Launch the navigation system for the x80sv:
>_:~/catkin_ws $ roslaunch x80sv_navigation x80sv_navigation.launch_

----------
Quick demo
----------

- Install all software (as described above) on the x80sv laptop (if not already done).

- Launch the robot drivers on the robot laptop:
>_x80sv:~/catkin_ws $ roslaunch x80sv_bringup real_robot.launch_

- Launch the navigation system for the x80sv on the robot laptop:
>_x80sv:~/catkin_ws $ roslaunch x80sv_navigation x80sv_navigation.launch_

- On the host pc make sure that the ROS_MASTER_URI is set to point to the robot:
>_hostpc:~/catkin_ws $ env | grep ROS_MASTER_URI_
and if not, set this environment variable:
>_hostpc:~/catkin_ws $ export ROS_MASTER_URI=http://x80sv:11311/_
It may be handy to add this export to your bashrc.

- Make sure that on both hostpc and x80sv laptop, the /etc/hosts file is setup
  such that hostnames are resolved correctly.
