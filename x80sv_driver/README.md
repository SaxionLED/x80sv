x80sv_driver
=============

Contains drivers for the x80sv robot.
Contains sensor_safety module that switches range finder sensors based on the situation.

Contains a wrapper specifically for the DrRobot X80SV that converts sensor data and actuation data to be usable by the skynav_control package.



To install the package, make sure to install the correct udev rules.

Copy the two rule files from the udev folder to the following directory:

 $ sudo cp udev/*.rules /etc/udev/rules.d

/etc/udev/rules.d/

This should yield two new symlinks to the correct ports must be present:

 $ ls -l /dev/tty*

- /dev/ttyROBOT
- /dev/ttyLASER
