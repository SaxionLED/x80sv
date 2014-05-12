
Simulation of the x80sv
=======================

This ROS package contains simulation scenarios for the x80sv robot. The launch file will load an
empty world, load controllers and launch a file that listens to the /cmd_vel topic.

To launch the simulation of the x80sv run:

  roslaunch x80sv_simulation simulation.launch

You will then see this:


.. image:: gazebo_gmapping.png


Offline models
--------------

For offline usage of the downloaded object models, the local model database should be used by gazebo.

This can be done by referencting the local gazebo model path database, in this case:
> <dl>export GAZEBO_MODEL_PATH=~/catkin_ws/src/x80sv/x80sv_simulation/models/</dl>

Models can be downloaded from the Gazebo model database [bitbucket](https://bitbucket.org/osrf/gazebo_models)
