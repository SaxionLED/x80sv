
This package controls the x80sv robot

To launch the simulation of the x80sv run:

  roslaunch groundcontrol_gui simulation.launch

You will then see this:


.. image:: gazebo_gmapping.png


To launch the gmapping run:

  roslaunch groundcontrol_gui intelligence.launch

This constructs the correct TF sequence:

.. image:: gmapping_frames.png

Then launch rviz:

.. image:: rviz_gmapping.png
