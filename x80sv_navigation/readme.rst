
This package contains the required files to launch the ros package move_base on the x80sv robot.


To launch this package, first launch the x80sv (either simulated or real variant).

.. code:: bash

  $ roslaunch x80sv_bringup sim_world.launch


Then launch the x80sv_navigation.launch


.. code:: bash

    $ launch x80sv_navigation x80sv_navigation.launch
