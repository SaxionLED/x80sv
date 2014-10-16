#!/bin/bash

rosrun xacro xacro.py urdf/x80sv.xacro
rosrun xacro xacro.py urdf/x80sv.xacro | check_urdf -


