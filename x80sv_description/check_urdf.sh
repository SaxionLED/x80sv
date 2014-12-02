#!/bin/bash

rosrun xacro xacro.py urdf/x80sv.xacro -o /tmp/x80sv.urdf
check_urdf /tmp/x80sv.urdf


