#!/usr/bin/env python

import random
import roslib
roslib.load_manifest('x80sv_navigation')
import rospy
import actionlib
import tf

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

def rand(lower, upper):
    return random.random() * (upper - lower) + lower

if __name__ == '__main__':
    rospy.init_node('drive_random')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    while True:
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.get_rostime()
        goal.target_pose.pose.position.x = rand(-5, 5)
        goal.target_pose.pose.position.y = rand(-5, 5)
        goal.target_pose.pose.orientation.w = 1
        client.send_goal(goal)
        rospy.loginfo('Waiting for result')
        client.wait_for_result()
        state = client.get_state()
        print(state)
        if state == 0:
            break
