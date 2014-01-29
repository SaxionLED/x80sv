#!/usr/bin/python
import math
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import tf


class OdomEmitter:
    """ Emits odometry message. This is the spot rate of motion. """
    def __init__(self):
        rospy.init_node('odom_emitter')
        self.odom = rospy.Publisher('/odom', Odometry)
        self.odom_broadcaster = tf.TransformBroadcaster()
        rospy.Subscriber('/x80sv/joint_states', JointState, self.handle_joint_states)
        self.wheel_base = 0.2
        self.wheel_radius = 0.05
        self.th = 0.0
        self.x = 0.0
        self.y = 0.0
        self.prev_time = None

    def handle_joint_states(self, states):
        # Extract left and right velocities and emit odometry based on these facts.
        v_l = states.velocity[0] * self.wheel_radius
        v_r = states.velocity[1] * self.wheel_radius
        self.emit(v_l, v_r)

    def emit(self, v_l, v_r):
        """ Emit pose of base_link in odom frame """
        nu = rospy.Time.now()
        if self.prev_time is None:
            self.prev_time = nu
            return
        dt = nu - self.prev_time
        self.prev_time = nu
        dt = dt.to_sec()   # Convert to floating point
        # calculate velocities:
        vy = 0.0 # We cannot move instant sideways
        vx = (v_l + v_r) / 2.0   # Average left and right velocities
        vth = (v_r - v_l) / (self.wheel_base) # Rotation

        # Integrate:
        delta_x = (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
        delta_y = (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt
        delta_th = vth * dt

        # Update angles:
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        odom_quat = tf.transformations.quaternion_from_euler(0.0, 0.0, self.th)

        # Send transform:
        self.odom_broadcaster.sendTransform((self.x, self.y, 0.0), odom_quat, nu, "base_link", "odom")

        # Send odom message:
        msg = Odometry()
        msg.header.stamp = nu
        msg.header.frame_id = 'odom'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = odom_quat[0]
        msg.pose.pose.orientation.y = odom_quat[1]
        msg.pose.pose.orientation.z = odom_quat[2]
        msg.pose.pose.orientation.w = odom_quat[3]
        msg.child_frame_id = 'base_link'
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.angular.z = vth

        self.odom.publish(msg)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    oe = OdomEmitter()
    oe.run()
