#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class Controller:
    """
        Converts linear velocity and angular velocity into wheel velocities
    """
    def __init__(self):
        rospy.init_node('control_x80sv')
        self.left_speed = rospy.Publisher('/x80sv/left_controller/command', Float64)
        self.right_speed = rospy.Publisher('/x80sv/right_controller/command', Float64)
        rospy.Subscriber('/cmd_vel', Twist, self.handle_twist)

    def handle_twist(self, twist):
        wheel_base = 0.4
        v = twist.linear.x
        omega = twist.angular.z
        v_l = v - omega * (wheel_base / 2)
        v_r = v + omega * (wheel_base / 2)
        self.left_speed.publish(Float64(v_l))
        self.right_speed.publish(Float64(v_r))

    def control(self):
        rospy.spin()

if __name__ == '__main__':
    controller = Controller()
    controller.control()
