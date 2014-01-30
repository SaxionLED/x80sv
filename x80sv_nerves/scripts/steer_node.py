#!/usr/bin/python
import math
from collections import namedtuple
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64
import tf

Point = namedtuple('Point', ['x', 'y'])
def distance(p1, p2):
    return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

def clipangle(a):
    if a > math.pi:
        return clipangle(a - 2*math.pi)
    if a < -math.pi:
        return clipangle(a + 2 *math.pi)
    return a


class Driver:
    def __init__(self):
        rospy.init_node('steer_node')
        # rospy.Subscriber()
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist)

    def handle_pose_estimage(self, pose):
        x_hat = pose.x, pose.y, pose.theta
        self.steer(x_hat)

    def do_steer(self, x_hat):
        if self.waypoints:
            cwp = self.waypoints[0]
            x, y, theta = x_hat
            cpos = Point(x, y)
            dist = distance(cpos, cwp)
            dx = cwp.x - cpos.x
            dy = cwp.y - cpos.y
            t_angle = math.arctan2(dy, dx)
            dangle = clipangle(t_angle - theta)
            u = 0.1*np.matrix([3.0, 4.0 * dangle]).transpose()
            if dist < 0.1:
                print('At waypoint!', self.waypoints.pop(0))
        else:
            u = np.matrix([[0.0], [0.0]])
        self.cmd_vel.publish(Twist())
        return u

    def steer(self):
        listener = tf.TransformListener()
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                (trans, rot) = listener.lookupTransform('/base_link', '/nav_target', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            print trans, rot
            rate.sleep()

if __name__ == '__main__':
    driver = Driver()
    driver.steer()
