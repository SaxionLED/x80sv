#!/usr/bin/env python

# Python part:
import sys
import os
import code
import readline
import rlcompleter

# Ros part:
import roslib
roslib.load_manifest('x80sv_driver')
import rospy
import rosnode
from diagnostic_msgs.msg import DiagnosticArray

def issubset(a, b):
    return len(a - b) == 0

red = 0x80, 0, 0
green = 0, 0x80, 0
orange = 0xff, 0x8c, 0
black = 0, 0, 0

class StatusLed:
    """ Uses a blink(1) to indicate robot state """
    required_nodes = {'neato_laser_publisher', 'x80_robot'}

    def __init__(self):
        rospy.loginfo('statusled')
        self.diagnostic_sub = rospy.Subscriber('diagnostics', DiagnosticArray, self.on_diagnostic)
        self.state = False
        # self.interact()
        self.toggle = False

    def interact(self):
        """ Start interactive shell at current point in time """
        vrs = globals()
        readline.set_completer(rlcompleter.Completer(vrs).complete)
        readline.parse_and_bind('tab: complete')
        shell = code.InteractiveConsole(vrs)
        shell.interact()

    def on_diagnostic(self, data):
        for status in data.status:
            self.state = status.level == 0

    def check_nodes(self):
        """ Check if all required nodes are alive """
        f = lambda x: x[1:]
        nodes = set(f(n) for n in rosnode.get_node_names())
        self.alive = issubset(self.required_nodes, nodes)

    def update_led(self):
        state = self.alive and self.state
        rospy.loginfo("State: alive: {} state: {}".format(self.alive, self.state))
        if self.toggle:
            if self.alive:
                if self.state:
                    color = green
                else:
                    color = orange
            else:
                color = red
        else:
            color = black
        self.set_led(*color)

    def set_led(self, r, g, b):
        exe = os.path.join(os.path.dirname(__file__), 'blink1-mini-tool')
        os.system('{} rgb {},{},{}'.format(exe, r, g, b))

    def run(self):
        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.check_nodes()
            self.toggle = not self.toggle
            self.update_led()
            r.sleep()
        rospy.loginfo('Done!')
        
if __name__ == '__main__':
    rospy.init_node('statusled')
    statusled = StatusLed()
    statusled.run()

