#!/usr/bin/env python

# Python part:
import sys
import os
import subprocess
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
black = 0, 0, 0


class StatusLed:
    """ Uses a blink(1) to indicate robot state """
    required_nodes = {'neato_laser_publisher', 'x80_robot'}

    def __init__(self):
        rospy.loginfo('statusled')
        self.diagnostic_sub = rospy.Subscriber('diagnostics', DiagnosticArray, self.on_diagnostic)
        self.state = False  # True if all diagnostics are OK
        self.diags = {}
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
            self.diags[status.name] = status.level
        self.state = all(status == 0 for status in self.diags.values())

    def check_nodes(self):
        """ Check if all required nodes are alive """
        f = lambda x: x[1:]
        nodes = set(f(n) for n in rosnode.get_node_names())
        self.alive = issubset(self.required_nodes, nodes)

    def update_led(self):
        # rospy.loginfo("Nodes alive: {} diags: {}".format(self.alive, self.diags))
        if self.toggle:
            if self.alive and self.state:
                color = green
            else:
                color = red
        else:
            color = black
        self.set_led(*color)

    def set_led(self, r, g, b):
        exe = os.path.join(os.path.abspath(os.path.dirname(__file__)), 'blink1-mini-tool')
        cmd = [exe, 'rgb', ','.join(str(x) for x in [r,g,b])]
        res = subprocess.check_output(cmd, stderr=subprocess.STDOUT)

    def run(self):
        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.toggle = not self.toggle
            self.check_nodes()
            self.update_led()
            r.sleep()
        rospy.loginfo('Done!')
        
if __name__ == '__main__':
    rospy.init_node('statusled')
    statusled = StatusLed()
    statusled.run()

