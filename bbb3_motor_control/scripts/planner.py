#!/usr/bin/env python

''' A simple planner: alternatively set the target velocity for motor A to 0 and 2.5.
'''

import rospy
from std_msgs.msg import Float32


# Parameters
period = 10

# Global variables
pub = None
v = 0

def timer_callback(dummy):
    global v
    if v==0:
        v = 2.5
    else:
        v = 0
    pub.publish(v)

def planner():
    global pub
    rospy.init_node('planner')
    pub = rospy.Publisher('target_vel', Float32)
    rospy.Timer(rospy.Duration(0.5*period), timer_callback)
    rospy.spin()


if __name__ == '__main__':
    planner()
