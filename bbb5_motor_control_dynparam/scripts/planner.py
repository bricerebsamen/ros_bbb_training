#!/usr/bin/env python

''' A simple planner: alternatively set the target velocity for motor A to 0 and 2.5.
'''

import rospy
from std_msgs.msg import Float32

from dynamic_reconfigure.server import Server
from bbb5_motor_control_dynparam.cfg import PlannerConfig

# Parameters
period = None

# Global variables
pub = None
timer = None
v = 0

def config_callback(config, level):
    global period, timer
    if period!=config.period:
        period = config.period
        if timer is not None:
            timer.shutdown()
        timer = rospy.Timer(rospy.Duration(0.5*period), timer_callback)
    return config

def timer_callback(dummy):
    global v
    if v==0:
        v = 2.5
    else:
        v = 0
    pub.publish(v)

def planner():
    global pub, timer
    rospy.init_node('planner')
    srv = Server(PlannerConfig, config_callback)
    pub = rospy.Publisher('target_vel', Float32)
    rospy.spin()


if __name__ == '__main__':
    planner()
