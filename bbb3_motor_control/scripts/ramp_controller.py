#!/usr/bin/env python

''' Speed controller.

Listens to target velocity and outputs a ramp from current velocity to
target velocity for the motor driver.
'''

import rospy
from std_msgs.msg import Float32


# Parameters
# duration (in seconds) of a ramp from complete stop to full speed (i.e. PWM from 0 to 100)
full_ramp_duration = 3
# velocity factor: how many m/s per motor driver unit (1 is full speed)
velocity_factor = 5
max_speed = 2.5
period = .1


# Global variables
pub = None
current_u = 0
target_u = 0
increment = 1.0 / full_ramp_duration * period


def v_callback(msg):
    global target_u
    target_v = msg.data
    if target_v > max_speed:
        target_v = max_speed
    elif target_v < -max_speed:
        target_v = -max_speed
    target_u = target_v / velocity_factor

        
def timer_callback(dummy):
    global current_u
    if target_u > current_u:
        current_u = min([current_u + increment, target_u])
    elif target_u < current_u:
        current_u = max([current_u - increment, target_u])
    pub.publish(current_u)

        
def ramp_controller():
    global pub
    rospy.init_node('ramp_controller', anonymous=True)
    pub = rospy.Publisher('drive', Float32)
    rospy.Timer(rospy.Duration(period), timer_callback)
    rospy.Subscriber('target_vel', Float32, v_callback)
    rospy.spin()


if __name__ == '__main__':
    ramp_controller()
