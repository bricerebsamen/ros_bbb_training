#!/usr/bin/env python

''' Motor driver.

Circuit:
--------

VM   P9_06  Power for the motors, 4.5 V to 13.5 V
VCC  P9_04  Power for the microcontroller, 2.7 V to 5.5 V
GND  P9_02  Ground
STBY VCC    stand by

PWMA P9_14  PWM input for motor A
AIN1 GND    direction control
AIN2 VCC    direction control
A01  motor A (-) lead
A02  motor B (+) lead

AIN1  AIN2  PMW  motor
 0     0     X    0
 X     X     0    0
 0     0     X    0
 0     1    >0    +
 1     0    >0    -
 1     1     X    0
'''

import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO
import rospy
from std_msgs.msg import Float32


# Pinout (from parameters)
channel_pwm = ''
channel_dir_in1 = ''
channel_dir_in2 = ''

    

def callback(msg):
    val = msg.data
    
    if val<-1 or val>1:
        rospy.logerr('Ignoring out of range value ' + val)

    if val>0:
        GPIO.output(channel_dir_in1, GPIO.LOW)
        GPIO.output(channel_dir_in2, GPIO.HIGH)
        PWM.set_duty_cycle(channel_pwm, val*100)
    elif val<0:
        GPIO.output(channel_dir_in1, GPIO.HIGH)
        GPIO.output(channel_dir_in2, GPIO.LOW)
        PWM.set_duty_cycle(channel_pwm, -val*100)
    else:
        GPIO.output(channel_dir_in1, GPIO.LOW)
        GPIO.output(channel_dir_in2, GPIO.LOW)
        PWM.set_duty_cycle(channel_pwm, 0)
        

def setup():
    PWM.start(channel_pwm, 0)
    GPIO.setup(channel_dir_in1, GPIO.OUT)
    GPIO.setup(channel_dir_in2, GPIO.OUT)
    GPIO.output(channel_dir_in1, GPIO.LOW)
    GPIO.output(channel_dir_in2, GPIO.LOW)
    

def getParameters():
    global channel_pwm, channel_dir_in1, channel_dir_in2
    # see http://wiki.ros.org/rospy/Overview/Parameter%20Server
    # no default provided --> parameters are mandatory
    
    channel_pwm = rospy.get_param('~pwm')
    channel_dir_in1 = rospy.get_param('~dir1')
    channel_dir_in2 = rospy.get_param('~dir2')
    
    
def motor():
    rospy.init_node('motor_driver')
    getParameters()
    setup()

    # subscribe
    rospy.Subscriber('motor', Float32, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    
if __name__ == '__main__':
    print 'REMINDER: this needs to be run as root'
    motor()