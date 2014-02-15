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


# Pinout parameters
channel_pwm = {'a': 'P9_14', 'b': 'P9_16'}
channel_dir_in1 = {'a': 'P8_07', 'b': 'P8_08'}
channel_dir_in2 = {'a': 'P8_09', 'b': 'P8_10'}


def setDutyCicle(m, val):
    if val<-1 or val>1:
        rospy.logerr('Ignoring out of range value ' + val)

    if val>0:
        GPIO.output(channel_dir_in1[m], GPIO.LOW)
        GPIO.output(channel_dir_in2[m], GPIO.HIGH)
        PWM.set_duty_cycle(channel_pwm[m], val*100)
    elif val<0:
        GPIO.output(channel_dir_in1[m], GPIO.HIGH)
        GPIO.output(channel_dir_in2[m], GPIO.LOW)
        PWM.set_duty_cycle(channel_pwm[m], -val*100)
    else:
        GPIO.output(channel_dir_in1[m], GPIO.LOW)
        GPIO.output(channel_dir_in2[m], GPIO.LOW)
        PWM.set_duty_cycle(channel_pwm[m], 0)
    

def callbackA(msg):
    setDutyCicle('a', msg.data)


def callbackB(msg):
    setDutyCicle('b', msg.data)


def motor():
    rospy.init_node('motor_driver')

    # setup channels
    PWM.start(channel_pwm['a'], 0)
    PWM.start(channel_pwm['b'], 0)
    GPIO.setup(channel_dir_in1['a'], GPIO.OUT)
    GPIO.setup(channel_dir_in1['b'], GPIO.OUT)
    GPIO.setup(channel_dir_in2['a'], GPIO.OUT)
    GPIO.setup(channel_dir_in2['b'], GPIO.OUT)
    GPIO.output(channel_dir_in1['a'], GPIO.LOW)
    GPIO.output(channel_dir_in2['a'], GPIO.LOW)
    GPIO.output(channel_dir_in1['b'], GPIO.LOW)
    GPIO.output(channel_dir_in2['b'], GPIO.LOW)

    # subscribe
    rospy.Subscriber('motor_a', Float32, callbackA)
    rospy.Subscriber('motor_b', Float32, callbackB)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    
if __name__ == '__main__':
    print 'REMINDER: this needs to be run as root'
    motor()