#!/usr/bin/env python

'''
Controls the PWM signal to smoothly vary the speed of the motor.

Circuit:
--------

GND  P9_02  Ground
VCC  P9_04  Power for the microcontroller, 2.7 V to 5.5 V
VM   P9_06  Power for the motors, 4.5 V to 13.5 V   (no power supply -> use VCC)
PWMA P9_14  PWM input for motor A
AIN1 GND    direction control
AIN2 VCC    direction control
STBY VCC    stand by

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
import time


pwm = 'P9_14'
duty_cycle_max = 50
period = 5
loop_period = .1


print 'REMINDER: this needs to be run as root'
print 'Setting PWM on ' + pwm


increment = duty_cycle_max * 2 / (period / loop_period)
direction = 1
duty_cycle = 0

#PWM.start(channel, duty, freq=2000, polarity=0)
PWM.start(pwm, duty_cycle)

while True:
    PWM.set_duty_cycle(pwm, duty_cycle)

    duty_cycle = duty_cycle + direction * increment;
    if duty_cycle >= duty_cycle_max:
        duty_cycle = duty_cycle_max
        direction = -direction
    elif duty_cycle <= 0:
        duty_cycle = 0
        direction = -direction
    
    time.sleep(loop_period)
