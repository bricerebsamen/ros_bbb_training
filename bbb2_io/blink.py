#!/usr/bin/env python

import Adafruit_BBIO.GPIO as GPIO
import time

port = 'P8_10'

print 'REMINDER: this needs to be run as root'

print 'Blinking LED on ' + port
GPIO.setup(port, GPIO.OUT)

while True:
    GPIO.output(port, GPIO.HIGH)
    print 'ON'
    time.sleep(0.5)
    GPIO.output(port, GPIO.LOW)
    print 'OFF'
    time.sleep(0.5)

