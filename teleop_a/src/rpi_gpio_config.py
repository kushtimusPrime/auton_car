#!/usr/bin/env python

import rospy
import message_filters
from std_msgs.msg import String,Float64

import RPI.GPIO as GPIO

#Set GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Set variables for the GPIO motor pins
pin_tl = 10
pin_tr = 9
pin_bl = 8
pin_br = 7

# How many times to turn the pin on and off each second
Frequency = 20

# Set the GPIO Pin mode to be Output
GPIO.setup(pin_tl, GPIO.OUT)
GPIO.setup(pin_tr, GPIO.OUT)
GPIO.setup(pin_bl, GPIO.OUT)
GPIO.setup(pin_br, GPIO.OUT)

# Set the GPIO to software PWM at 'Frequency' Hertz
pwm_tl = GPIO.PWM(pin_tl,Frequency)
pwm_tr = GPIO.PWM(pin_tr,Frequency)
pwm_bl = GPIO.PWM(pin_bl,Frequency)
pwm_br = GPIO.PWM(pin_br,Frequency)

# Start the software PWM with a duty cycle of 0
pwm_tl.start(0)
pwm_tr.start(0)
pwm_bl.start(0)
pwm_br.start(0)

def callback(tl_duty,tr_duty,bl_duty,br_duty):
    pwm_tl.ChangeDutyCycle(tl_duty)
    pwm_tr.ChangeDutyCycle(tr_duty)
    pwm_bl.ChangeDutyCycle(bl_duty)
    pwm_br.ChangeDutyCycle(br_duty)

rospy.init_node("rpi_gpio_config")

tl_duty_sub = message_filters.Subscriber("tl_duty_cycle",Float64)
tr_duty_sub = message_filters.Subscriber("tr_duty_cycle",Float64)
bl_duty_sub = message_filters.Subscriber("bl_duty_cycle",Float64)
br_duty_sub = message_filters.Subscriber("br_duty_cycle",Float64)

ts = message_filters.TimeSynchronizer([tl_duty_sub,tr_duty_sub,bl_duty_sub,br_duty_sub],10)
ts.registerCallback(callback)

rospy.spin()

pwm_tl.ChangeDutyCycle(0)
pwm_tr.ChangeDutyCycle(0)
pwm_bl.ChangeDutyCycle(0)
pwm_br.ChangeDutyCycle(0)
GPIO.cleanup()
