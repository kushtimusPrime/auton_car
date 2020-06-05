#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, String

import RPi.GPIO as GPIO

#Set GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

#Set variables for GPIO Pin Motors
MOTOR_P=17
MOTOR_N=22
MOTOR_ENABLE=25
FREQUENCY=1000

#Set GPIO pins to be outputs
GPIO.setup(MOTOR_P,GPIO.OUT)
GPIO.setup(MOTOR_N,GPIO.OUT)
GPIO.setup(MOTOR_ENABLE,GPIO.OUT)

#Set pwm controller and start it
pwm=GPIO.PWM(MOTOR_ENABLE,FREQUENCY)
pwm.start(0)
GPIO.output(MOTOR_P,GPIO.HIGH)
GPIO.output(MOTOR_N,GPIO.LOW)
GPIO.output(MOTOR_ENABLE,GPIO.HIGH)

#Message Handler
def left_callback(data):

    hi=data.data+1
    pwm.ChangeDutyCycle(data.data)

#Initialize as ROS node
rospy.init_node('motor_control_b')
rospy.Subscriber("tl_duty_cycle",Float64,left_callback)
# Ready to go
rospy.loginfo("Motor Control B initialized...")

# Loop continuously
rospy.spin()

GPIO.output(MOTOR_ENABLE,GPIO.LOW)
pwm.stop()
GPIO.cleanup()
