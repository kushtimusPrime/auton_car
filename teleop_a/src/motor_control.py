#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, String

import RPi.GPIO as GPIO

class MotorControl:

	def _init_(self):
		GPIO.setmode(GPIO.BCM)

		motor_p=17
		motor_n=22
		motor_enable=25

		GPIO.setup(motor_p,GPIO.OUT)
		GPIO.setup(motor_n,GPIO.OUT)
		GPIO.setup(motor_enable,GPIO.OUT)

		pwm=GPIO.PWM(motor_enable,1000)

		pwm.start(0)
		GPIO.output(motor_p,GPIO.HIGH)
		GPIO.output(motor_n,GPIO.LOW)
		GPIO.output(motor_enable,GPIO.HIGH)

		self.left_motor = rospy.Subscriber('tl_duty_cycle', Float64,self.callback)

	def callback(self,data):
		pwm.ChangeDutyCycle(data.data)

if __name__ == '__main__':

	rospy.init_node('motor_control')
	
	control=MotorControl()

	rospy.loginfo("Motor Control intialized...")

	rospy.spin()

	GPIO.output(motor_enable,GPIO.LOW)

	pwm.stop()

	GPIO.cleanup()