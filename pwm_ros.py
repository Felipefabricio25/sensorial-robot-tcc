#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
import time
from geometry_msgs.msg import Twist

GPIO.setmode(GPIO.BCM)


motor_right_enable_pin = 9
motor_right_in_3_pin = 17
motor_right_in_4_pin = 22

motor_left_enable_pin =10
motor_left_in_1_pin = 23
motor_left_in_2_pin = 24

#Pinos motor esquerdo
GPIO.setup(motor_left_enable_pin, GPIO.OUT)
GPIO.setup(motor_left_in_1_pin, GPIO.OUT)
GPIO.setup(motor_left_in_2_pin, GPIO.OUT)
motor_pwm_left = GPIO.PWM(motor_left_enable_pin, 250)
motor_pwm_left.start(0)

#Pinos motor direito
GPIO.setup(motor_right_enable_pin, GPIO.OUT)
GPIO.setup(motor_right_in_3_pin, GPIO.OUT)
GPIO.setup(motor_right_in_4_pin, GPIO.OUT)
motor_pwm_right = GPIO.PWM(motor_right_enable_pin, 250)
motor_pwm_right.start(0)

def forward_left(duty):          
    #motor esquerdo
    GPIO.output(motor_left_in_1_pin, True) 
    GPIO.output(motor_left_in_2_pin, False) 
    motor_pwm_left.ChangeDutyCycle(duty)

def forward_right(duty):
    #motor direito
    GPIO.output(motor_right_in_3_pin, True) 
    GPIO.output(motor_right_in_4_pin, False) 
    motor_pwm_right.ChangeDutyCycle(duty)

def reverse_left(duty): 
    #motor esquerdo         
    GPIO.output(motor_left_in_1_pin, False) 
    GPIO.output(motor_left_in_2_pin, True) 
    motor_pwm_left.ChangeDutyCycle(duty)    

def reverse_right(duty):
    #motor direito         
    GPIO.output(motor_right_in_3_pin, False) 
    GPIO.output(motor_right_in_4_pin, True) 
    motor_pwm_right.ChangeDutyCycle(duty)    

def stop_left():
    #motor esquerdo
    GPIO.output(motor_left_in_1_pin, False) 
    GPIO.output(motor_left_in_2_pin, False) 
    motor_pwm_left.ChangeDutyCycle(0)
    
def stop_right():
    #motor direito
    GPIO.output(motor_right_in_3_pin, False) 
    GPIO.output(motor_right_in_4_pin, False) 
    motor_pwm_right.ChangeDutyCycle(0)
    
        
def callback_cmd_vel(data):
	rospy.loginfo(data)
	#velocidade linear
	if data.linear.x > 0:
		forward_right(100)
		forward_left(100)
	elif data.linear.x < 0:
		reverse_right(100)
		reverse_left(100)
	elif data.linear.x == 0:
		stop_right()
		stop_left()

	#velocidade angular	
	if data.angular.z > 0:
		forward_right(100)
		reverse_left(100)
	elif data.angular.z < 0:
		reverse_right(100)
		forward_left(100)

if __name__ == '__main__':
	rospy.init_node('controleCMD_VEL',  anonymous=True)
	rospy.Subscriber("cmd_vel", Twist, callback_cmd_vel)
	rospy.spin()

#	try
#
#
#	finally
#
#    print("Cleaning up")
#    GPIO.cleanup()

