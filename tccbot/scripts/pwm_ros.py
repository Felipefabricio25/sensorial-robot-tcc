#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
import time
from geometry_msgs.msg import Twist

GPIO.setmode(GPIO.BCM)

wheel_radius = 20

right_enable = 9
right_1 = 17
right_2 = 22

left_enable =10
left_1 = 23
left_2 = 24

#Pinos motor esquerdo
GPIO.setup(left_enable, GPIO.OUT)
GPIO.setup(left_1, GPIO.OUT)
GPIO.setup(left_2, GPIO.OUT)
motor_pwm_left = GPIO.PWM(left_enable, 250)
motor_pwm_left.start(0)

#Pinos motor direito
GPIO.setup(right_enable, GPIO.OUT)
GPIO.setup(right_1, GPIO.OUT)
GPIO.setup(right_2, GPIO.OUT)
motor_pwm_right = GPIO.PWM(right_enable, 250)
motor_pwm_right.start(0)

def forward_left(duty):          
    #motor esquerdo
    GPIO.output(left_1, True) 
    GPIO.output(left_2, False) 
    motor_pwm_left.ChangeDutyCycle(duty)

def forward_right(duty):
    #motor direito
    GPIO.output(right_1, True) 
    GPIO.output(right_2, False) 
    motor_pwm_right.ChangeDutyCycle(duty)

def reverse_left(duty): 
    #motor esquerdo         
    GPIO.output(left_1, False) 
    GPIO.output(left_2, True) 
    motor_pwm_left.ChangeDutyCycle(duty)    

def reverse_right(duty):
    #motor direito         
    GPIO.output(right_1, False) 
    GPIO.output(right_2, True) 
    motor_pwm_right.ChangeDutyCycle(duty)    

def stop_left():
    #motor esquerdo
    GPIO.output(left_1, False) 
    GPIO.output(left_2, False) 
    motor_pwm_left.ChangeDutyCycle(0)
    
def stop_right():
    #motor direito
    GPIO.output(right_1, False) 
    GPIO.output(right_2, False) 
    motor_pwm_right.ChangeDutyCycle(0)
    
        
def vel_callback(data):
    global wheel_radius

    Vel_R = (data.linear.x + (wheel_radius*data.angular.z)/2)*200
    Vel_L = (data.linear.x - (wheel_radius*data.angular.z)/2)*200

    if Vel_R > 100:
        Vel_R = 100
    if Vel_R < -100:
        Vel_R = -100
    if Vel_L > 100:
        Vel_L = 100
    if Vel_L < -100:
        Vel_L = -100

        

	#velocidade linear
    if data.linear.x > 0:
        forward_right(abs(Vel_R))
        forward_left(abs(Vel_L))
    elif data.linear.x < 0:
        reverse_right(abs(Vel_R))
        reverse_left(abs(Vel_L))
    elif data.linear.x == 0:
        if data.angular.z == 0:
            stop_right()
            stop_left()
        if data.angular.z > 0:
            forward_right(abs(Vel_R))
            reverse_left(abs(Vel_L))
        elif data.angular.z < 0:
            reverse_right(abs(Vel_R))
            forward_left(abs(Vel_L))

	#velocidade angular	

if __name__ == '__main__':
	rospy.init_node('L298N_control',  anonymous=True)
	rospy.Subscriber("cmd_vel", Twist, vel_callback)
	rospy.spin()

#	try
#
#
#	finally
#
#    print("Cleaning up")
#    GPIO.cleanup()

