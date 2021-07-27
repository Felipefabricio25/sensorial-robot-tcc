#!/usr/bin/python3
# -*- coding: iso-8859-1 -*-
 
import rospy
import serial
import tf
import time
import math
import numpy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

class Seriall(object):
    
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyUSB0', 9600)
        self.imu_pub = rospy.Publisher("/imu_real", Imu, queue_size=10)
        self.yaw_pub = rospy.Publisher("/yaw", Float64, queue_size=10)
        self.svalue_pub = rospy.Publisher("/smoke/value", Float64, queue_size=10)
        self.ssafety_pub = rospy.Publisher("/smoke/safety", Float64, queue_size=10)
        self.dist_pub = rospy.Publisher("/distance", Float64, queue_size=10)
        self.temp_pub = rospy.Publisher("/temperature", Float64, queue_size=10)
        self.imu_data = Imu()
        self.first_data = Imu()
        self.first = 0
        self.last_time = rospy.get_time()
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.smoke_level = 0
        self.smoke_safety = 0
        self.distance = 0
        self.temperature = 0
        self.m_movelx = [0,0,0,0,0]
        self.m_movely = [0,0,0,0,0]
        self.m_movelz = [0,0,0,0,0]
        self.invert = 1
        time.sleep(2)

    def Average(self, lst):
        return sum(lst) / len(lst)

    def main(self):
        time_now = rospy.get_time()
        delta_t = time_now - self.last_time
        self.last_time = time_now
        print("Time: "+ str(delta_t))
        VALUE_SERIAL=self.ser.readline()
        i = 0
        j = 0
        point = "a"
        bytelen = len(VALUE_SERIAL)
        print("Bytes = "+str(bytelen))
        Stringson = []
        Accel_Values = []
        Gyro_Values = []
        Magneto_Values = []

        print(VALUE_SERIAL)
        
        for letters in VALUE_SERIAL:
            if j > bytelen-3:
                pass
            else:
                Stringson.append(chr(letters))
            j+=1

        print(Stringson)
        

        text_base = "".join(Stringson)
        print(text_base)
        values = text_base.split(" ")
        print(values)

        if self.first < 5:
            pass
            self.first +=1
        elif self.first == 5:
            try:
                self.first_data.linear_acceleration.x = float(values[1])
                self.first_data.linear_acceleration.y = float(values[2])
                self.first_data.linear_acceleration.z = float(values[3])
                self.first_data.angular_velocity.x = float(values[4])
                self.first_data.angular_velocity.y = float(values[5])
                self.first_data.angular_velocity.z = float(values[6])                
            except:
                pass
            self.first = 11
        elif self.first == 11:
            self.m_movelx.append(self.m_movelx.pop(0))
            self.m_movely.append(self.m_movely.pop(0))
            self.m_movelz.append(self.m_movelz.pop(0))

            self.imu_data.linear_acceleration.x = float(values[1])
            self.imu_data.linear_acceleration.y = float(values[2])
            self.imu_data.linear_acceleration.z = float(values[3])
            self.m_movelx[0] = float(values[4]) - self.first_data.angular_velocity.x
            self.m_movely[0] = float(values[5]) - self.first_data.angular_velocity.y
            self.m_movelz[0] = float(values[6]) - self.first_data.angular_velocity.z

            self.imu_data.angular_velocity.x = round(self.Average(self.m_movelx),2)
            self.imu_data.angular_velocity.y = round(self.Average(self.m_movely),2)
            self.imu_data.angular_velocity.z = round(self.Average(self.m_movelz),2)
            
            self.smoke_level = float(values[7])
            self.smoke_safety = float(values[8])
            if float(values[9]) > 100:
                pass
            else:
                self.distance = float(values[9])
            self.temperature = float(values[0])
            

        print("First: " + str(self.first_data))

        self.roll = self.imu_data.angular_velocity.x * delta_t + self.roll
        if (self.roll > math.pi):
            self.roll = self.roll - 2*(math.pi)
        elif(self.roll < (-1)*math.pi):
            self.roll = self.roll + 2*math.pi

        self.pitch = self.imu_data.angular_velocity.y * delta_t + self.pitch
        if (self.pitch > (0.5)*math.pi):
            self.pitch = self.pitch - (math.pi)
        elif(self.pitch < (-0.5)*math.pi):
            self.pitch = self.pitch + math.pi

        self.yaw = self.imu_data.angular_velocity.z * delta_t + self.yaw 
        if (self.yaw > math.pi):
            self.yaw = self.yaw - 2*(math.pi)
        elif(self.yaw < (-1)*math.pi):
            self.yaw = self.yaw + 2*math.pi

        self.yaw_pub.publish(self.yaw)

        degrees = self.yaw*180/math.pi
        print("Yaw: " + str(degrees))

        quaternion = tf.transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw)


        self.imu_data.orientation.x = quaternion[0]
        self.imu_data.orientation.y = quaternion[1]
        self.imu_data.orientation.z = quaternion[2]
        self.imu_data.orientation.w = quaternion[3]

        print("Accel: " + str(Accel_Values))
        print("Gyro: " + str(Gyro_Values))
        print("----------------------------")
        print(self.imu_data)
        print("----------------------------")

        self.imu_pub.publish(self.imu_data)
        self.svalue_pub.publish(self.smoke_level)
        self.ssafety_pub.publish(self.smoke_safety)
        self.dist_pub.publish(self.distance)
        self.temp_pub.publish(self.temperature)


if __name__ == '__main__':
    rospy.init_node("imu_publish")
    rate = rospy.Rate(10)
    serial_object = Seriall()
    print("Initializing...")
    while not rospy.is_shutdown():
        serial_object.main()
        rate.sleep()
