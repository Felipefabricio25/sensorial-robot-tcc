
#!/usr/bin/python
# -*- coding: iso-8859-1 -*-
 
import serial
import rospy
import time
from std_msgs.msg import String

class Seriall(object):
    
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyUSB0', 9600)
        self.tempublisher = rospy.Publisher("/temp", String, queue_size=10)
        self.umidpublisher = rospy.Publisher("/umid", String, queue_size=10)
        self.temp = String()
        self.umid = String()
        time.sleep(2) 

    def main(self):
        VALUE_SERIAL=self.ser.readline()
        print '\nRetorno da serial: %s' % (VALUE_SERIAL)
        self.temp = VALUE_SERIAL
        self.tempublisher.publish(self.temp)
        VALUE_SERIAL=self.ser.readline()
        print '\nRetorno da serial: %s' % (VALUE_SERIAL)
        self.umid = VALUE_SERIAL
        self.umidpublisher.publish(self.umid)


if __name__ == '__main__':
    rospy.init_node("serialduino")
    rate = rospy.Rate(10)
    serial_object = Seriall()
    print("Initializing...")
    while not rospy.is_shutdown():
        serial_object.main()
        rate.sleep()
