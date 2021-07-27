#!/usr/bin/env python3

#Import necessary libraries
import rospy
from std_msgs.msg import String,Float64
from sensor_msgs.msg import Imu
from flask import Flask, render_template, Response, jsonify, request, abort
import cv2
#Initialize the Flask app
app = Flask(__name__)

#camera = cv2.VideoCapture(2)
'''
for ip camera use - rtsp://username:password@ip_address:554/user=username_password='password'_channel=channel_number_stream=0.sdp' 
for local webcam use cv2.VideoCapture(0)
'''

imu_data = Imu
smoke_value = 0
smoke_safety = 0
dist = 0
temperature = 0

def callback_imu(data):
    global imu_data
    imu_data = data

def callback_value(data):
    global smoke_value
    smoke_value = data
    
def callback_safety(data):
    global smoke_safety
    smoke_safety = data
    
def callback_dist(data):
    global dist
    dist = data
    
def callback_temp(data):
    global temperature
    temperature = data

def gen_frames():  
    while True:
        success, frame = camera.read()  # read the camera frame
        if not success:
            break
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')  # concat frame one by one and show result

@app.route('/')
def index():
    return render_template('index.html')

# @app.route('/video_feed')
# def video_feed():
#     return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# Route/Function to read info from topic and them publish to route
@app.route('/info', methods = ['GET'])
def info():
    global temperature, dist, smoke_safety, smoke_value, imu_data
    safety = "True"

    if smoke_safety.data == 0:
        safety = "False"
    else:
        safety = "True"
    
    if request.method =='GET':
        payload = ( "X Position:"+str(imu_data.linear_acceleration.x)+"<br>"+
                    "\nY Position:"+str(imu_data.linear_acceleration.y)+"<br>"+
                    "\nZ Position:"+str(imu_data.linear_acceleration.z)+"<br>"+
                    "\nAngular Velocity X:"+str(imu_data.angular_velocity.x)+"<br>"+
                    "\nAngular Velocity Y:"+str(imu_data.angular_velocity.y)+"<br>"+
                    "\nAngular Velocity Z:"+str(imu_data.angular_velocity.z)+"<br>"+
                    "\nDistance:"+str(dist.data)+"<br>"+
                    ",\nSmoke Safety:"+str(safety)+"<br>"+
                    ",\nTemperature:"+str(temperature.data))   

        return(payload)
    return("Nothing publishing...")


if __name__ == "__main__":
    rospy.loginfo('Starting listener')
    rospy.Subscriber("imu_real", Imu, callback_imu)
    rospy.Subscriber("smoke/value", Float64, callback_value)
    rospy.Subscriber("smoke/safety", Float64, callback_safety)
    rospy.Subscriber("distance", Float64, callback_dist)
    rospy.Subscriber("temperature", Float64, callback_temp)
    rospy.init_node('flask_subs', anonymous=True)
    app.run(debug=True, host='192.168.0.22')