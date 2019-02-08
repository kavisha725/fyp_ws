#!/usr/bin/env python

# Python node to read database and publish data as ROS msg.

import rospy
from app_interface.msg import app_point
from geometry_msgs.msg import Twist,Vector3
from math import sin,cos,pi
import std_srvs.srv

import requests
import json

def server_read():
    point_pub = rospy.Publisher('app_point', app_point, queue_size=10)
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('server_read_node', anonymous=True)
    rate = rospy.Rate(2) # 2 HZ
    #float vel_x, vel_theta
    previous_mode = 0
    while not rospy.is_shutdown():
        # Read database
        # Change IP as needed
        r = requests.put('http://10.42.0.1/select_from_app.php')
        json_data = r.text
        obj = json.loads(json_data)[0]
        mode = int(obj['MODE'])

        # Create and Publish velocity message
        if (mode == 1):
            vel_x = float(obj['STRENGTH']) * sin(float(obj['AZIMUTH'])*pi/180.0) * (0.35/100.0)
            vel_theta = -1 * float(obj['STRENGTH']) * cos(float(obj['AZIMUTH'])*pi/180.0) * (1.0/100.0)
            vel_msg = Twist(Vector3(vel_x,0,0),Vector3(0,0,vel_theta))
            vel_pub.publish(vel_msg)

        else:
            if (mode != previous_mode):               
                print("Mode change")
                robot_mode_change = rospy.ServiceProxy('robot_mode_change', std_srvs.srv.SetBool())
                try:
                    if (mode == 0):
                        resp1 = robot_mode_change(True)
                    elif (mode == 2):
                        resp1 = robot_mode_change(False)
                    print resp1
                    previous_mode = mode
                except rospy.ServiceException as exc:
                    print("Service did not process request: " + str(exc))

            if (float(obj['TO_X_LOC']) >= 0.0):
                # Create and Publish point message
                point_msg = app_point()
                point_msg.read_time = rospy.get_time()
                point_msg.id = int(obj['ID'])
                point_msg.mode = mode
                point_msg.x = float(obj['TO_X_LOC'])
                point_msg.y = float(obj['TO_Y_LOC'])
                point_msg.strength = float(obj['STRENGTH'])
                point_msg.azimuth = float(obj['AZIMUTH'])       
                #rospy.loginfo(msg)
                point_pub.publish(point_msg)

                # Clear database after sending goal
                # Change IP as needed
                update = 'http://10.42.0.1/update_from_app.php?TO_X_LOC=-1&TO_Y_LOC=-1&MODE=%d&STRENGTH=0&AZIMUTH=0'%(mode)
                # update = 'http://10.10.15.186/update_from_app.php?TO_X_LOC=50&TO_Y_LOC=51&MODE=2&STRENGTH=0&AZIMUTH=0'
                u = requests.put(update)
                rospy.loginfo(u.text) 

        	rate.sleep()

if __name__ == '__main__':
    try:
        server_read()
    except rospy.ROSInterruptException:
        pass
