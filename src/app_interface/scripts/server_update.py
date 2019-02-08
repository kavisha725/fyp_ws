#!/usr/bin/env python

# Python node to update database about robot state.

import rospy
import time
import requests
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8
import tf
from geometry_msgs.msg import PoseStamped


class ServerUpdate(object):
    def __init__(self):
        # Params
        self.nav_state_ = 0
        self.origin_x_   = -6.8999999999999915
        self.origin_y_   = -5.8999999999999915
        self.resolution_ = 0.05
        self.map_height_ = 194
        self.map_width_  = 231

        self.origin_x_ = rospy.get_param("/send_goal/origin_x",-6.8999999999999915)
        self.origin_y_ = rospy.get_param("/send_goal/origin_y",-5.8999999999999915)
        self.resolution_ = rospy.get_param("/send_goal/resolution",0.05)
        self.map_height_ = rospy.get_param("/send_goal/map_height",194)
        self.map_width_ = rospy.get_param("/send_goal/map_width",231)
	print self.map_height_, self.map_width_, self.origin_x_,self.origin_y_

        # Subscribers
        rospy.Subscriber("odometry/filtered", Odometry, self.odomCallback, queue_size=1)
        rospy.Subscriber("nav_state", Int8, self.navCallback, queue_size=1)
	self.listener = tf.TransformListener()

    def odomCallback(self, odom_msg):
	try:
		self.listener.waitForTransform('/map','/odom',rospy.Time(),rospy.Duration(4.0))
		map_pose = self.listener.transformPose('/map',PoseStamped(odom_msg.header, odom_msg.pose.pose))
	except:
		return
	#print ("trasformed")
	#print trans
        #x_odom = odom_msg.pose.pose.position.x - trans[0]
        #y_odom = odom_msg.pose.pose.position.y- trans[1]
	x_odom = map_pose.pose.position.x
	y_odom = map_pose.pose.position.y
        linear_vel = odom_msg.twist.twist.linear.x
        battery_percent = 100.0
        x_app = (x_odom - self.origin_x_)*(100.0/self.map_width_)*(1.0/self.resolution_)
        y_app = 100.0 - (y_odom - self.origin_y_)*(100.0/self.map_height_)*(1.0/self.resolution_)
        print "X= %f Y=%f SPD=%f nav_state=%r \n"%(x_app,y_app,linear_vel,self.nav_state_)

        url = 'http://10.42.0.1/update.php?X_LOC=%f&Y_LOC=%f&SPD=%f&BAT=%f&NAV_STATE=%d'%(x_app,y_app,linear_vel,battery_percent,self.nav_state_)
        r = requests.put(url)
        #print(r.text)
        time.sleep(0.5)

    def navCallback(self, nav_msg):
        print "nav_callback"
        # print nav_msg
        print nav_msg.data
        self.nav_state_ = nav_msg.data
        # print self.nav_state_

    def start(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('server_update_node', anonymous=True)

    print "server_update main"

    my_node = ServerUpdate()
    my_node.start()
