#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

class scanRemap:
    def __init__(self):
        rospy.init_node('scan_remapper')
        self.pub = rospy.Publisher('scan_new', LaserScan, queue_size=10)
        rospy.Subscriber("scan_filtered", LaserScan, self.callback)
        rospy.spin()

    def callback(self,msg):
        msg.header.stamp = rospy.Time.now()
        self.pub.publish(msg)
        

if __name__ == '__main__':
    scanRemap()
