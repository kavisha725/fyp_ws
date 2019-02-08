#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point,Pose,Quaternion,Twist,Vector3
import numpy as np
from math import sin,cos,pi
import tf


class baseController:
    def __init__(self):
        self.Xk = np.zeros((5,1),dtype=np.float64)
        self.Pk = np.eye(5,dtype=np.float64)
        self.Q = np.eye(5,dtype=np.float64)
        self.Q[0,0] = 0.25
        self.Q[1,1] = 0.25
        self.Q[2,2] = 0.27
        self.Q[3,3] = 0.01
        self.Q[4,4] = 0.1

        self.H1 = np.array([[0,0,0,1,0],[0,0,0,0,1]],dtype=np.float64)
        self.R1 = np.array([[0.01,0],[0,0.1]],dtype=np.float64)

        self.H2 = np.array([[0,0,0,0,1]],dtype=np.float64)
        self.R2 = np.array([[0.1]],dtype=np.float64)

        self.G = np.eye(5,dtype=np.float64)


        rospy.init_node('odometry_publisher')
        self.pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.tf_pub = tf.TransformBroadcaster()
        rospy.Subscriber("base_controller_data", Float32MultiArray, self.callback)
        rospy.spin()

    def callback(self,msg):
        dt = msg.data[8]
        self.G[0,2] = -self.Xk[3,0]*sin(self.Xk[2,0])*dt
        self.G[0,3] = dt*cos(self.Xk[2,0])
        self.G[1,2] = self.Xk[3,0]*cos(self.Xk[2,0])*dt
        self.G[1,3] = dt*sin(self.Xk[2,0])
        self.G[2,4] = dt

        self.Xk += np.array([[self.Xk[3,0]*cos(self.Xk[2,0])*dt],[self.Xk[3,0]*sin(self.Xk[2,0])*dt],[self.Xk[4,0]*dt],[0],[0]])
        self.Pk = self.G.dot(self.Pk).dot(self.G.T) + self.Q

        vx = msg.data[3]
        vth = msg.data[4]

        g_factor = 1.125
        Z1 = np.array([[vx],[vth*g_factor]],dtype=np.float64)

        S = self.H1.dot(self.Pk).dot(self.H1.T) + self.R1

        K = self.Pk.dot(self.H1.T).dot(np.linalg.pinv(S))

        self.Xk += K.dot(Z1 - self.H1.dot(self.Xk))

        self.Pk -= K.dot(self.H1).dot(self.Pk)

        gyro_rate = msg.data[6]*pi/180.0

        Z2 = np.array([[gyro_rate*g_factor]],dtype=np.float64)

        S = self.H2.dot(self.Pk).dot(self.H2.T) + self.R2

        K = self.Pk.dot(self.H2.T)/S

        # rospy.loginfo(str(K.shape))
        # rospy.loginfo(str(S.shape))
        # rospy.loginfo(str(Z2.shape))
        # rospy.loginfo(str(self.H2.shape))
        self.Xk += K.dot(Z2 - self.H2.dot(self.Xk))

        self.Pk -= K.dot(self.H2).dot(self.Pk)

        odom_quat = tf.transformations.quaternion_from_euler(0,0,self.Xk[2,0])
        current_time = rospy.Time.now()

        self.tf_pub.sendTransform((self.Xk[0,0],self.Xk[1,0],0),odom_quat,current_time,"base_link","odom")


        odom_msg = Odometry()

        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"

        
        odom_msg.pose.pose = Pose(Point(self.Xk[0,0],self.Xk[1,0],0),Quaternion(*odom_quat))
        odom_msg.pose.covariance = self.setPoseCovariance()


        odom_msg.child_frame_id = "base_link"
        odom_msg.twist.twist = Twist(Vector3(self.Xk[3,0],0,0),Vector3(0,0,self.Xk[4,0]))
        odom_msg.twist.covariance = self.setTwistCovariance()

        self.pub.publish(odom_msg)

    def setPoseCovariance(self):
        pose_covariance = [0]*36

        pose_covariance[0] = self.Pk[0,0]
        pose_covariance[1] = self.Pk[0,1]
        pose_covariance[5] = self.Pk[0,2]
        pose_covariance[6] = self.Pk[1,0]
        pose_covariance[7] = self.Pk[1,1]
        pose_covariance[11] = self.Pk[1,2]
        pose_covariance[30] = self.Pk[2,0]
        pose_covariance[31] = self.Pk[2,1]
        pose_covariance[35] = self.Pk[2,2]

        return pose_covariance 

    def setTwistCovariance(self):
        twist_covariance = [0]*36

        twist_covariance[0] = self.Pk[3,3]
        twist_covariance[5] = self.Pk[3,4]
        twist_covariance[30] = self.Pk[4,3]
        twist_covariance[35] = self.Pk[4,4]
        
        return twist_covariance
        

if __name__ == '__main__':
    baseController()
