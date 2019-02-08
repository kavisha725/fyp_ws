#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/SetBool.h>

//#define KINECT_BASE_ANGLE 0.0//-0.65
#define M_PI           3.14159265358979323846
//#define GYRO_OFFSET 0.0125

class baseController{
  public:
    baseController(){
      ros::NodeHandle nh_;

      ros::NodeHandle nhLocal_("~");
      nhLocal_.getParam("gyro_offset",gyro_offset);
      nhLocal_.getParam("tf_data_teleop", tf_data_teleop);
      nhLocal_.getParam("tf_data_human", tf_data_human);

      sub_ = nh_.subscribe("base_controller_data", 1000, &baseController::baseCallback,this);
      pub_ = nh_.advertise<nav_msgs::Odometry>("wheel_odom", 50);
      imuPub_ = nh_.advertise<sensor_msgs::Imu>("imu_data", 50);
      

      service_ = nh_.advertiseService("robot_mode_change", &baseController::serviceCallback,this);
      pubMode_ = nh_.advertise<std_msgs::Bool>("robot_mode", 50);

      // ROS_ERROR("%f",gyro_offset);

    } 

    void baseCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
      double x = msg->data[0];
      double y = msg->data[1];
      double th = msg->data[2];
      double vx = msg->data[3];
      double vth = msg->data[4];
	    double pitch = msg->data[5];
      double yaw_rate = msg->data[6];
      double accelerometer_rate = msg->data[7];
      teleop_mode = msg->data[8];

      // ROS_ERROR("%d",teleop_mode);

      ros::Time current_time = ros::Time::now();

	  //---------------imu_publish-----------------------------------------
      yaw_rate = yaw_rate/180.0*M_PI - gyro_offset;
      sensor_msgs::Imu imu_msg;
      imu_msg.header.stamp = current_time;
      imu_msg.header.frame_id = "imu_link";

      imu_msg.angular_velocity.x = 0;
      imu_msg.angular_velocity.y = 0;
      imu_msg.angular_velocity.z = yaw_rate;
      imu_msg.angular_velocity_covariance = { 0, 0, 0, 0, 0, 0, 0, 0, 0.001};

      imu_msg.linear_acceleration.x = accelerometer_rate;
      imu_msg.linear_acceleration.y = 0; 
      imu_msg.linear_acceleration.z = 9.8;
      imu_msg.linear_acceleration_covariance = { 0.01, 0, 0, 0, 0, 0, 0, 0, 0 };

      imu_msg.orientation_covariance = {-1,   0,   0, 0,  -1,   0, 0,   0,  -1 };

      imuPub_.publish(imu_msg);
     
	  //----------------------------------------------------------------------
	  
	  //-------------base_link_to_pitch_link---------------------------------
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "base_link";
      odom_trans.child_frame_id = "pitch_link";

      odom_trans.transform.translation.x = 0.0;
      odom_trans.transform.translation.y = 0.0;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation =  tf::createQuaternionMsgFromRollPitchYaw(0,(pitch)*M_PI/180.0,0);

      //send the transform
      tf_broadcaster_.sendTransform(odom_trans);

      //-------------------------------------------------------------------

      //---------static_tfs------------------------------------------------
      if(teleop_mode == 1){

        geometry_msgs::TransformStamped static_trans;
        static_trans.header.stamp = current_time;
        static_trans.header.frame_id = "base_link";
        static_trans.child_frame_id = "lidar_link";

        static_trans.transform.translation.x = tf_data_teleop[0];
        static_trans.transform.translation.y = 0.0;
        static_trans.transform.translation.z = tf_data_teleop[1];
        static_trans.transform.rotation =  tf::createQuaternionMsgFromRollPitchYaw(0,0,0);

        tf_broadcaster_.sendTransform(static_trans);

        static_trans.header.stamp = current_time;
        static_trans.header.frame_id = "pitch_link";
        static_trans.child_frame_id = "camera_link";

        static_trans.transform.translation.x = tf_data_teleop[2];
        static_trans.transform.translation.y = 0.0;
        static_trans.transform.translation.z = tf_data_teleop[3];
        static_trans.transform.rotation =  tf::createQuaternionMsgFromRollPitchYaw(0,0.17,0);

        tf_broadcaster_.sendTransform(static_trans);

      } else if (teleop_mode == 0){

        geometry_msgs::TransformStamped static_trans;
        static_trans.header.stamp = current_time;
        static_trans.header.frame_id = "base_link";
        static_trans.child_frame_id = "lidar_link";

        static_trans.transform.translation.x = tf_data_human[0];
        static_trans.transform.translation.y = 0.0;
        static_trans.transform.translation.z = tf_data_human[1];
        static_trans.transform.rotation =  tf::createQuaternionMsgFromRollPitchYaw(0,0,0);

        tf_broadcaster_.sendTransform(static_trans);

        static_trans.header.stamp = current_time;
        static_trans.header.frame_id = "pitch_link";
        static_trans.child_frame_id = "camera_link";

        static_trans.transform.translation.x = tf_data_human[2];
        static_trans.transform.translation.y = 0.0;
        static_trans.transform.translation.z = tf_data_human[3];
        static_trans.transform.rotation =  tf::createQuaternionMsgFromRollPitchYaw(0,0.17,0);

        tf_broadcaster_.sendTransform(static_trans);

      }


      //------------------------------------------------------------------


	  //-----------------------------------------------------------------------

      nav_msgs::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";

      //set the position
      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = y;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);
	    odom.pose.covariance[0] = 0.25;
	    odom.pose.covariance[7] = 0.25;
      odom.pose.covariance[14] = 0.25;
	    odom.pose.covariance[21] = 0.1;
	    odom.pose.covariance[28] = 0.1;
      odom.pose.covariance[35] = 0.1;

      //set the velocity
      odom.child_frame_id = "base_link";
      odom.twist.twist.linear.x = vx;
      odom.twist.twist.linear.y = 0;
      odom.twist.twist.angular.z = vth;
  	  odom.twist.covariance[0] = 0.01;
  	  odom.twist.covariance[7] = 0.01;
  	  odom.twist.covariance[14] = 0.01;
  	  odom.twist.covariance[21] = 0.01;
  	  odom.twist.covariance[28] = 0.01;
  	  odom.twist.covariance[35] = 0.02;


      //publish the message
      pub_.publish(odom);

    }

    bool serviceCallback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
      {
        std_msgs::Bool msg;

        msg.data = request.data;

        pubMode_.publish(msg);

        response.success = true;

        if (msg.data == true){
          response.message = "mode changed to teleop";
        }else{
           response.message = "mode changed to human control";
        }

        return true;
      }

  protected:
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Publisher imuPub_;
    tf::TransformBroadcaster tf_broadcaster_;

    ros::ServiceServer service_;
    ros::Publisher pubMode_;

    double gyro_offset;
    bool teleop_mode;
    std::vector<double> tf_data_teleop; //teleop - (x1,z1) -- lidar,(x2,z2) -- kinect
    std::vector<double> tf_data_human; // human - (x3,z3),(x4,z4)

};



int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  baseController base_controller;

  ros::spin();
  
}
