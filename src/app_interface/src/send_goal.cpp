#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <app_interface/app_point.h>
#include <std_msgs/Int8.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class navGoals
{
public:
  navGoals() : ac("move_base", true)

  {
    ros::NodeHandle n;

    std::string node_name = ros::this_node::getName();
    std::string ns = ros::this_node::getNamespace();
    std::string full_name = (ns.compare("/") == 0) ? node_name : (ns + node_name);

    n.param<double>(full_name + "/origin_x", origin_x_, -6.8999999999999915);
    n.param<double>(full_name + "/origin_y", origin_y_, -5.8999999999999915);
    n.param<double>(full_name + "/resolution", resolution_, 0.05);
    n.param<int>(full_name + "/map_height", map_height_, 194);
    n.param<int>(full_name + "/map_width", map_width_, 231);
    n.param<float>(full_name + "/tolerance", tolerance_, 0.7);
    ROS_INFO("origin_x_:%f, origin_y_:%f, map_height_:%d, map_width_:%d, resolution_:%f, tolerance_:%f", origin_x_,
             origin_y_, map_height_, map_width_, resolution_, tolerance_);

    app_sub = n.subscribe("/app_point", 1000, &navGoals::getGoalCallback, this);
    pose_sub = n.subscribe("/amcl_pose", 1000, &navGoals::poseCallback, this);
    nav_state_pub = n.advertise<std_msgs::Int8>("nav_state", 1000);

    prev_x_ = 999.0;
    prev_x_ = 999.0;
    goal_active_.data = 0;
  }

  void getGoalCallback(const app_interface::app_point::ConstPtr& app_goal)
  {
    if ((app_goal->mode) != 1)
    {
      if ((prev_x_ == app_goal->x) && (prev_y_ == app_goal->y))
      {
        // ROS_INFO("Repeated app goal. Skipped.");
      }
      else
      {
        ROS_INFO("Goal callback");
        move_x_ = (app_goal->x) * (map_width_ / 100.0) * resolution_ + origin_x_;
        move_y_ = (100.0 - app_goal->y) * (map_height_ / 100.0) * resolution_ + origin_y_;
        // Scale to prevent going out of bounds
        // move_x_ *= 0.9;
        // move_y_ *= 0.9;
        ROS_INFO("app_x:%f, app_y:%f, move_x_:%f, move_y_:%f", app_goal->x, app_goal->y, move_x_, move_y_);
        move_base_msgs::MoveBaseGoal goal;

        // wait for the action server to  ome up
        while (!ac.waitForServer(ros::Duration(5.0)))
        {
          ROS_INFO("Waiting for the move_base action server to come up");
        }
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = move_x_;
        goal.target_pose.pose.position.y = move_y_;
        goal.target_pose.pose.orientation.w = 1.0;

        ROS_INFO("Sending goal");
        ac.sendGoal(goal, boost::bind(&navGoals::doneCallback, this, _1), MoveBaseClient::SimpleActiveCallback());
        goal_active_.data = 1;
        nav_state_pub.publish(goal_active_);
        prev_x_ = app_goal->x;
        prev_y_ = app_goal->y;
      }
    }
  }

  void doneCallback(const actionlib::SimpleClientGoalState& state)
  {
    ROS_INFO("DONECB: Finished in state [%s]", state.toString().c_str());
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      // do something as goal was reached
      goal_active_.data = 3;
      nav_state_pub.publish(goal_active_);
      ROS_INFO("Goal done");
    }
    if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
    {
      // do something as goal was canceled
      goal_active_.data = 2;
      nav_state_pub.publish(goal_active_);
      ROS_INFO("Goal done");
    }
  }

  void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& current_pose)
  {
    current_pose_ = current_pose->pose.pose;
    ROS_INFO("Pose callback");
    double current_dist_to_target =
        std::sqrt((current_pose_.position.x - move_x_) * (current_pose_.position.x - move_x_) +
                  (current_pose_.position.y - move_y_) * (current_pose_.position.y - move_y_));
    ROS_INFO("dist to goal: %f", current_dist_to_target);
    if ((current_dist_to_target <= tolerance_) && (goal_active_.data == true))
    {
      ac.cancelAllGoals();
      goal_active_.data = 0;
      nav_state_pub.publish(goal_active_);
      ROS_INFO("goal cancelled");
    }
  }

protected:
  double origin_x_;
  double origin_y_;
  double resolution_;
  int map_height_;
  int map_width_;
  float tolerance_;

  double move_x_;
  double move_y_;
  std_msgs::Int8 goal_active_;

  double prev_x_;
  double prev_y_;

  geometry_msgs::Pose current_pose_;

  ros::Subscriber app_sub;
  ros::Subscriber pose_sub;
  ros::Publisher nav_state_pub;
  MoveBaseClient ac;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "app_navigation_goals");
  navGoals navigation;

  ROS_INFO("main");
  ros::spin();

  return 0;
}