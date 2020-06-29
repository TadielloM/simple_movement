#include <actionlib/server/simple_action_server.h>
#include <stl_aeplanner_msgs/FlyToAction.h>  // Note: "Action" is appended

#include <ros/ros.h>
#include <tf/transform_listener.h>

class FlyTo
{
private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  actionlib::SimpleActionServer<stl_aeplanner_msgs::FlyToAction> as_;

  tf::TransformListener listener;


public:
  FlyTo()
    : pub_(nh_.advertise<geometry_msgs::PoseStamped>("fly_to_cmd", 1000))
    , as_(nh_, "fly_to", boost::bind(&FlyTo::execute, this, _1, &as_), false)
  {
    ROS_INFO("Starting fly to server");
    as_.start();
  }

  void execute(const stl_aeplanner_msgs::FlyToGoalConstPtr& goal,
               actionlib::SimpleActionServer<stl_aeplanner_msgs::FlyToAction>* as)
  {
    ROS_INFO_STREAM("Got new goal: Fly to (" << goal->pose.pose.position.x << ", " << goal->pose.pose.position.y << ", "
                                             << goal->pose.pose.position.z << ") ");

    ros::Rate r(20);
    ignition::math::Pose3<double> actual_goal = goal->pose.pose.position;
    ignition::math::Pose3<double> actual_position = this->model->WorldPose();
    float distance_to_goal = 9001;  // Distance is over 9000
    float yaw_diff = M_PI;

    // Check if target is reached...
    do
    {
      actual_position = this->model->WorldPose();

      ROS_INFO_STREAM("Publishing goal to (" << p.x << ", " << p.y << ", " << p.z << ") ");
      pub_.publish(goal->pose);
      ignition::math::Vector3d velocity;
      velocity = (actual_goal.Pos() - actual_position.Pos()).Normalize() * 5;
      
      distance_to_goal = sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
      ROS_INFO_STREAM("Distance to goal: " << distance_to_goal);
      yaw_diff = fabs(atan2(sin(goal_yaw - current_yaw), cos(goal_yaw - current_yaw)));

    } while (distance_to_goal > goal->distance_converged or yaw_diff > goal->yaw_converged); actual_position.Pos().Distance(actual_goal.Pos()) > 0

    as->setSucceeded();
  }
};