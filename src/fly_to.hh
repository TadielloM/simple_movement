#ifndef FLY_TO_HH
#define FLY_TO_HH

#include <actionlib/server/simple_action_server.h>
#include <simple_movement/FlyToAction.h>  // Note: "Action" is appended
#include <ignition/math/Vector3.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>

#include "rrt.hh"
#include <ros/ros.h>
#include <tf/transform_listener.h>

class FlyTo
{
  private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber octomap_sub_;
  actionlib::SimpleActionServer<simple_movement::FlyToAction> as_;
  gazebo::physics::ModelPtr model;
  std::shared_ptr<octomap::OcTree> ot_;


    public:
  FlyTo();
  void execute(const simple_movement::FlyToGoalConstPtr& goal,
               actionlib::SimpleActionServer<simple_movement::FlyToAction>* as);
  void setModel(gazebo::physics::ModelPtr model_);
  void octomapCallback(const octomap_msgs::Octomap& msg);
  point_rtree getRtree(std::shared_ptr<octomap::OcTree> ot, octomap::point3d min, octomap::point3d max);

};

#endif