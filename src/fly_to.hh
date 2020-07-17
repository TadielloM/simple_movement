#ifndef FLY_TO_HH
#define FLY_TO_HH

#include <actionlib/server/simple_action_server.h>
#include <simple_movement/FlyToAction.h>  // Note: "Action" is appended
#include <ignition/math/Vector3.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>
#include <tf/transform_listener.h>

class FlyTo
{
    private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  actionlib::SimpleActionServer<simple_movement::FlyToAction> as_;
  gazebo::physics::ModelPtr model;

    public:
  FlyTo();
  void execute(const simple_movement::FlyToGoalConstPtr& goal,
               actionlib::SimpleActionServer<simple_movement::FlyToAction>* as);
  void setModel(gazebo::physics::ModelPtr model_);

};

#endif