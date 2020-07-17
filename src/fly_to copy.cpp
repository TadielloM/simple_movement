#include "fly_to.hh"
// #include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
FlyTo::FlyTo()
    : pub_(nh_.advertise<geometry_msgs::PoseStamped>("fly_to_cmd", 1000)), as_(nh_, "fly_to", 
    boost::bind(&FlyTo::execute, this, _1, &as_), false){
  ROS_INFO("Starting fly to server");
  as_.start();
}

void FlyTo::setModel(gazebo::physics::ModelPtr model_){
  this->model=model_;
}

void FlyTo::execute(const simple_movement::FlyToGoalConstPtr &goal,
                    actionlib::SimpleActionServer<simple_movement::FlyToAction> *as)
{
  // ROS_INFO_STREAM("Got new goal: Fly to (" << goal->pose.pose.position.x << ", " << goal->pose.pose.position.y << ", "
  //                                          << goal->pose.pose.position.z << ") ");

  ros::Rate r(20);
  ignition::math::Pose3<double> actual_goal;
  actual_goal.Pos().X() = goal->pose.pose.position.x;
  actual_goal.Pos().Y() = goal->pose.pose.position.y;
  actual_goal.Pos().Z() = goal->pose.pose.position.z;

  tf2::Quaternion quat;

  quat.setRPY( 1, 1, 1 );

  tf2::convert(goal->pose.pose.orientation, quat);

  double goal_roll, goal_pitch, goal_yaw;
  tf2::Matrix3x3(quat).getRPY(goal_roll, goal_pitch, goal_yaw);
  // std::cout<<"Goal orientation: "<<goal_roll<<" "<<goal_pitch<<" "<<goal_yaw<<" "<<"\n";
  ignition::math::Pose3<double> actual_position = this->model->WorldPose();
  float distance_to_goal = 9001; // Distance is over 9000
  float yaw_diff = M_PI;

  // Check if target is reached...
  do
  {
    actual_position = this->model->WorldPose();
    // ROS_INFO_STREAM("Publishing goal to (" << actual_goal.Pos() << ") ");
    pub_.publish(goal->pose);

    ignition::math::Vector3<double> eulers = actual_position.Rot().Euler();
    // std::cout << eulers << std::endl;

    ignition::math::Vector3d velocity;
    if(actual_position.Pos().Distance(actual_goal.Pos()) > 0.6){
      velocity = (actual_goal.Pos() - actual_position.Pos()).Normalize() * 5;
      this->model->SetLinearVel(velocity);
    }
    else if(actual_position.Pos().Distance(actual_goal.Pos()) > 0.2){
      velocity = (actual_goal.Pos() - actual_position.Pos()).Normalize();
      this->model->SetLinearVel(velocity);
    }

    // ROS_INFO_STREAM("Distance to goal: " << actual_position.Pos().Distance(actual_goal.Pos()));
    yaw_diff = goal_yaw - eulers[2];
    ignition::math::Vector3d angular_vel(0,0,(yaw_diff>=0?1:-1)*2);
    // if(yaw_diff>0)
    //   this->model->SetAngularVel(angular_vel);

  } while (actual_position.Pos().Distance(actual_goal.Pos()) > goal->distance_converged); // or yaw_diff > goal->yaw_converged);
  as->setSucceeded();
}

