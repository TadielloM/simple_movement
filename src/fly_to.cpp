#include "fly_to.hh"
// #include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <exception>
#include <visualization_msgs/Marker.h>

visualization_msgs::Marker visualize_goal(ignition::math::Pose3<double> actual_goal){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "goal";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = actual_goal.Pos().X();
  marker.pose.position.y = actual_goal.Pos().Y();
  marker.pose.position.z = actual_goal.Pos().Z();
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.lifetime =ros::Duration(10);
  //only if using a MESH_RESOURCE marker type:
  // marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
  return marker;
}

FlyTo::FlyTo(std::shared_ptr<ros::NodeHandle> nh) :
    nh_(nh),
    pub_(nh_->advertise<geometry_msgs::PoseStamped>("fly_to_cmd", 1000)), as_(*nh_, "fly_to", 
    boost::bind(&FlyTo::execute, this, _1, &as_), false),
    octomap_sub_(nh_->subscribe("octomap_full", 1, &FlyTo::octomapCallback,this)),
    ot_(NULL),
    vis_pub(nh_->advertise<visualization_msgs::Marker>( "goal_to_reach", 0 ))
{
  // ROS_INFO("Starting fly to server");
  as_.start();
}

void FlyTo::setModel(gazebo::physics::ModelPtr model_)
{
  this->model=model_;
}

void FlyTo::setVelocitiesPointer(ignition::math::Vector3d* linear_vel, ignition::math::Vector3d* angular_vel)
{
  this->linear_vel =linear_vel;
  this->angular_vel =angular_vel;
}

void FlyTo::octomapCallback(const octomap_msgs::Octomap& msg)
{
  octomap::AbstractOcTree* aot = octomap_msgs::msgToMap(msg);
  octomap::OcTree* ot = (octomap::OcTree*)aot;
  ot_ = std::make_shared<octomap::OcTree>(*ot);

  delete ot;
}

point_rtree FlyTo::getRtree(std::shared_ptr<octomap::OcTree> ot, octomap::point3d min, octomap::point3d max)
{
  point_rtree octomap_rtree;
  // std::cout <<*ot<<std::endl;
  for (octomap::OcTree::leaf_bbx_iterator it = ot->begin_leafs_bbx(min, max), it_end = ot->end_leafs_bbx();
       it != it_end; ++it)
  {
    if (it->getLogOdds() > 0)
    {
      octomap_rtree.insert(point3d(it.getX(), it.getY(), it.getZ()));
    }
  }
  return octomap_rtree;
}


void FlyTo::execute(const simple_movement::FlyToGoalConstPtr &goal,
                    actionlib::SimpleActionServer<simple_movement::FlyToAction> *as)
{
  // ROS_INFO_STREAM("Got new goal: Fly to (" << goal->pose.pose.position.x << ", " << goal->pose.pose.position.y << ", " << goal->pose.pose.position.z << ") ");

  ros::Rate r(20);
  ignition::math::Pose3<double> actual_goal;
  actual_goal.Pos().X() = goal->pose.pose.position.x;
  actual_goal.Pos().Y() = goal->pose.pose.position.y;
  actual_goal.Pos().Z() = goal->pose.pose.position.z;
  
  vis_pub.publish(visualize_goal(actual_goal));

  tf2::Quaternion quat;

  quat.setRPY( 1, 1, 1 );

  tf2::convert(goal->pose.pose.orientation, quat);

  double goal_roll, goal_pitch, goal_yaw;
  tf2::Matrix3x3(quat).getRPY(goal_roll, goal_pitch, goal_yaw);
  // std::cout<<"Goal orientation: "<<goal_roll<<" "<<goal_pitch<<" "<<goal_yaw<<" "<<"\n";
  ignition::math::Pose3<double> actual_position = this->model->WorldPose();
  float distance_to_goal = 9001; // Distance is over 9000
  float roll_diff = M_PI;
  float pitch_diff = M_PI;
  float yaw_diff = M_PI;

  Eigen::Vector4d current_state;
  current_state[0] = actual_position.Pos().X();
  current_state[1] = actual_position.Pos().Y();
  current_state[2] = actual_position.Pos().Z();
  current_state[3] = 0;
  Eigen::Vector4d state_to_reach;
  state_to_reach[0] = actual_goal.Pos().X();
  state_to_reach[1] = actual_goal.Pos().Y();
  state_to_reach[2] = actual_goal.Pos().Z();
  state_to_reach[3] = 0;
  
  // ROS_INFO_STREAM("The current state is: "<<current_state);
  // ROS_INFO_STREAM("The state to reach is "<<state_to_reach);


  std::shared_ptr<RRT> root_(std::make_shared<RRT>(current_state));
  value_rtree rrt_rtree;
  rrt_rtree.insert(std::make_pair(point3d(current_state[0], current_state[1], current_state[2]), root_));;


  octomap::point3d min(current_state[0] - max_sampling_radius - 0.5,
                       current_state[1] - max_sampling_radius - 0.5,
                       current_state[2] - max_sampling_radius - 0.5);

  octomap::point3d max(current_state[0] + max_sampling_radius + 0.5,
                       current_state[1] + max_sampling_radius + 0.5,
                       current_state[2] + max_sampling_radius + 0.5);

  std::shared_ptr<point_rtree> octomap_rtree = std::make_shared<point_rtree>(getRtree(ot_, min, max));
  nav_msgs::Path path_to_goal = root_->findTrajectory(ot_,octomap_rtree,&rrt_rtree,current_state,state_to_reach);
  // std::cout << "drone " <<nh_.get()<<std::endl;

  // Check if target is reached...
  for(int i=0; i<path_to_goal.poses.size();i++){
    actual_goal.Pos().X() = path_to_goal.poses[i].pose.position.x;
    actual_goal.Pos().Y() = path_to_goal.poses[i].pose.position.y;
    actual_goal.Pos().Z() = path_to_goal.poses[i].pose.position.z;
    do
    {
      actual_position = this->model->WorldPose();
      // ROS_INFO_STREAM("Publishing goal to (" << actual_goal.Pos() << ") ");
      pub_.publish(goal->pose);

      ignition::math::Vector3<double> eulers = actual_position.Rot().Euler();
      // std::cout << eulers << std::endl;

      //Setting linear velocity (smaller id closer to goal)
      ignition::math::Vector3d velocity;
      if(actual_position.Pos().Distance(actual_goal.Pos()) > 0.6){
        velocity = (actual_goal.Pos() - actual_position.Pos()).Normalize() * 5;
        *linear_vel = velocity;
      }
      else if(actual_position.Pos().Distance(actual_goal.Pos()) > 0.2){
        velocity = (actual_goal.Pos() - actual_position.Pos()).Normalize();
        *linear_vel = velocity;
      }
      // std:cout<<"The velocity is: "<< *linear_vel<<std::endl;

      // ROS_INFO_STREAM("Distance to goal: " << actual_position.Pos().Distance(actual_goal.Pos()));

      //setting angular velocity
      roll_diff = goal_roll - eulers[0];
      pitch_diff = goal_pitch - eulers[1];
      yaw_diff = goal_yaw - eulers[2];
      ignition::math::Vector3d angular_velocity((roll_diff>=goal->yaw_converged?1:0)*2,(pitch_diff>=goal->yaw_converged?1:0)*2,(yaw_diff>=goal->yaw_converged?1:0)*2);
      *angular_vel = angular_velocity;

    } while (actual_position.Pos().Distance(actual_goal.Pos()) > goal->distance_converged); //TODO Check also when angle converge
  }

  *linear_vel =ignition::math::Vector3d(0,0,0);
  *angular_vel = ignition::math::Vector3d(0,0,0);
  this->result_.steps = int(path_to_goal.poses.size());
  as->setSucceeded(this->result_);
}

