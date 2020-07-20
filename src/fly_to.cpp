#include "fly_to.hh"
// #include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <exception>

FlyTo::FlyTo()
    : pub_(nh_.advertise<geometry_msgs::PoseStamped>("fly_to_cmd", 1000)), as_(nh_, "fly_to", 
    boost::bind(&FlyTo::execute, this, _1, &as_), false),
    octomap_sub_(nh_.subscribe("octomap_full", 1, &FlyTo::octomapCallback,this)),
    ot_(NULL)
{
  ROS_INFO("Starting fly to server");
  as_.start();
}

void FlyTo::setModel(gazebo::physics::ModelPtr model_){
  this->model=model_;
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
      std::cout<<"ARRIVO PRIMA DI COSTRUIRE TUTTO 6.1\n";

  point_rtree octomap_rtree;
      std::cout<<"ARRIVO PRIMA DI COSTRUIRE TUTTO 6.2\n";

  // std::cout <<*ot<<std::endl;

  for (octomap::OcTree::leaf_bbx_iterator it = ot->begin_leafs_bbx(min, max), it_end = ot->end_leafs_bbx();
       it != it_end; ++it)
  {
        

    if (it->getLogOdds() > 0)
    {
      octomap_rtree.insert(point3d(it.getX(), it.getY(), it.getZ()));
      // std::cout<<"ARRIVO PRIMA DI COSTRUIRE TUTTO 6.3\n";
    }
  }
      std::cout<<"ARRIVO PRIMA DI COSTRUIRE TUTTO 6.4\n";

  return octomap_rtree;
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

  std::cout<<"ARRIVO PRIMA DI COSTRUIRE TUTTO1\n";
  std::shared_ptr<RRT> root_(std::make_shared<RRT>(current_state));
  std::cout<<"ARRIVO PRIMA DI COSTRUIRE TUTTO2\n";

  value_rtree rrt_rtree;
  std::cout<<"ARRIVO PRIMA DI COSTRUIRE TUTTO3\n";

  rrt_rtree.insert(std::make_pair(point3d(current_state[0], current_state[1], current_state[2]), root_));;
  std::cout<<"ARRIVO PRIMA DI COSTRUIRE TUTTO4\n";

  // std::shared_ptr<octomap::OcTree> ot;
  //   std::cout<<"ARRIVO PRIMA DI COSTRUIRE TUTTO5\n";
  
  // boost::shared_ptr<octomap_msgs::Octomap const> shared_octo_msg;
  // octomap_msgs::Octomap octo_msg ;
  // std::cout<<"ARRIVO PRIMA DI COSTRUIRE TUTTO5\n";

  // shared_octo_msg = ros::topic::waitForMessage<octomap_msgs::Octomap>("octomap_full",ros::Duration(10));
  // std::cout<<"ARRIVO PRIMA DI COSTRUIRE TUTTO5\n";

  // if(shared_octo_msg != NULL){
  //   octo_msg = *shared_octo_msg;
  //   std::cout<<"octomsg emptty";
  // }
  // std::cout<<"ARRIVO PRIMA DI COSTRUIRE TUTTO6\n";

  // std::cout<< octo_msg <<std::endl;
  // octomap::AbstractOcTree* aot = octomap_msgs::msgToMap(octo_msg);
  // octomap::OcTree* ot_tmp = (octomap::OcTree*)aot;
  // ot_ = std::make_shared<octomap::OcTree>(*ot_tmp);

  // delete ot_tmp;
  // ot = ot_;


  std::cout<<"ARRIVO PRIMA DI COSTRUIRE TUTTO 5\n";

  octomap::point3d min(current_state[0] - max_sampling_radius - 0.5,
                       current_state[1] - max_sampling_radius - 0.5,
                       current_state[2] - max_sampling_radius - 0.5);

  octomap::point3d max(current_state[0] + max_sampling_radius + 0.5,
                       current_state[1] + max_sampling_radius + 0.5,
                       current_state[2] + max_sampling_radius + 0.5);
  std::cout<<"ARRIVO PRIMA DI COSTRUIRE TUTTO 6\n";

  // if(ot_ == NULL)
  //   as->setSucceeded();


  std::shared_ptr<point_rtree> octomap_rtree = std::make_shared<point_rtree>(getRtree(ot_, min, max));
  std::cout<<"ARRIVO PRIMA DI COSTRUIRE TUTTO DEFINITIVO\n";

  nav_msgs::Path path_to_goal = root_->findTrajectory(ot_,octomap_rtree,&rrt_rtree,current_state,state_to_reach);

  // Check if target is reached...
  std::cout<<"ARRIVO PRIMA DEL FOR FOR PATH\n";
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
  }
  
  as->setSucceeded();
}

