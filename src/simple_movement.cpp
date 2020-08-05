#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include "fly_to.hh"

namespace gazebo
{
    class BasicMovement : public ModelPlugin
    {

    private: std::shared_ptr<ros::NodeHandle> nh_;

        //public: std::unique_ptr<ros::NodeHandle> n;
        ignition::math::Pose3<double> actual_goal;
        ignition::math::Pose3<double> pose;
        std::string name;
        uint32_t counter_msg = 0;
        uint8_t status=3;
        ignition::math::Vector3d linear_vel;
        ignition::math::Vector3d angular_vel;

    public:
        transport::NodePtr gazebo_node_;
       
        FlyTo *fly_to;
        ros::Publisher pub;
    
        //Constructor
        BasicMovement()
        {
            // gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
            // gazebo_node_->Init();

            // Make sure the ROS node for Gazebo has already been initialized
            
            
        }

        //Destructor
        ~BasicMovement()
        {
        }

        void pubFunc(ignition::math::Pose3<double> actual_position)
        {
            geometry_msgs::PoseStamped msg;
            msg.header.seq = counter_msg++;
            msg.header.frame_id = "map";
            msg.pose.position.x = actual_position.Pos().X();
            msg.pose.position.y = actual_position.Pos().Y();
            msg.pose.position.z = actual_position.Pos().Z();
            msg.pose.orientation.w = actual_position.Rot().W();
            msg.pose.orientation.x = actual_position.Rot().X();
            msg.pose.orientation.y = actual_position.Rot().Y();
            msg.pose.orientation.z = actual_position.Rot().Z();

            pub.publish(msg);
        }

        //Load
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {

            this->model = _parent;
            //Setup of this drone Agent
            name = this->model->GetName();
            
            std::cout << this->model << std::endl;
            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&BasicMovement::OnUpdate, this));

            std::cout << "Basic Moviment started!\n";
            // this->n.reset(new ros::NodeHandle("simple_movement_node"));
            // Create node handle
            // nh_ = new ros::NodeHandle();
            if (!ros::isInitialized())
            {
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                 << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
                return;
                // int argc = 0;
                // char **argv = NULL;
                // ros::init(argc, argv, "fly_to_node");
            }
            
            // ros::init();
            
            this->nh_.reset(new ros::NodeHandle("~"));
            fly_to = new FlyTo(nh_);
            this->pub = nh_->advertise<geometry_msgs::PoseStamped>("/position_drone", 1);
            linear_vel = ignition::math::Vector3d(0,0,0);
            angular_vel =ignition::math::Vector3d(0,0,0);
            fly_to->setModel(this->model);
            fly_to->setVelocitiesPointer(&linear_vel,&angular_vel);
            ros::Rate(20);
            std::cout<< "URI: " <<ros::master::getURI() << std::endl;
            std::cout<< " The Namespace is: " << nh_->getNamespace() <<std::endl;
            std::vector< std::string > nodes;
            ros::master::getNodes(nodes);

            std::cout<< "The active nodes are: "<< std::endl;
            for (auto i : nodes){
                std::cout<< i <<std::endl;
            }
        }

        // Called by the world update start event
        void OnUpdate()
        {
            ignition::math::Pose3<double> actual_position = this->model->WorldPose();
            
            this->model->SetLinearVel(linear_vel);
            this->model->SetAngularVel(angular_vel);
            pubFunc(actual_position);
            ros::spinOnce();
            
        }


        // Pointer to the model
    private:
        physics::ModelPtr model;

        // Pointer to the update event connection
    private:
        event::ConnectionPtr updateConnection;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(BasicMovement)

} // namespace gazebo