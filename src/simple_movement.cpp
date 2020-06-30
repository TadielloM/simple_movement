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
        //public: std::unique_ptr<ros::NodeHandle> n;
        ignition::math::Pose3<double> actual_goal;
        ignition::math::Pose3<double> pose;
        std::string name;
        uint32_t counter_msg = 0;

    public:
        transport::NodePtr gazebo_node_;
        ros::NodeHandle *nh_;
        FlyTo fly_to;

    public:
        //Constructor
        BasicMovement()
        {
            gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
            gazebo_node_->Init();

            // Make sure the ROS node for Gazebo has already been initialized
            if (!ros::isInitialized())
            {
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                 << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
                return;
            }
            // Create node handle
            nh_ = new ros::NodeHandle("/");
        }

        //Destructor
        ~BasicMovement()
        {
        }

        //Load
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {

            this->model = _parent;
            //Setup of this drone Agent
            name = this->model->GetName();
            fly_to.setModel(this->model);

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&BasicMovement::OnUpdate, this));

            std::cout<<"Basic Moviment started!\n";
            //this->n.reset(new ros::NodeHandle("simple_movement_node"));
        }

        // Called by the world update start event
        void OnUpdate()
        {
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