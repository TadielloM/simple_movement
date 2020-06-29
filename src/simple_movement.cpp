#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>



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
        ros::Subscriber sub; 
        ros::Publisher pub;
        ros::NodeHandle* nh_;

    public:
    
        //Constructor
        BasicMovement()
        {
            gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
            gazebo_node_->Init();

            // Make sure the ROS node for Gazebo has already been initialized
            if (!ros::isInitialized()) {
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
                return;
            }
            // Create node handle
            nh_ = new ros::NodeHandle("/");
            this->pub = nh_->advertise<geometry_msgs::PoseStamped>("/position_drone", 1);
            this->sub = nh_->subscribe("/next_goal", 100, &BasicMovement::subCallback, this);  
        }

        //Destructor
        ~BasicMovement()
        {}

        void pubFunc(ignition::math::Pose3<double> actual_position){
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

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&BasicMovement::OnUpdate, this));

            //this->n.reset(new ros::NodeHandle("simple_movement_node"));
                 
        
        }

        //Subscriber Callback
        void subCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
        {
            actual_goal.Pos().X() = msg->pose.position.x;
            actual_goal.Pos().Y() = msg->pose.position.y;
            actual_goal.Pos().Z() = msg->pose.position.z;
            actual_goal.Rot().W() = msg->pose.orientation.w;
            actual_goal.Rot().X() = msg->pose.orientation.x;
            actual_goal.Rot().Y() = msg->pose.orientation.y;
            actual_goal.Rot().Z() = msg->pose.orientation.z;
        }

        
        // Called by the world update start event
        void OnUpdate()
        {
            
            ignition::math::Pose3<double> actual_position = this->model->WorldPose();

            pubFunc(actual_position);

            if (actual_position.Pos().Distance(actual_goal.Pos()) > 0)
            {
                pose = this->model->WorldPose();

                ignition::math::Vector3d velocity;
                velocity = (actual_goal.Pos() - actual_position.Pos()).Normalize() * 5;
                this->model->SetLinearVel(velocity);
            }
            else
            {
                this->model->SetLinearVel(actual_goal.Pos() * 0);
            }
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

}