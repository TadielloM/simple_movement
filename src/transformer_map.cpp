#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/ModelStates.h>

tf::Transform transform;
tf::Quaternion q;

void state_callback(const gazebo_msgs::ModelStates::ConstPtr &state)
{
    static tf::TransformBroadcaster br;

    q.setX(state->pose[1].orientation.x);
    q.setY(state->pose[1].orientation.y);
    q.setZ(state->pose[1].orientation.z);
    q.setW(state->pose[1].orientation.w);

    transform.setOrigin(tf::Vector3(
        state->pose[1].position.x,
        state->pose[1].position.y,
        state->pose[1].position.z));
    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "transformer_map");
    ros::NodeHandle n("~");
    ros::Subscriber state_sub = n.subscribe("/gazebo/model_states", 1, state_callback);
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}