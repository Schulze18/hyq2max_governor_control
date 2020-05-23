// Includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/ContactsState.h"


void groundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg)
{

    ROS_INFO("I heard: [%s]", msg->child_frame_id.c_str());
    ROS_INFO("%f %f %f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

}

void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{

    ROS_INFO("I heard joint: %s", msg->name[0].c_str());
    ROS_INFO("%f %f %f", msg->position[0], msg->position[1], msg->position[2]);

}

void footBumperCallback(const gazebo_msgs::ContactsState::ConstPtr& msg)
{

    ROS_INFO("I heard bumper: %s", msg->states[0].collision2_name.c_str());
    ROS_INFO("%f %f %f", msg->states[0].contact_positions[0].x, msg->states[0].contact_positions[0].y, msg->states[0].contact_positions[0].z);

}



// Main Loop
int main(int argc, char **argv)
{

    ros::init(argc, argv, "hyq2max_governor_control_node");

    ros::NodeHandle node;

    ros::Subscriber sub_ground_truth = node.subscribe("/hyq2max/ground_truth", 1000, groundTruthCallback);

    ros::Subscriber sub_joint_states = node.subscribe("/hyq2max/joint_states", 1000, jointStatesCallback);

    // boost::bind(
    ros::Subscriber sub_foot_bumper = node.subscribe("/lf_foot_bumper", 1000, footBumperCallback);


    ros::spin();
    
    return 0;
}

