//ROS
#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>

//ROS Messages
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int8.h>

//Namespaces
using namespace std;

//Global variables
ros::Publisher LeaderTurn;

int rotationAck;
double zTol = 0.3;

void odomCallback(nav_msgs::Odometry odomData){
    if (abs(odomData.twist.twist.angular.z) > zTol) {
        rotationAck = 1;
    } else
        rotationAck = 0;
}


int main (int argc, char** argv)
{
    //ROS Initialization
    ros::init(argc, argv, "master");
    ROS_INFO("Master Turtlebot is Connected\n");
    ros::NodeHandle nh_("~");//ROS Handler - local namespace

    //Subscribing to topics
    ros::Subscriber sub_odom = nh_.subscribe<nav_msgs::Odometry>
            ("/master/odom", 1000, odomCallback);

    //Publishing to topics
    LeaderTurn = nh_.advertise<std_msgs::Int8>("/master/status", 1);

    ros::Rate rate(20) ;
    ROS_INFO("Turtlebot Master Node spinning @ 20Hz");

    while (ros::ok()){

        // Attend callbacks
        ros::spinOnce();

        std_msgs::Int8 ackMsg;
        ackMsg.data = rotationAck;
        LeaderTurn.publish(ackMsg);

    rate.sleep();

    }
    ROS_INFO("ROS-Node Terminated\n");
}


