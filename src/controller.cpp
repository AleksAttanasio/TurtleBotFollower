//ROS
#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

//ROS Messages
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>


//Namespaces
using namespace std;

//Parameters for linear velocity
float zThresh = 0.5;
//double zTol = 0.05;

//linear gain
double Kp = .2;
double Kp_neg = 0.7;

//initializing speed and error for linear parameters
double linearSpeed = 0.0;
double zErr = 0.0;

// Parameters for angluar velocity
float oriThresh = 0.0;
double xThresh = 0.0;
//double oriTol = 0.05;

// angular gain
double Ka = 2;
//double Ka_neg = 0.7;

//initializing speed and error for angular parameters
double angSpeed = 0.0;
double oriErr = 0.0;
double xErr = 0.0;
double odomErr = 0.0;
double odomTol = 0.15;
double zOriTol = 0.05;
double zOriErr = 0.0;


//Global variables
ros::Publisher pub_command ;
ros::Publisher pub_rstodom;
ros::Publisher pub_state;
ros::Publisher pub_rstMasterodom;

double xPosLeader;
double yPosLeader;
double zPosLeader;
double zOriLeader;
int markerStatus;
int masterStatus;
double lastLinVel = 0.0;
double lastAngVel = 0.0;
double slowVelocity = 0.0;
double SAVED_distance = 0.0;
double refDistance = 0.0;
bool masterTurned;
bool turnFinished=false;
int i = 0;

void leaderCallback(geometry_msgs::PoseStamped leader){
    // Copy leader information into global variables
    xPosLeader            = leader.pose.position.x  ;
    yPosLeader            = leader.pose.position.y  ;
    zPosLeader            = leader.pose.position.z  ;
    zOriLeader          = leader.pose.orientation.z ;
}

//double xOdomLeader;
//double yOdomLeader;
//double zOdomLeader;
//void leaderOdomCallback(nav_msgs::Odometry odomData){
//    xOdomLeader = odomData.pose.pose.position.x;
//    yOdomLeader = odomData.pose.pose.position.y;
//    zOdomLeader = odomData.pose.pose.orientation.z;
//}

//double xOdomFollower;
//double yOdomFollower;
//double zOdomFollower;
//void followerOdomCallback(nav_msgs::Odometry odomData){
//    xOdomFollower = odomData.pose.pose.position.x;
//    yOdomFollower = odomData.pose.pose.position.y;
//    zOdomFollower = odomData.pose.pose.orientation.z;
//}

void markerCallback(std_msgs::Int8 marker){
    markerStatus =  marker.data;
}

//void rotAckCallback(std_msgs::Int8 ack){
//    masterStatus = ack.data;
//}

int main (int argc, char** argv) {
    //ROS Initialization
    ros::init(argc, argv, "track_turtlebot");
    ROS_INFO("Node initialized.");
    ros::NodeHandle nh_("~");//ROS Handler - local namespace

    //Subscribing
    ROS_INFO("Subscribing to topics\n");

    //ros::Subscriber pose
    ros::Subscriber Leader
            = nh_.subscribe<geometry_msgs::PoseStamped>("/visp_auto_tracker/object_position", 1, leaderCallback);
//    ros::Subscriber MasterRotation
//            = nh_.subscribe<std_msgs::Int8>("/master/status", 1, rotAckCallback);
    ros::Subscriber Marker
            = nh_.subscribe<std_msgs::Int8>("/visp_auto_tracker/status", 1, markerCallback);
//    ros::Subscriber sub_master_odom
//            = nh_.subscribe<nav_msgs::Odometry>("/master/odom", 1, leaderOdomCallback);
//    ros::Subscriber sub_slave_odom
//            = nh_.subscribe<nav_msgs::Odometry>("/odom", 1, followerOdomCallback);
    usleep(600000);

    //Publishing follower velocity
    pub_command = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
//    pub_rstodom = nh_.advertise<std_msgs::Empty>("/mobile_base/commands/reset_odometry",1);
//    pub_rstMasterodom = nh_.advertise<std_msgs::Empty>("/master/mobile_base/commands/reset_odometry",1);
//    pub_state = nh_.advertise<std_msgs::Int8>("/slave_state",1);

    ros::Rate rate(100);
    ROS_INFO("Turtle control spinning @ 100Hz");

    while (ros::ok()) {

        // Attend callbacks
        ros::spinOnce();
        geometry_msgs::Twist cmd_msg;
        //      If the master turns raise the flag
        if (masterStatus == 1) {
            masterTurned = true;
        }

        //      if the marker is not tracked and the master is "idle" than stand still
        if (markerStatus == 1 && masterStatus == 0 && masterTurned==false) {
            ROS_INFO("No Info Received");
            slowVelocity = lastLinVel;

            if (slowVelocity > 0.007) {
                slowVelocity = slowVelocity * 0.8;
                cmd_msg.linear.x = slowVelocity;
                lastLinVel = slowVelocity;
            }
            pub_command.publish(cmd_msg);
        }
        //      else if the marker is tracked and the master is "idle" reach it
        else if (markerStatus!= 1) {
            //ROS_INFO("Following marker.");
            zErr = zPosLeader - zThresh;
            xErr = xPosLeader - xThresh;
            //                        oriErr = zOriLeader - oriThresh;

            cout << "Tracking error: " << zErr << endl;
            cout << "Z Pose error: " << zPosLeader << endl;

            //increase the gain when robots are too close, back away faster
            if (zErr < 0) {
                Kp = Kp_neg;
            }

            linearSpeed = zErr * Kp;
            angSpeed = -Ka*1.5 * xErr;

            if (abs(linearSpeed - lastLinVel) > 0.1) {
                linearSpeed = lastLinVel;
            }
            if (abs(angSpeed - lastLinVel) > 0.5) {
                angSpeed = lastAngVel;
            }

            cmd_msg.linear.x = linearSpeed;
            lastLinVel = linearSpeed;
            cmd_msg.angular.z = angSpeed;
            lastAngVel = angSpeed;

            pub_command.publish(cmd_msg);
        }

    }

    ROS_INFO("ROS-Node Terminated\n");

}

