#ifndef SLAVE_FUNCTIONS_H_INCLUDED
#define SLAVE_FUNCTIONS_H_INCLUDED

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

//**************************************
//*** Parameters for linear velocity ***
//**************************************
//+++ Tracking phase +++
extern double zPosLeader;   // Distance from master
extern float zThresh;       // Reference distance from master
extern double Kp;           // Gain to apply on error when positive
extern double Kp_neg;       // Gain to apply to error when negative
extern double zErr;         // Distance error from master
extern double linearSpeed;  // Linear speed evaluated on distance error from master
extern double lastLinVel;   // Last saved linear speed
extern double slowVelocity; // Velocity used to slow down smoothly when losing marker

//+++ Odometry based approaching +++
extern double xOriTol;          // Value used to reset odometry whenever the aligment is satisfying
extern double odomErr;          // Error used to evaluate linear speed
extern double odomTol;          // Threshold for defining the robot arrived
extern double SAVED_distance;   // Distance saved when the master has turned
extern double xOdomFollower;    // Current x value of slave's odometry
extern double refDistance;      // Continuously updated distance in odometry topic


//***************************************
//*** Parameters for angular velocity ***
//***************************************
//+++ Tracking phase +++
extern double Ka;           // Gain to apply on error
extern double xPosLeader;   // Position of marker in image
extern float oriThresh;     // Reference position for marker x coordinate in image
extern double oriErr;       // Orientation error with master alignment
extern double angSpeed;     // Angular speed evaluated on marker position in image
extern double lastAngVel;   // Last saved angular speed

//+++ Odometry based approaching +++
extern double zOriErr;      // Error on orientation between master and slave
extern double zOriTol;      // Threshold for alignment in orientation
extern double zOdomLeader;  // Orientation of master based on odometry
extern double zOdomFollower;// Orientation of slave based on odometry


// **********************
// *** Flags & status ***
// **********************
extern int markerStatus;  // Marker Status(1: not tracked, 3-4: tracked)
extern int masterStatus;  // Master Status (0: idle status, 1: turning)
extern bool masterTurned; // Flag raised if the master has turned
extern bool turnFinished; // Flag raised if the slave has finished turning
extern int i;             // Flag used to execute commands only once

// *****************
// *** Functions ***
// *****************

// This function stops the robot whether the marker is lost.
void slaveStop(geometry_msgs::Twist cmd_msg, ros::Publisher pub_cmd);

// This function provides a behavior for the turtlebot to follow the marker.
//
void followMarker(geometry_msgs::Twist cmd_msg, ros::Publisher pub_cmd, ros::Publisher pub_rstodometry);

void reachMasterPostionAndTurn(geometry_msgs::Twist cmd_msg, ros::Publisher pub_cmd, ros::Publisher pub_rstodm, ros::Publisher pub_rstMasodm, ros::Publisher pub_st);

#endif