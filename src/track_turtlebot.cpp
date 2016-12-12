#include "track/slave_functions.h"

//Namespaces
using namespace std;

// ******************
// *** Publishers ***
// ******************

ros::Publisher pub_command ;
ros::Publisher pub_rstodom;
ros::Publisher pub_state;
ros::Publisher pub_rstMasterodom;

// *****************
// *** Callbacks ***
// *****************

void leaderCallback(geometry_msgs::PoseStamped leader){
    xPosLeader            = leader.pose.position.x  ;
    zPosLeader            = leader.pose.position.z  ; }

void leaderOdomCallback(nav_msgs::Odometry odomData){
    zOdomLeader = odomData.pose.pose.orientation.z; }

void followerOdomCallback(nav_msgs::Odometry odomData){
    xOdomFollower = odomData.pose.pose.position.x;
    zOdomFollower = odomData.pose.pose.orientation.z; }

void markerCallback(std_msgs::Int8 marker){
    markerStatus =  marker.data; }

void rotAckCallback(std_msgs::Int8 ack){
    masterStatus = ack.data; }

// ************
// *** MAIN ***
// ************

int main (int argc, char** argv) {
    // *** ROS Initialization ***
    ros::init(argc, argv, "track_turtlebot");
    ROS_INFO("Node initialized.");
    ros::NodeHandle nh_("~");//ROS Handler - local namespace
    ROS_INFO("Subscribing to topics\n");

    // *******************
    // *** Subscribers ***
    // *******************
    ros::Subscriber Leader
            = nh_.subscribe<geometry_msgs::PoseStamped>("/visp_auto_tracker/object_position", 1, leaderCallback);
    ros::Subscriber MasterRotation
            = nh_.subscribe<std_msgs::Int8>("/master/status", 1, rotAckCallback);
    ros::Subscriber Marker
            = nh_.subscribe<std_msgs::Int8>("/visp_auto_tracker/status", 1, markerCallback);
    ros::Subscriber sub_master_odom
            = nh_.subscribe<nav_msgs::Odometry>("/master/odom", 1, leaderOdomCallback);
    ros::Subscriber sub_slave_odom
            = nh_.subscribe<nav_msgs::Odometry>("/odom", 1, followerOdomCallback);
    usleep(600000);

    // ******************
    // *** Publishers ***
    // ******************
    pub_command = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    pub_rstodom = nh_.advertise<std_msgs::Empty>("/mobile_base/commands/reset_odometry",1);
    pub_rstMasterodom = nh_.advertise<std_msgs::Empty>("/master/mobile_base/commands/reset_odometry",1);
    pub_state = nh_.advertise<std_msgs::Int8>("/slave_state",1);

    ros::Rate rate(100);
    ROS_INFO("Turtle control spinning @ 100Hz");

    while (ros::ok()) {

        ros::spinOnce();
        geometry_msgs::Twist cmd_msg;

        // If the master turns raise the flag
        if (masterStatus == 1) {
            masterTurned = true; }
        // If the marker is not tracked and the master is "idle" than stand still...
        if (markerStatus == 1 && masterStatus == 0 && !masterTurned) {
            slaveStop(cmd_msg, pub_command); }
        // ...else if the marker is tracked and the master is "idle" reach it
        else if (markerStatus!= 1 && masterStatus == 0 && !masterTurned) {
            followMarker(cmd_msg, pub_command, pub_rstodom); }
        // ... else if the master has turned reach it and turn to reach its orientation.
        else if (masterTurned) {
            ROS_INFO("Master Turning");
            std_msgs::Int8 ready;
            ready.data = 0;

            // At first iteration save the distance from master as reference for odometry
            if (i == 0) {
                SAVED_distance = zPosLeader;
                refDistance = (xOdomFollower + SAVED_distance) + .15;
                i++;
            }

            usleep(600000);
            // Evaluate odometry error at each sampling step and linear speed to follow
            std::cout << "Reference Distance: " << refDistance << std::endl;
            std::cout << "Odometry Distance x: " << xOdomFollower << std::endl;
            odomErr = refDistance - xOdomFollower;
            std::cout << "Odometry Distance error: " << odomErr << std::endl;
            linearSpeed = odomErr * Kp * 0.22;
            std::cout << "Linear Speed:" << linearSpeed << std::endl;
            cmd_msg.linear.x = linearSpeed;
            cmd_msg.angular.z = 0.0;
            pub_command.publish(cmd_msg);

            // If the error is below the threshold than turn to the same master's orientation
            if(abs(odomErr) < odomTol){

                // Evaluate error on orientation between master and slave and angular speed
                zOriErr = (zOdomLeader - zOdomFollower);
                angSpeed = zOriErr * Ka * 0.4;

                std::cout << "Turning ANGULAR error:" << zOriErr << std::endl;

                // If the slave has turned to the master's orientation then raise the turnFinished flag
                if (abs(zOriErr) < zOriTol) {
                    turnFinished = true;
                }

                //Publish speed on command topics
                cmd_msg.angular.z = angSpeed;
                pub_command.publish(cmd_msg);
            }

            // If the slave is aligned to the master then reset the flags and send a ready message on
            // salve_status topic and reset odometries
            if(abs(odomErr) < odomTol && turnFinished){
                // Resetting the flags
                masterTurned = false;
                turnFinished = false;
                i = 0;

                // Reset odometry of both master and slave
                std_msgs::Empty rstOdom;
                pub_rstodom.publish(rstOdom);
                pub_rstMasterodom.publish(rstOdom);

                // Publish flag and speeds on slave_status and command topics
                ready.data = 1;
                cmd_msg.angular.z = 0.0;
                cmd_msg.linear.x = 0.0;
                pub_command.publish(cmd_msg);

                usleep(60000);
            }

            pub_state.publish(ready);
        }
        // ... else the status of the robot is uknown
        else{
            ROS_INFO("*************** Unknown status. ***************");
        }
        rate.sleep();
    }
    ROS_INFO("ROS-Node Terminated\n");
}