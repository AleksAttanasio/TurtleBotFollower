#include "track/slave_functions.h"

// ******************
// *** Parameters ***
// ******************

double zPosLeader;           // Distance from master
float zThresh = 0.5;         // Reference distance from master
double Kp = .2;              // Gain to apply on error when positive
double Kp_neg = 0.7;         // Gain to apply to error when negative
double zErr = 0.0;           // Distance error from master
double linearSpeed = 0.0;    // Linear speed evaluated on distance error from master
double lastLinVel = 0.0;     // Last saved linear speed
double slowVelocity = 0.0;   // Velocity used to slow down smoothly when losing marker
double xOriTol = 0.05;       // Value used to reset odometry whenever the aligment is satisfying
double zOdomLeader;  // Orientation of master based on odometry
double zOdomFollower;// Orientation of slave based on odometry
double odomErr = 0.0;        // Error used to evaluate linear speed
double odomTol = 0.15;       // Threshold for defining the robot arrived
double SAVED_distance = 0.0; // Distance saved when the master has turned
double xOdomFollower;    // Current x value of slave's odometry
double refDistance = 0.0;    // Continuously updated distance in odometry topic
double Ka = 2;           // Gain to apply on error
double xPosLeader;       // Position of marker in image
float oriThresh = 0.0;   // Reference position for marker x coordinate in image
double oriErr = 0.0;     // Orientation error with master alignment
double angSpeed = 0.0;   // Angular speed evaluated on marker position in image
double lastAngVel = 0.0; // Last saved angular speed
double zOriErr = 0.0;        // Error on orientation between master and slave
double zOriTol = 0.05;       // Threshold for alignment in orientation
int markerStatus;        // Marker Status(1: not tracked, 3-4: tracked)
int masterStatus;        // Master Status (0: idle status, 1: turning)
bool masterTurned;       // Flag raised if the master has turned
bool turnFinished = false; // Flag raised if the slave has finished turning
int i = 0;               // Flag used to execute commands only once

// *****************
// *** Functions ***
// *****************
void slaveStop(geometry_msgs::Twist cmd_msg, ros::Publisher pub_cmd) {
    ROS_INFO("No Info Received");
    slowVelocity = lastLinVel;

    if (slowVelocity > 0.007) {
        slowVelocity = slowVelocity * 0.8;
        cmd_msg.linear.x = slowVelocity;
        lastLinVel = slowVelocity;
    }
    pub_cmd.publish(cmd_msg);
}

void followMarker(geometry_msgs::Twist cmd_msg, ros::Publisher pub_cmd, ros::Publisher pub_rstodometry){
    // Evaluate error on distance and orientation of master
    zErr = zPosLeader - zThresh;
    oriErr = xPosLeader - oriThresh;

    std::cout << "Tracking error: " << zErr << std::endl;
    std::cout << "Z Pose error: " << zPosLeader << std::endl;

    //increase the gain when robots are too close, back away faster
    if (zErr < 0) {
        Kp = Kp_neg;
    }

    // Evaluate linear and angular speed to follow the master
    linearSpeed = zErr * Kp;
    angSpeed = -Ka * oriErr;

    // Make every change of velocity smooth by avoiding jumps in speeds
    if (abs(linearSpeed - lastLinVel) > 0.1) {
        linearSpeed = lastLinVel;
    }
    if (abs(angSpeed - lastLinVel) > 0.5) {
        angSpeed = lastAngVel;
    }

    // If the alignment of slave and master is satisfying than reset odometry
    if(abs(xPosLeader) < xOriTol) {
        std_msgs::Empty rstOdom;
        pub_rstodometry.publish(rstOdom);
    }

    // Publish speeds on command topics
    cmd_msg.linear.x = linearSpeed;
    lastLinVel = linearSpeed;
    cmd_msg.angular.z = angSpeed;
    lastAngVel = angSpeed;
    pub_cmd.publish(cmd_msg);
}

void reachMasterPostionAndTurn(geometry_msgs::Twist cmd_msg, ros::Publisher pub_cmd, ros::Publisher pub_rstodm, ros::Publisher pub_rstMasodm, ros::Publisher pub_st){

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
    pub_cmd.publish(cmd_msg);

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
        pub_cmd.publish(cmd_msg);
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
        pub_rstodm.publish(rstOdom);
        pub_rstMasodm.publish(rstOdom);

        // Publish flag and speeds on slave_status and command topics
        ready.data = 1;
        cmd_msg.angular.z = 0.0;
        cmd_msg.linear.x = 0.0;
        pub_cmd.publish(cmd_msg);

        usleep(60000);
    }

    pub_st.publish(ready);
}