#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

float e_T = 0.0;
float e_TOld = 0.0;
float timeStep = 0.1;
float errorIntegral = 0.0;
float tenDegrees = 0.174; //ten degrees in radians
float twentyDegrees = 0.349; //20 degrees in radians

void errorUpdate(const geometry_msgs::PointStamped::ConstPtr& msg) {
    e_T = msg->point.x;
}

float calculateControlInput() {
    //PID constants set here
    float Kp = 3.0;
    float Kd = 0.10;
    float Ki = 0.10;

    //Update the error integral
    errorIntegral = e_T * timeStep + errorIntegral;

    //Calculate the error derivative
    float errorDerivative = e_T - e_TOld;
    errorDerivative = errorDerivative / timeStep;

    //Calculate the PID control input
    float u = (Kp * e_T + Ki * errorIntegral + Kd * errorDerivative);

    //Updte the "old" error value
    e_TOld = e_T;

    return u;
}

float calculateCarSpeed(float steeringAngle) {
    if (abs(steeringAngle) < twentyDegrees) {
        if (abs(steeringAngle) < tenDegrees) {
            return 1.5;
        } else {
            return 1.0;
        }
    } else {
        return 0.5;
    }
}

int main(int argc, char** argv){ 
    
    ros::init(argc, argv, "car_controller");
    ros::NodeHandle nh;
    ros::Subscriber error_sub = nh.subscribe("/wall_follow_error",1, errorUpdate);
    ros::Publisher drive_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/drive",1);
    
    float steeringAngle;
    float carSpeed;
    ackermann_msgs::AckermannDriveStamped driveMessage;

    while(ros::ok()) {
        
        steeringAngle = calculateControlInput();
        carSpeed = calculateCarSpeed(steeringAngle);

        driveMessage.drive.steering_angle = steeringAngle;
        driveMessage.drive.speed = carSpeed;
        driveMessage.header.stamp = ros::Time::now();

        drive_pub.publish(driveMessage);

        ros::Duration(timeStep).sleep();
        ros::spinOnce();
    }

}