#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <iostream>

//Global Variables
ros::Publisher brake_pub, brake_bool_pub, ttc_pub, ttc_min_pub;

std_msgs::Bool brake_bool_msg;
std_msgs::Float64 ttc_min, ttc_threshold;
ackermann_msgs::AckermannDriveStamped brake_msg;

float angleIncrement = 0.00436332309619;    //Radians, from the /scan rostopic
float* vx = new float;                      //Pointer to the current speed variable
float ttcThreshold = 0.70;                 //Minimum time-to-collision before applying the brake
float zero = 0.0;                           //Just a 0,0 as a float

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    //std::cout << msg->ranges[269] << std::endl;
    float rDot;
    float angle;
    float tcc;
    float v = 1.5;
    float ttcMin = 1000;
    ttc_threshold.data = ttcThreshold;
    ttc_min.data = ttcMin;
    for (int i = 0; i < msg->ranges.size(); i++) {
        //For each beam, calculate its angle, and the velocity the car is moving in that direction
        angle = i * angleIncrement;
        rDot = -1.0*v*cos(angle);
        rDot = std::max(rDot,zero);
        //Calculate the time-to-collision in this direction
        tcc = msg->ranges[i] / rDot;
	//std::cout << tcc << std::endl;
        //Filter out the -Inf's
        if (tcc < 0)
            tcc = 100000;
	if (tcc < ttcMin) {
	    ttcMin = tcc;
	}
        //Check if any of the ttc is less than some acceptable threshold
        if (tcc < ttcThreshold) {
            //If the ttc is < threshold, apply the brake
	    std::cout << "braking" << std::endl;
            brake_pub.publish(brake_msg);
            brake_bool_msg.data = true;
            brake_bool_pub.publish(brake_bool_msg);
        }
    }
    ttc_min.data = ttcMin;
    ttc_min_pub.publish(ttc_min);
    ttc_pub.publish(ttc_threshold);
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    //This function updates the global velocity value
    *vx = msg->twist.twist.linear.x;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scan_answer");
  ros::NodeHandle n;

  ros::Subscriber scan_sub = n.subscribe("/scan", 1, scanCallback);
  ros::Subscriber odom_sub = n.subscribe("/odom", 1, odometryCallback);
  brake_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/high_level/ackermann_cmd_mux/input/nav_0",1);
  ttc_pub = n.advertise<std_msgs::Float64>("/ttc_threshold",1);
  ttc_min_pub = n.advertise<std_msgs::Float64>("/ttc_min",1);
  brake_bool_pub = n.advertise<std_msgs::Bool>("/brake_bool",1);

  ros::spin();

  return 0;
}
