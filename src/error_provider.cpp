#include "ros/ros.h"
#include <iostream>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PointStamped.h"

#define ANGLE_A_INDEX 480
#define ANGLE_B_INDEX 270
float radiansBetweenScans = 0.00582315586507;
float theta = 1.22;
float L = 0.3;
geometry_msgs::PointStamped error;

ros::Publisher error_pub;

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    int numberOfelements = msg->ranges.size(); //1080 in the sim

    float a = msg->ranges[ANGLE_A_INDEX];
    float b = msg->ranges[ANGLE_B_INDEX];
    float top = a*cos(theta) - b;
    float bottom = a*sin(theta);
    float alpha = top/bottom;
    float D_T = b*cos(alpha);
    float D_T1 = D_T + L*sin(alpha); 
    error.point.x = 1 - D_T1;
    error_pub.publish(error);
}

int main (int argc, char** argv) {

    ros::init(argc, argv, "error_provider");
    ros::NodeHandle nh;
    ros::Subscriber laser_sub = nh.subscribe("/scan", 1000, laserScanCallback);
    error_pub = nh.advertise<geometry_msgs::PointStamped>("/wall_follow_error",1);
    ros::spin();

  return 0;
}