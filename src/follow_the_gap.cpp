#include "ros/ros.h"
#include <iostream>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PointStamped.h"
#include <algorithm>
#include <iterator>
#include "ackermann_msgs/AckermannDriveStamped.h"


float radiansBetweenScans = 0.00582315586507;
int centerIdx = 540;
int feildOfView = 60;
float timeStep = 0.1;
int target_idx = 540;

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    //I cant get the copy function to work so here we go
    //(find the smallest point while we are at it)
    float closestPoint = 100.0;
    int closestIndex = 0;
    const int numberOfelements = msg->ranges.size();
    float ranges[540];

    for (int i = 0; i < 540; i++) {
        ranges[i] = msg->ranges[i + 270];
        if (ranges[i] < closestPoint) {
            closestPoint = ranges[i];
            closestIndex = i;
        }
    }

    //std::cout << "The closest Point is " << closestIndex << std::endl;
    //std::cout << "the range vector is " << ranges[closestIndex] << std::endl;

    // //Zero out the Saftey Bubble in the scan message
    int safetyRadius = 180;
    for (int i = 0; i < safetyRadius; i++) {
        if (closestIndex + i < 540)
            ranges[closestIndex + i] = 0.0;
        if (closestIndex - i > 0)
            ranges[closestIndex - i] = 0.0;
    }

    //Find the max length gap
    //Initialize the start of the max gap to be at teh start of the ranges array and its lengh to be 1
    int startOfMaxLengthGap = 0;
    int maxLengthGapSize = 1;
    int tempStartOfMaxLengthGap = 0;
    int tempMaxLengthGapSize = 1;
    for (int i = 1; i < 540; i++) {
        if (ranges[i] > 0.4 && i != (539)) {// If the next element in the ranges array is greater than zero and not at the end of the array, add one element to the size of the max gap
            tempMaxLengthGapSize = tempMaxLengthGapSize + 1;
        } else { //We are at the end of the arrary or have found the safety bubble, is the gap larger than the max?
            if (tempMaxLengthGapSize > maxLengthGapSize) {
                maxLengthGapSize = tempMaxLengthGapSize;
                startOfMaxLengthGap = tempStartOfMaxLengthGap;
            }
            tempMaxLengthGapSize = 0; //reset the size of the temporary max gap
            tempStartOfMaxLengthGap = i;
        }
    }
    //std::cout << "The max length gap starts at " << startOfMaxLengthGap << std::endl;
    //std::cout << "the max length gap size is " << maxLengthGapSize << std::endl;

    //Find the best goal point in the gap
    float maxLength = 0.0;
    int maxLengthIdx = startOfMaxLengthGap;
    for (int i = startOfMaxLengthGap; i < (startOfMaxLengthGap + maxLengthGapSize); i++) {
        if (ranges[i] > maxLength) {
            maxLength = ranges[i];
            maxLengthIdx = i;
        }
    }

    target_idx = maxLengthIdx;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "follow_the_gap");
    ros::NodeHandle nh;

    ros::Subscriber laser_sub = nh.subscribe("/scan", 1, laserScanCallback);
    ros::Publisher drive_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/drive",1);

    ackermann_msgs::AckermannDriveStamped driveMessage;
    while(ros::ok()) {
        float driveAngleIndex = target_idx - 270;
        float driveAngle = driveAngleIndex * radiansBetweenScans;

        driveMessage.drive.steering_angle = driveAngle;
        driveMessage.drive.speed = 2.0;
        driveMessage.header.stamp = ros::Time::now();

        drive_pub.publish(driveMessage);
    
        ros::Duration(0.025).sleep();
        ros::spinOnce();
    }
}