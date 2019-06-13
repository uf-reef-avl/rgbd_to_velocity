//
// Created by humberto on 5/20/19.
//
#include <ros/ros.h>
#include "rgbd_to_velocity.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "rgbd_to_velocity_node");
    rgbd_to_velocity::RgbdToVelocity rgbd_to_velocity_object;
    ros::spin();
    return 0;
}

