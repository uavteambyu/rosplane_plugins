//
// Created by tim on 11/2/17.
//
#include "wind_estimator/wind_estimator.h"
#include <ros/ros.h>
#include <cstdlib>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wind_estimator_node");
    ros::NodeHandle nh; // get a handle to ROS

    wind_estimator::windEstimator* Thing = new wind_estimator::windEstimator;


    ros::spin();


    return 0;
}
