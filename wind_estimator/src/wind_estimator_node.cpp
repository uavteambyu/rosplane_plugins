//
// Created by tim on 11/2/17.
//
#include "wind_estimator/wind_estimator.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wind_estimator");
    ros::NodeHandle nh; // get a handle to ROS

    wind_estimator::windEstimator Thing();

    ros::spin();

    return 0;
}
