//
// Created by tim on 11/2/17.
//
#include "wind_estimator.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wind_estimator");
    rosplane::wind_estimator *wind_est = new rosplane::wind_estimator;

    ros::spin();

    return 0;
}