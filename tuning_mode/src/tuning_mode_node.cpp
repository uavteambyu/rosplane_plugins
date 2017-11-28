//
// Created by tim on 11/20/17.
//
#include <ros/ros.h>
#include "tuning_mode/tuning_mode.h"
#include <cstdlib>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "tuning_mode_node"); // register the node on ROS
    ros::NodeHandle nh; // get a handle to ROS

    bomb_drop::bombDrop Thing();  // instatiate our class object

    ros::spin(); // check for new messages and call the callback if we get one

    return 0;
}
