//
// Created by tim on 11/2/17.
//
#include <ros/ros.h>
#include "bomb_drop/bomb_drop.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bomb_drop_node"); // register the node on ROS
    ros::NodeHandle nh; // get a handle to ROS

    bomb_drop::bombDrop Thing;  // instatiate our class object

    ros::spin(); // check for new messages and call the callback if we get one

    return 0;
}
