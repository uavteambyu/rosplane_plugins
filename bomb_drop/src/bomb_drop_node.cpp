//
// Created by tim on 11/2/17.
//
#include <ros/ros.h>
#include "bomb_drop/bomb_drop.h"
#include <cstdlib>
#include <math.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "bomb_drop_node"); // register the node on ROS
    ros::NodeHandle nh; // get a handle to ROS
    float timeStep = .001;
    float Va = 14;
    float Cd = .5; //drag coefficient for a sphere
    float S = M_PI*pow(.01,2);
    float rho = .051;

    bomb_drop::bombDrop Thing(timeStep,Va,S,Cd,rho);  // instatiate our class object

    ros::spin(); // check for new messages and call the callback if we get one

    return 0;
}
