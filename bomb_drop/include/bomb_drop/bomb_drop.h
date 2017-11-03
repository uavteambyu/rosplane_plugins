//
// Created by tim on 11/2/17.
//

#ifndef PROJECT_BOMB_DROP_H
#define PROJECT_BOMB_DROP_H
#include <ros/ros.h>
#include <rosplane_plugins/WindEstimate.h>
namespace rosplane_plugins
{
    class bombDrop{

    public:
        bombDrop();
    private:
        ros::NodeHandle nh_; //public for subscribing, publishing, etc
        ros::NodeHandle nhPrivate_; //private for pulling parameter values from the parameter server
        ros::Subscriber windEstimateSubscriber;
        ros::Publisher waypointPublisher;

        void windCallback(const rosplane_plugins::WindEstimate &msg);

    };



};

#endif //PROJECT_BOMB_DROP_H
