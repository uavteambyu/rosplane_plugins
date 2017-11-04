//
// Created by tim on 11/2/17.
//

#ifndef PROJECT_BOMB_DROP_H
#define PROJECT_BOMB_DROP_H
#include <ros/ros.h>
#include <rosplane_plugins_msgs/WindEstimate.h>
#include <rosplane_msgs/Waypoint.h>
namespace rosplane_plugins
{
    struct coordinate{
        float n;
        float e;
        float d;
    };
    struct locationVelocity{
        coordinate location;
        coordinate velocity;
    };
    class bombDrop{

    public:
        bombDrop(float timeStep, float initialVelocity, float mass, float S, float Cd, float rho );
    private:
        ros::NodeHandle nh_; //public for subscribing, publishing, etc
        ros::NodeHandle nhPrivate_; //private for pulling parameter values from the parameter server
        ros::Subscriber windEstimateSubscriber;
        ros::Publisher waypointPublisher;

        void windCallback(const rosplane_plugins::WindEstimate &msg);
        void computeDropLocation();
        void addInWind();
        void createWaypointPath();
        Waypoint createWaypoint(bool current);
        void publishWaypointPath();
        void computeNextVStep();
        void computeNextZStep();

        float magnitude(coordinate& coord);
        float timeStep;
        int currentIndex;
        bool windValid;
        float rho;
        float Cd;
        float S;
        float mass;
        coordinate Pdrop;
        coordinate Ptarget;
        coordinate Zdrop;
        float Tdrop;
        WindEstimate windEstimate;
        Waypoint path[10];
        locationVelocity stateArray[5000];



    };



};

#endif //PROJECT_BOMB_DROP_H
