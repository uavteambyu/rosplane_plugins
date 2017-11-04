//
// Created by tim on 11/2/17.
//

#ifndef BOMB_DROP_H
#define BOMB_DROP_H
#include <ros/ros.h>
#include <rosplane_plugin_msgs/WindEstimate.h>
#include <rosplane_msgs/Waypoint.h>
namespace bomb_drop
{
    class bombDrop{

    public:
        bombDrop(float timeStep, float initialVelocity, float mass, float S, float Cd, float rho );
    private:
        struct coordinate{
            float n;
            float e;
            float d;
        };
        struct locationVelocity{
            coordinate location;
            coordinate velocity;
        };
        ros::NodeHandle nh_; //public for subscribing, publishing, etc
        ros::NodeHandle nhPrivate_; //private for pulling parameter values from the parameter server
        ros::Subscriber windEstimateSubscriber;
        ros::Publisher waypointPublisher;

        void windCallback(const rosplane_plugin_msgs::WindEstimate &msg);
        void computeDropLocation();
        void addInWind();
        void createWaypointPath();
        rosplane_msgs::Waypoint createWaypoint(bool current);
        void publishWaypointPath();
        void computeNextVStep();
        void computeNextZStep();
	float windAngle;
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
        rosplane_plugin_msgs::WindEstimate windEstimate;
        rosplane_msgs::Waypoint path[10];
        locationVelocity stateArray[5000];



    };



};

#endif //PROJECT_BOMB_DROP_H
