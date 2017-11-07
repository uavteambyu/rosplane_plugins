//
// Created by tim on 11/2/17.
//

#include <ros/ros.h>
#include <rosplane_plugin_msgs/WindEstimate.h>
#include "bomb_drop/bomb_drop.h"
#include <cstdlib>
#include <math.h>
namespace bomb_drop
{


    bombDrop::bombDrop(float timeStep, float initialVelocity, float mass, float S, float Cd, float rho ) :
            nh_(ros::NodeHandle()), nhPrivate_(ros::NodeHandle("~"))
    {
        currentIndex = 0;
        stateArray[0].location.n = 0;
        stateArray[0].location.e = 0;
        stateArray[0].location.d = 0;
        stateArray[0].velocity.n = initialVelocity;
        stateArray[0].velocity.e = 0;
        stateArray[0].velocity.d = 0;
        this->timeStep = timeStep;
        this->mass = mass;
        this->S = S;
        this->Cd = Cd;
        this->rho = rho;
        finishedComputation = false;
        ROS_INFO("Computing the Drop Location");
        computeDropLocation();
        finishedComputation = true;
        ROS_INFO("Finished computing the Drop Location");
        windEstimateSubscriber = nh_.subscribe("wind_estimate", 10, &bombDrop::windCallback,this);
        waypointPublisher = nh_.advertise<rosplane_msgs::Waypoint>("waypoint_path",10);
        ROS_INFO("The waypoint for the drop is: n(%f) e(%f) d(%f)",Zdrop.n,Zdrop.e,Zdrop.d);
    }

    void bombDrop::windCallback(const rosplane_plugin_msgs::WindEstimate &msg) {
        if (msg.confidence > 0.9 && windValid != true){
            windEstimate.wn = msg.wn;
            windEstimate.we = msg.we;
            windValid = true;
            addInWind();
            createWaypointPath();
            publishWaypointPath();
        }
    }

    void bombDrop::addInWind(){
        windAngle = atan2(windEstimate.we, windEstimate.wn);
        Pdrop.n = Ptarget.n - Tdrop*windEstimate.wn - cos(windAngle)*Zdrop.n + sin(windAngle)*Zdrop.e;
        Pdrop.e = Ptarget.e - Tdrop*windEstimate.we - sin(windAngle)*Zdrop.n - cos(windAngle)*Zdrop.e;
        Pdrop.d = Ptarget.d - Zdrop.d;
    }

    void bombDrop::createWaypointPath(){
        path[0] = createWaypoint(false);
        path[0].w[0] = Pdrop.n;
        path[0].w[1] = Pdrop.e;
        path[0].w[2] = Pdrop.d;
        path[1] = createWaypoint(true);
        path[1].w[0] = Pdrop.n - cos(windAngle)*Zdrop.n + sin(windAngle)*Zdrop.e;
        path[1].w[1] = Pdrop.e - sin(windAngle)*Zdrop.n - cos(windAngle)*Zdrop.e;
        path[1].w[2] = Pdrop.d;
    }

    rosplane_msgs::Waypoint bombDrop::createWaypoint(bool current){
        rosplane_msgs::Waypoint waypoint;
        waypoint.Va_d = stateArray[0].velocity.n;
        waypoint.chi_valid = false;
        waypoint.chi_d = windAngle*180/M_PI;
        waypoint.set_current = current;
        waypoint.clear_wp_list = false;
    }

    void bombDrop::publishWaypointPath(){

    }

    void bombDrop::computeDropLocation(){
        while (stateArray[currentIndex].location.d < 50){
            currentIndex++;
            computeNextVStep();
            computeNextZStep();
        }
        Tdrop = currentIndex*timeStep;
        Zdrop = stateArray[currentIndex].location;
        addInWind();
        //Vdrop.n = stateArray[0].velocity.n*

    }
    void bombDrop::computeNextVStep(){
        stateArray[currentIndex].velocity.n = stateArray[currentIndex-1].velocity.n + timeStep*((-1/(2*mass))*rho*S*Cd*magnitude(stateArray[currentIndex-1].velocity)*stateArray[currentIndex-1].velocity.n);
        stateArray[currentIndex].velocity.e = stateArray[currentIndex-1].velocity.e + timeStep*((-1/(2*mass))*rho*S*Cd*magnitude(stateArray[currentIndex-1].velocity)*stateArray[currentIndex-1].velocity.e);;
        stateArray[currentIndex].velocity.d = stateArray[currentIndex-1].velocity.d + timeStep*(9.8 - (1/(2*mass))*rho*S*Cd*magnitude(stateArray[currentIndex-1].velocity)*stateArray[currentIndex-1].velocity.d);

    }
    void bombDrop::computeNextZStep(){
        stateArray[currentIndex].location.n = stateArray[currentIndex-1].velocity.n*(timeStep) + stateArray[currentIndex-1].location.n;
        stateArray[currentIndex].location.e = stateArray[currentIndex-1].velocity.e*(timeStep) + stateArray[currentIndex-1].location.e;
        stateArray[currentIndex].location.d = stateArray[currentIndex-1].velocity.d*(timeStep) + stateArray[currentIndex-1].location.d;
    }

    float bombDrop::magnitude(coordinate& coord){
        return (sqrt(pow(coord.n,2)+pow(coord.e,2)+pow(coord.d,2)));
    }



};
