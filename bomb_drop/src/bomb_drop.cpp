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
        Ptarget.n = 0;
        Ptarget.e = 0;
        Ptarget.d = 0;
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
        stateSubscriber = nh_.subscribe("fixedwing/state", 10, &bombDrop::stateCallback, this);;
        commandPublisher = nh_.advertise<rosflight_msgs::Command>("command",10);
    }

    void bombDrop::stateCallback(const rosplane_msgs::State &msg) {
        stateCoord.n = msg.position[0];
        stateCoord.e = msg.position[1];
        stateCoord.d = msg.position[2];
        stateAirspeed = msg.Va;
        stateCourseAngle = msg.chi;
        if(inRange(stateAirspeed, stateArray[currentIndex].velocity.n, 1) &&
                inRange(stateCoord.n, Pdrop.n, 5) &&
                inRange(stateCoord.e, Pdrop.e, 5) &&
                inRange(stateCoord.d, Pdrop.d, 5) &&
                inRange(stateCourseAngle, windAngle, (10*M_PI/180))) {
            // drop the bomb
            rosflight_msgs::Command dropDaBomb;
            dropDaBomb.ignore = rosflight_msgs::Command::IGNORE_X |
                    rosflight_msgs::Command::IGNORE_Y | rosflight_msgs::Command::IGNORE_Z |
                    rosflight_msgs::Command::IGNORE_F;
            dropDaBomb.mode = rosflight_msgs::Command::MODE_PASS_THROUGH;
            dropDaBomb.bomb_drop = 1;
            commandPublisher.publish(dropDaBomb);
        }
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
        for(int i = 0; i < 2; ++i){
            waypointPublisher.publish(path[i]);
        }
    }

    void bombDrop::computeDropLocation(){
        while (stateArray[currentIndex].location.d < 50){
            currentIndex++;
            computeNextVStep();
            computeNextZStep();
        }
        Tdrop = currentIndex*timeStep;
        Zdrop = stateArray[currentIndex].location;
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

    bool bombDrop::inRange(float distCurr, float distDes, int maxDiff){
        return (abs(distCurr-distDes) <= maxDiff);
    }


};
