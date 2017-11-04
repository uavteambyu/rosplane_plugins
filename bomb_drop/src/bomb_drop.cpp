//
// Created by tim on 11/2/17.
//

#include <ros/ros.h>
#include <rosplane_plugins/WindEstimate.h>
#include <cstdlib>
#include <math.h>
namespace rosplane_plugins
{
class bombDrop {

    bombDrop::bombDrop(float timeStep, float initialVelocity, float mass, float S, float Cd, float rho ) :
            nh_(ros::NodeHandle()), nh_private_(ros::NodeHandle("~"))
    {
        currentIndex = 0;
        stateArray[0].position.n = 0;
        stateArray[0].position.e = 0;
        stateArray[0].position.d = 0;
        stateArray[0].velocity.n = initialVelocity;
        stateArray[0].velocity.e = 0;
        stateArray[0].velocity.d = 0;
        computeDropLocation();
        this.timeStep = timeStep;
        this.mass = mass;
        this.S = S;
        this.Cd = Cd;
        this.rho = rho;
        while (!windValid){

        }
        addInWind();
        createWaypointPath();
        publishWaypointPath();
    }

    void bombDrop::windCallback(const rosplane_plugins::WindEstimate &msg) {
        if (msg->confidence > 0){
            windEstimate.wn = msg->wn;
            windEstimate.we = msg->we;
            windValid = true;
        }
    }

    void bombDrop::addInWind(){
        windAngle = atan2(windEstimate.we, windEstimate.wn);
        Pdrop.n = Ptarget.n - Tdrop*windEstimate.wn - cos(windAngle)*Zdrop.location.n + sin(windAngle)*Zdrop.location.e;
        Pdrop.e = Ptarget.e - Tdrop*windEstimate.we - sin(windAngle)*Zdrop.location.n - cos(windAngle)*Zdrop.location.e;
        Pdrop.d = Ptarget.d - Zdrop.location.d;
    }

    void bombDrop::createWaypointPath(){
        path[0] = createWaypoint();
        path[0].w[0] = Pdrop.n;
        path[0].w[1] = Pdrop.e;
        path[0].w[2] = Pdrop.d;
        path[1] = createWaypoint();
        path[1].w[0] = Pdrop.n - cos(windAngle)*Zdrop.location.n + sin(windAngle)*Zdrop.location.e;
        path[1].w[1] = Pdrop.e - sin(windAngle)*Zdrop.location.n - cos(windAngle)*Zdrop.location.e;
        path[1].w[2] = Pdrop.d;
    }

    Waypoint bombDrop::createWaypoint(bool current){
        Waypoint waypoint();
        waypoint.Va_d = stateArray[0].velocity.n;
        waypoint.chi_valid = false;
        waypoint.chi_d = windAngle*180/M_PI;
        waypoint.set_current = current;
        waypoint.clear_wp_list = false;
    }

    void bombDrop::publishWaypointPath(){

    }

    void bombDrop::computeDropLocation(){
        while (stateArray[currentIndex].position.d > 50){
            currentIndex++;
            computeNextVStep();
            computeNextZStep();
        }
        Tdrop = currentIndex*timeStep;
        Zdrop = stateArray[currentIndex];
        addInWind();
        //Vdrop.n = stateArray[0].velocity.n*

    }
    void bombDrop::computeNextVStep(){
        stateArray[currentIndex].velocity.n = stateArray[currentIndex-1].velocity.n + timeStep*((-1/(2*mass))*rho*S*Cd*magnitude(stateArray[currentIndex-1].velocity)*stateArray[currentIndex-1].velocity.n);
        stateArray[currentIndex].velocity.e = stateArray[currentIndex-1].velocity.e + timeStep*((-1/(2*mass))*rho*S*Cd*magnitude(stateArray[currentIndex-1].velocity)*stateArray[currentIndex-1].velocity.e);;
        stateArray[currentIndex].velocity.d = stateArray[currentIndex-1].velocity.d + timeStep*(9.8 - (1/(2*mass))*rho*S*Cd*magnitude(stateArray[currentIndex-1].velocity)*stateArray[currentIndex-1].velocity.d);

    }
    void bombDrop::computeNextZStep(){
        stateArray[currentIndex].location.n = stateArray[currentIndex-1].velocity.n*(timeStep) + stateArray[currentIndex].location.n;
        stateArray[currentIndex].location.e = stateArray[currentIndex-1].velocity.e*(timeStep) + stateArray[currentIndex].location.e;
        stateArray[currentIndex].location.d = stateArray[currentIndex-1].velocity.d*(timeStep) + stateArray[currentIndex].location.d;
    }

    float bombDrop::magnitude(coordinate& coord){
        return (sqrt(pow(coord.n,2),pow(coord.e,2),pow(coord.d)));
    }




};



};