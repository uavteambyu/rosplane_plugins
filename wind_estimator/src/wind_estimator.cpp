//
// Created by tim on 11/2/17.
//
#include "wind_estimator/wind_estimator.h"
#include <ros/ros.h>
#include <rosplane_plugin_msgs/WindEstimate.h>
#include <cstdlib>
#include <math.h>

namespace wind_estimator
{


    windEstimator::windEstimator() :
          nh_(ros::NodeHandle()), nhPrivate_(ros::NodeHandle("~"))
    {
        ArrayIndex = 0;
        PresentSampleNumber = 0;
        ArrayFilled = false;
        StdDeviation.we = 0;
        StdDeviation.wn = 0;
        Mean.we = 0;
        Mean.wn = 0;
        WindEstimatePublisher = nh_.advertise<rosplane_plugin_msgs::WindEstimate>("wind_estimate",10);
        StateSubscriber = nh_.subscribe("state", 10, &windEstimator::stateCallback, this);
    }

    void windEstimator::stateCallback(const rosplane_msgs::State &msg){
        WindSamples.wn[ArrayIndex] = msg.wn;
        WindSamples.we[ArrayIndex] = msg.we;
        //ROS_INFO("Added wind data, n(%f) e(%f), to sample index %d. We have recorded %d samples.",
          //       WindSamples.wn[ArrayIndex], WindSamples.we[ArrayIndex], ArrayIndex, PresentSampleNumber+1);
        if(ArrayIndex < MaxSampleNumber-1)
        {
            ArrayIndex = ArrayIndex+1;
        } else
        {
            ArrayIndex = 0;
            ArrayFilled = true;
            ROS_INFO("Sample array filled. New samples will overwrite old values beginning with the first measurement.");
        }
        if(PresentSampleNumber < MaxSampleNumber) {
            PresentSampleNumber = PresentSampleNumber + 1;
            ROS_INFO("Sample array not yet full.");
        } else{
            ROS_INFO("Sample array full. Publishing estimate based on %d samples.", MaxSampleNumber);
            publishWindEstimate();

        }
    }
    void windEstimator::publishWindEstimate() //publish wind
    {
        calculateAverage();
        calculateStdDev();
        windEstimate.wn = Mean.wn;
        windEstimate.we = Mean.we;
        windEstimate.confidence = calculateConfidence();
        windEstimatePublisher.publish(windEstimate);
    }

    void windEstimator::calculateAverage()//function to calculate sample average
    {
        float WnAccumulated = 0;
        float WeAccumulated = 0;

        for(uint8_t n = 0; n<PresentSampleNumber; n++)
        {
            //iterate through the values for wn
            WnAccumulated += WindSamples.wn[n];
            //iterate through the values for we
            WeAccumulated += WindSamples.we[n];
        }
        //store in corresponding mean variable after dividing by sample number
        Mean.wn = WnAccumulated/PresentSampleNumber;
        Mean.we = WeAccumulated/PresentSampleNumber;
        //ROS_INFO("Average wind value is: n(%f) e(%f)", Mean.wn, Mean.we);
    }

    void windEstimator::calculateStdDev()//function to calculate std deviation
    {
        windEstimator::calculateAverage();
        float DifferenceSumWn = 0;
        float DifferenceSumWe = 0;
        for(uint8_t n = 0; n<PresentSampleNumber; n++)
        {
            DifferenceSumWn += pow((WindSamples.wn[n]-Mean.wn),2);
            DifferenceSumWe += pow((WindSamples.we[n]-Mean.we),2);
        }
        StdDeviation.wn = sqrt((DifferenceSumWn/(PresentSampleNumber-1)));
        StdDeviation.we = sqrt((DifferenceSumWe/(PresentSampleNumber-1)));
    }

    float windEstimator::calculateConfidence()// function to calculate "confidence"
    {
        double RatioStdDevMeanWn = 0; //compares std deviation to a percentage of mean
        double RatioStdDevMeanWe = 0;
        double PercentageWn = 0;
        double PercentageWe = 0;


        if(Mean.wn==0)
        {
            ROS_INFO("Zero mean in north direction. Standard deviation is (%f).", StdDeviation.wn);
        }
        else
            RatioStdDevMeanWn = (PercentNearMean*abs(Mean.wn))/StdDeviation.wn;
        if(Mean.we==0)
        {
            ROS_INFO("Zero mean in east direction. Standard deviation is (%f).", StdDeviation.we);
        }
        else
            RatioStdDevMeanWe = (PercentNearMean*abs(Mean.we))/StdDeviation.we;

        PercentageWe = 2*(phi(RatioStdDevMeanWe)-phi(0));
        PercentageWn = 2*(phi(RatioStdDevMeanWn)-phi(0));

        ROS_INFO("(%f) percent of measurements are within %f percent of the mean for the north direction.", PercentageWn*100, 100*PercentNearMean);
        ROS_INFO(" mean %f in east direction. Standard deviation is (%f). Their ratio is %f",Mean.we, StdDeviation.we, RatioStdDevMeanWe);
        ROS_INFO("(%f) percent of measurements are within %f percent of the mean for the east direction.", PercentageWe*100, 100*PercentNearMean);

        return std::min(PercentageWe, PercentageWn);

    }


    float windEstimator::phi(float x)
    {//implementation is open domain from https://www.johndcook.com/blog/cpp_phi/
        // constants
        const double a1 =  0.254829592;
        const double a2 = -0.284496736;
        const double a3 =  1.421413741;
        const double a4 = -1.453152027;
        const double a5 =  1.061405429;
        const double p  =  0.3275911;

        // Save the sign of x
        int sign = 1;
        if (x < 0)
            sign = -1;
        x = fabs(x)/sqrt(2.0);

        // A&S formula 7.1.26
        double t = 1.0/(1.0 + p*x);
        double y = 1.0 - (((((a5*t + a4)*t) + a3)*t + a2)*t + a1)*t*exp(-x*x);

        return 0.5*(1.0 + sign*y);
    }
}



