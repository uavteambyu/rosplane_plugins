//
// Created by tim on 11/2/17.
//
#include "wind_estimator.h"

#include <cstdlib>
#include <math.h>

namespace wind_estimator {
    windEstimator::windEstimator(){
        ArrayIndex = 0;
        PresentSampleNumber = 0;
        ArrayFilled = false;
        StdDeviation->we = 0;
        StdDeviation->wn = 0;
        Mean->we = 0;
        Mean->wn = 0;
    }
    void windEstimator::stateCallback(const rosplane_msgs::planeState &msg){
        WindSamples.wn[ArrayIndex] = msg.wn;
        WindSamples.we[ArrayIndex] = msg.we;
        ROS_INFO("Added wind data, n(%f) e(%f), to sample index %d. We have recorded %d samples.",
                 WindSamples.wn[ArrayIndex], WindSamples.we[ArrayIndex], ArrayIndex, PresentSampleNumber+1);
        if(ArrayIndex < 49)
        {
            ArrayIndex = ArrayIndex+1;
        } else
        {
            ArrayIndex = 0;
            ArrayFilled = true;
            ROS_INFO("Sample array filled. New samples will overwrite old values beginning with the first measurement.");
        }
        if(PresentSampleNumber < 50)
            PresentSampleNumber = PresentSampleNumber+1;
    }
    void windEstimator::publishWindEstimate(); //publish wind

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
        ROS_INFO("Average wind value is: n(%f) e(%f)", Mean.wn, Mean.we);
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
        float RatioStdDevMeanWn = 0;
        float RatioStdDevMeanWe = 0;
        if(2*StdDeviation.wn/abs(Mean.wn)<=0.1 && 2*StdDeviation.we/abs(Mean.we)<=0.1)
        {
            ROS_INFO(" //print out what percentage is within 10% of mean
            return //whichever is less accurate of the two directions
        } else
        {
            ROS_INFO(" //print out that not x% is within 10% of mean
            return //whichever is less accurate of the two directions
        }
    }
}



