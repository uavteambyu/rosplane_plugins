//
// Created by tim on 11/2/17.
//

#ifndef WIND_ESTIMATOR_H
#define WIND_ESTIMATOR_H

#include <ros/ros.h>
#include <rosplane_plugin_msgs/WindEstimate.h>
#include <rosplane_msgs/State.h>

namespace wind_estimator {
    class windEstimator {
    public:
        windEstimator();
    private:
        const uint8_t MaxSampleNumber = 50; //number of samples to be used for estimation
        struct wind_array{
            float wn[MaxSampleNumber];
            float we[MaxSampleNumber];
        };
        struct single_wind_sample{
            float wn;
            float we;
        };
        rosplane_plugin_msgs::WindEstimate windEstimate;
        ros::NodeHandle nh_; //public for subscribing, publishing, etc
        ros::NodeHandle nhPrivate_; //private for pulling parameter values from the parameter server
        ros::Subscriber StateSubscriber;
        ros::Publisher WindEstimatePublisher;
        single_wind_sample StdDeviation; //holds std_deviation for each direction (treat as independent random variables)
        single_wind_sample Mean; //holds means value for each direction
        uint8_t ArrayIndex;
        uint8_t PresentSampleNumber;
        wind_array WindSamples;
        bool ArrayFilled;

        void stateCallback(const rosplane_msgs::planeState &msg);
        void publishWindEstimate(); //publish wind
        void calculateAverage();//function to calculate sample average
        void calculateStdDev();//function to calculate std deviation
        float calculateConfidence();// function to calculate confidence
        float phi(float x); //function acting as lookup table for cumulative dist. function
    };
}

#endif //WIND_ESTIMATOR_H
