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
    private:
        const uint8_t SampleNumber = 50; //number of samples to be used for estimation
        struct wind {
            float n[SampleNumber];
            float e[SampleNumber];
        };
        rosplane_plugin_msgs::WindEstimate windEstimate;
        ros::NodeHandle nh_; //public for subscribing, publishing, etc
        ros::NodeHandle nhPrivate_; //private for pulling parameter values from the parameter server
        ros::Subscriber StateSubscriber;
        ros::Publisher WindEstimatePublisher;
        float StdDeviation;
        float Mean;
        uint8_t ArrayIndex;



        void stateCallback(const rosplane_msgs::planeState &msg);
        void publishWindEstimate(); //publish wind
        //function to calculate std deviation

        //function to calculate confidence
        //
    };
}

#endif //WIND_ESTIMATOR_H
