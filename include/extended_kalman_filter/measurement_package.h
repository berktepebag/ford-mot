#ifndef MEASUREMENT_PACKAGE_H
#define MEASUREMENT_PACKAGE_H

#include "ros/ros.h"
#include <eigen3/Eigen/Dense>

class MeasurementPackage{
    public:

    enum SensorType{
        IMU,ENCODER,RADAR
    } sensor_type;

    ros::Time timestamp_;
    Eigen::VectorXd raw_measurements_;

    // int64_t timestamp;
};

#endif