#ifndef TRACKING_H_
#define TRACKING_H_

#include "ford_mot/kalman_filter.h"
#include "ford_mot/measurement_package.h"
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "nav_msgs/Path.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class Tracking{
    public:
        Tracking();
        virtual ~Tracking();

        void ProcessMeasurement(const std::string sensorType,const Eigen::VectorXd measurements);
        ExtKalmanFilter ekf_;

        void PublishOdom();

    private:

        bool is_initialized_;
        // int64_t previous_timestamp_;
        
        ros::Time current_timestamp_, previous_timestamp_;
        ros::Duration time_diff_;

        //acceleration noise components
        float Qv_ax,Qv_ay;
        // Radar noise components
        float R_Radar_range_lon, R_Radar_range_lat, R_Radar_vel_lon, R_Radar_vel_lat;

        ros::Publisher odom_pub;   
        ros::Publisher posePub;

        // Odometry Publisher
        geometry_msgs::Quaternion odom_quat;
        geometry_msgs::TransformStamped odom_trans;
        // Point Publisher
        nav_msgs::Path pathMsg;

        
        tf::TransformBroadcaster odom_broadcaster;

        nav_msgs::Odometry odom;

        void PublishPathMessage();

};

#endif