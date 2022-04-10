#include "ford_mot/tracking.h"
#include <iostream>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "ford_mot/eigenmvn.h"

#include <ros/ros.h>
#include <math.h>

bool DEBUG = false;

Tracking::Tracking()
{
    ros::NodeHandle nh;

    is_initialized_ = false;
    previous_timestamp_  = ros::Time::now(); // zero

    // State Matrix
    ekf_.x_ = Eigen::VectorXd(4);

    // State Covariance Matrix
    ekf_.P_ = Eigen::MatrixXd(4,4);
    ekf_.P_ <<  100, 0, 0, 0, 
                0, 100, 0, 0,
                0, 0, 100, 0, 
                0, 0, 0, 100;
    
    // State Transition Matrix
    ekf_.F_ = Eigen::MatrixXd(4,4);
    ekf_.F_ <<  1, 0, 1, 0, 
                0, 1, 0, 1, 
                0, 0, 1, 0, 
                0, 0, 0, 1;
    
    if (!nh.getParam ("ekf/Qv_ax", Qv_ax)){ Qv_ax = 0.1;}  //Eigen::EigenMultivariateNormal<float> normX_solver1(mean,covar);     
    if (!nh.getParam ("ekf/Qv_ay", Qv_ay)){ Qv_ay = 0.1;}       

    // Radar Measurement Covariance Matrix
    if (!nh.getParam ("ekf/R_Radar_range_lon", R_Radar_range_lon)){ R_Radar_range_lon = 0.4;} //m   
    if (!nh.getParam ("ekf/R_Radar_range_lon", R_Radar_range_lat)){ R_Radar_range_lat = 0.4;} //m   
    if (!nh.getParam ("ekf/R_Radar_vel_lon", R_Radar_vel_lon)){ R_Radar_vel_lon = 0.027;} //m/s
    if (!nh.getParam ("ekf/R_Radar_vel_lat", R_Radar_vel_lat)){ R_Radar_vel_lat = 0.027;} //m/s    

    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 30);   
    posePub = nh.advertise<nav_msgs::Path>("path", 5);
}

Tracking::~Tracking(){}

std::pair<Eigen::VectorXd, Eigen::MatrixXd> Tracking::ProcessMeasurement(const std::string sensorType, const Eigen::VectorXd measurements)
{
    // Set sensor type
    ekf_.sensorType_ = sensorType;

    if(!is_initialized_)
    {
        if(DEBUG) ROS_INFO("Initializing Tracking");
        // Set current everything to zero (x, y, vx, vy)
        ekf_.x_ << 0, 0, 0, 0;

        // previous_timestamp_ = measurement_package.timestamp_;
        previous_timestamp_ = ros::Time::now();

        is_initialized_ = true;
        return {ekf_.x_,ekf_.P_};
    }
    // time between last sensor reading and current reading
    // float dt = (measurement_package.timestamp_ - previous_timestamp_) / 1000000.0;
    current_timestamp_ = ros::Time::now();
    time_diff_ = current_timestamp_ - previous_timestamp_;
    float dt = time_diff_.toSec();
    previous_timestamp_ = current_timestamp_;
    // ROS_INFO("dt: %.2f",dt);

    double dt2_2 = dt * dt / 2;
    
    // dt of x
    ekf_.F_(0,2) = dt;
    // dt of y
    ekf_.F_(1,3) = dt;

    // Q = G Qv G(^T)
    Eigen::MatrixXd G = Eigen::MatrixXd(4,2);
    G <<    dt2_2 ,0,
            0, dt2_2,
            dt, 0,
            0, dt;

    Eigen::MatrixXd Qv = Eigen::MatrixXd(2,2);
    Qv <<   Qv_ax*Qv_ax, 0,
            0, Qv_ay*Qv_ay;

    ekf_.Q_ = G * Qv * G.transpose();

    if(DEBUG) ROS_INFO("!!!!!!!!!!!!!!!!!! \n Predicting...");
    ekf_.Predict();
    if(DEBUG)
    {
        std::cout << "x_= " << ekf_.x_ << std::endl;
        std::cout << "P_= " << ekf_.P_ << std::endl;
        ROS_INFO("Predict Complete...");
    }

    if(sensorType == "RADAR")
    {
        if(DEBUG) ROS_INFO("Setting RADAR");
        ekf_.H_ = Eigen::MatrixXd::Identity(4,4);
              
        ekf_.R_ = Eigen::MatrixXd(4,4);
        ekf_.R_ <<  R_Radar_range_lon*R_Radar_range_lon, 0, 0, 0,
                    0, R_Radar_range_lat*R_Radar_range_lat, 0, 0,
                    0, 0, R_Radar_vel_lon*R_Radar_vel_lon, 0,
                    0, 0, 0, R_Radar_vel_lat*R_Radar_vel_lat;                    
    }    
    
    // ROS_INFO("Updating...");
    ekf_.Update(measurements);

    // ROS_INFO("***********");
    // std::cout << "x_= " << ekf_.x_[0] << std::endl;
    // std::cout << "P_= " << ekf_.P_.coeff(0,0) << std::endl;

    // PublishOdom();
    // PublishPathMessage();

    // ekf_.x_, ekf_.P_;
    return {ekf_.x_, ekf_.P_};
}

void Tracking::PublishPathMessage()
{
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.stamp = current_timestamp_;
    poseStamped.header.frame_id = "pose";
    poseStamped.header.frame_id = "map";
    poseStamped.pose.position.x = ekf_.x_[0];
    poseStamped.pose.position.y = ekf_.x_[1];

    pathMsg.header.stamp = current_timestamp_;
    pathMsg.header.frame_id = "pose";
    pathMsg.header.frame_id = "map";

    pathMsg.poses.push_back(poseStamped);

    posePub.publish(pathMsg);

}

void Tracking::PublishOdom()
{
    // Publish transformation and position
    odom_quat = tf::createQuaternionMsgFromYaw(ekf_.x_[2]);
    
    odom_trans.header.stamp = current_timestamp_;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = ekf_.x_[0];
    odom_trans.transform.translation.y = ekf_.x_[1];
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);

    odom.header.stamp = current_timestamp_;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = ekf_.x_[0];
    odom.pose.pose.position.y = ekf_.x_[1];
    odom.pose.pose.position.z = 0.0;    
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = ekf_.x_[3];
    odom.twist.twist.linear.y = ekf_.x_[4];
    odom.twist.twist.angular.z = ekf_.x_[5];

    //publish the message
    odom_pub.publish(odom);
}