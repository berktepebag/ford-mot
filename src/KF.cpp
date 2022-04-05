#include "ros/ros.h"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <iostream>

#include "eigen3/Eigen/Dense"
#include "extended_kalman_filter/measurement_package.h"
#include "extended_kalman_filter/tracking.h"


class SensorClass
{
    private:

        ros::NodeHandle nh;
        ros::Time currentTime, prevTime;

        ros::Duration deltaTime; float dt;

        ros::Subscriber radarSubs;

        Tracking tracking;
        Eigen::VectorXd radarMeas = Eigen::VectorXd(4);

        std::string sensorType;

        bool DEBUG = false;


    public:
        SensorClass()
        {
            radarSubs = nh.subscribe("lrrObjects",1, &SensorClass::radarCallback, this);
        
            currentTime = ros::Time::now();
            prevTime = currentTime;
        }

        ~SensorClass(){}



    void radarCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        sensorType = "RADAR";
        if(true) ROS_INFO("Radar call back");   

        for(int col=0;col<10;col++)
        {
            
            for(int row=0; row<14; row++)
            {
                if(msg->data[col+20] == 0) continue;

                if(DEBUG)
                {
                    ROS_INFO("Obj: %d \n S_Type: %.0f \n S_No: %.0f \n Obj_Id: %.0f \n  Lon_Dist: %f m \n  Lat_Dist: %f m \n  Lon_Vel: %f m/s \n  Lat_vel: %f m/s \n  w: %f \n  h: %f \n d: %f \n  Time: %.0f:%.0f:%.0f:%.0f ", 
                    col, 
                    msg->data[col],
                    msg->data[col+10],
                    msg->data[col+20],
                    msg->data[col+30],
                    msg->data[col+40],
                    msg->data[col+50],
                    msg->data[col+60],
                    msg->data[col+70],
                    msg->data[col+80],
                    msg->data[col+90],
                    msg->data[col+100],
                    msg->data[col+110],
                    msg->data[col+120],
                    msg->data[col+130]
                    );
                }

                radarMeas << 
                msg->data[col+30],
                msg->data[col+40],
                msg->data[col+50],
                msg->data[col+60];

                tracking.ProcessMeasurement(sensorType, radarMeas);   

            }            
        }
    }

};

int main(int argc, char **argv)
{  
  ros::init(argc, argv, "radarKF"); 
  SensorClass RadarSensor; 
  ros::spin();

  return 0;
}