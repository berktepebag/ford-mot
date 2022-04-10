#include "ford_mot/main.h"

#include <stdio.h>
#include <iostream>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

#include "eigen3/Eigen/Dense"
#include "ford_mot/measurement_package.h"
#include "ford_mot/tracking.h"

#include <vector>


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
        // Predicted Points will be created when a track exits (x_, P_)
        std::vector<std::pair<Eigen::VectorXd, Eigen::MatrixXd>> predictedPoints;
        // Tentative tracks will exists until either deleted or converted into comfirmedTrack
        std::vector<std::pair<Eigen::VectorXd, Eigen::MatrixXd>> tracks;

        // Global Counter, counts steps since tracker initialized
        int GLOBAL_COUNTER = 0;



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
        GLOBAL_COUNTER++;

        // ROS_INFO("Beginning Pred.Point.Size: %d", predictedPoints.size());

        sensorType = "RADAR";
        if(DEBUG) ROS_INFO("Radar call back");   

        // If there is no pred. points yet, we first predict and add them to the tentative track list
        if(true){
        // if(predictedPoints.size() == 0){

            for(int col=0;col<10;col++)
            {            
                // If Obj Id == 0 continue
                if(msg->data[col+20] == 0) continue;

                // for(int row=0; row<14; row++)
                // {
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

                    auto [x_, p_] = tracking.ProcessMeasurement(sensorType, radarMeas);   
                   std::cout << "RESULTS ARE COMING *******" << x_ << "\n" << p_ << std::endl;

                    predictedPoints.push_back(tracking.ProcessMeasurement(sensorType, radarMeas) );
                    // ROS_INFO("Adding Pred.Point.Size: %d", predictedPoints.size());
                // }            
            }
        }

        ROS_INFO("End Pred.Point.Size: %d", predictedPoints.size());
        // for (auto &pp:predictedPoints){std::cout << "*** x_:" << pp.first << " \n";}

        // ROS_INFO("Global Counter: %d", GLOBAL_COUNTER);

        //JPDAF Starts Here

        // Get Predicted Points from previous step

        // Get current Observations

        // For each Predicted Point Calculate Mahalanobis Distance dij2

        // Create Hypothesis 

        // Pick Highest Prob. "Pred. Point - Observation" Pair

        // Track Maintenance Called

        // For tentative and comfirmed tracks run KF, predict next Predicted Points 





        
    }

};

int main(int argc, char **argv)
{
    //1- Subscribe to Observations
    ros::init(argc, argv, "JPDAF"); 
    SensorClass RadarSensor; 
    ros::spin();

    return 0;
}
