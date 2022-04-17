#include "ford_mot/main.h"

#include <stdio.h>
#include <iostream>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

#include "eigen3/Eigen/Dense"
#include "ford_mot/measurement_package.h"
#include "ford_mot/tracking.h"

#include <vector>

#include "ford_mot/utilities.h"

class Track
{
    public:
        Track(int &TRACK_ID)
        {
            setID(TRACK_ID);
            std::cout << "Track ID: " << TRACK_ID << " established as Comfirmed: " << std::boolalpha << comfirmed << std::endl;            
        }
        ~Track(){}

        void tentativeToComfirmed()
        {
            comfirmed = true;
        }

        void setID(int &TRACK_ID)
        {
            ID = TRACK_ID;
            TRACK_ID++;
        }
        int getID(){return ID;}

        bool getStatus(){return comfirmed;}

        // Assign Observation point to the track points list

        auto getPredictedPointXandP(std::string sensorType, std::vector<Eigen::VectorXd> measurements)
        {
            return trackTracking.ProcessMeasurement(sensorType, measurements);
        }
        


    private:
        Tracking trackTracking;

        bool comfirmed = false;
        int ID = 0;

        // Each time a observation is assigned increase by one. If reaches threshold, call tentativeToComfirmed()
        int tentToComfCount = 0;
        int comfToDelCount = 0;

        // Hold x and y points to be drawen in RVIZ
        std::vector<std::pair<float, float>> trackPointsList;

        // state vector at k-1
        Eigen::VectorXd x_;
        // state covariance matrix
        Eigen::MatrixXd P_;


};

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
        std::vector<Track> tracksList;

        // Global Counter, counts steps since tracker initialized
        int GLOBAL_STEP_COUNTER = 0;
        // Track ID's for track maintenance
        int TRACK_ID = 0;
        
        Utilities utilities;

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
        std::vector<Eigen::VectorXd> measurements;
        GLOBAL_STEP_COUNTER++;

        // ROS_INFO("Beginning Pred.Point.Size: %d", predictedPoints.size());
        sensorType = "RADAR";
        if(DEBUG) ROS_INFO("Radar call back");   

        for(int col=0;col<10;col++)
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

            measurements.push_back(radarMeas);
        }

        // If there is no track in tracksList
        if(tracksList.size() == 0){
        
            // std::cout << "meas size: " << measurements.size() << std::endl;
            auto [x_, p_] = tracking.ProcessMeasurement(sensorType, measurements);
            // std::cout << "x_: " << x_ << " p_: " << p_ << std::endl;
            predictedPoints.push_back({x_, p_});
            // Add them as "tentavi tracks" to tracksList
            tracksList.emplace_back(Track(TRACK_ID));
        }

        for(auto track : tracksList)
        {
            std::cout << "Track ID: " << track.getID() << " Status: " << std::boolalpha << track.getStatus() << std::endl;
            std::cout << "Predicting Points using KF \n";    

            auto [x_, p_] = track.getPredictedPointXandP(sensorType, measurements); 
            std::cout << "Predicted Points x_: " << x_ << " p_: " << p_ << std::endl;

        }



        // ROS_INFO("GLOBAL_STEP_COUNTER: %d", GLOBAL_STEP_COUNTER);    
        // ROS_INFO("Predicted Points Size: %d", predictedPoints.size());    
        




        // for (auto &pp:predictedPoints){std::cout << "*** x_:" << pp.first << " \n";}

        // ROS_INFO("Global Counter: %d", GLOBAL_STEP_COUNTER);

        // *******JPDAF Starts Here

        // Get Predicted Points from previous step
        // ROS_INFO("End Pred.Point.Size: %d", predictedPoints.size());

        // Get current Observations
        // ROS_INFO("Measurements.Size: %d", measurements.size());
        // for (size_t i = 0; i < predictedPoints.size(); i++)
        // {
        //     // std::cout << "predictedPoints[i].first " << predictedPoints[i].first[0] << std::endl;
        //     float mu_x = predictedPoints[i].first[0];

        //     utilities.CalculateMahalanobisDistance(measurements, predictedPoints);


        //     // for (size_t j = 0; j < measurements.size(); j++)
        //     // {
        //     //     // Eigen::VectorXd dij_y = measurements[j][0]
        //     //     // std::cout << "measurements x: " << measurements[j][0] << std::endl;


        //     //     // Eigen::VectorXd dij_y = measurements[j][0] - mu_x;

        //     // }
            
        // }
        
        // ROS_INFO("Predicted points P sigmax: %f sigmay %f", )

        // float dij2 = 


        // Create Hypothesis 
        
        // For each Predicted Point Calculate Mahalanobis Distance dij2

        // Pick Highest Prob. "Pred. Point - Observation" Pair

        // Track Maintenance Called

        // For tentative and comfirmed tracks run KF, predict next Predicted Points 


        predictedPoints.empty();


        
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
