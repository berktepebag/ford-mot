#include "ford_mot/main.h"

#include <stdio.h>
#include <iostream>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

#include "eigen3/Eigen/Dense"
#include "ford_mot/measurement_package.h"
#include "ford_mot/tracking.h"
#include "ford_mot/utilities.h"

#include <vector>
#include <map>


class Track
{
    private:
        Tracking trackKF;

        bool comfirmed = false;
        int ID = 0;

        // Each time a observation is assigned increase by one. If reaches threshold, call tentativeToComfirmed()
        int tentToComfCount = 0;
        int comfToDelCount = 0;

        // Hold x and y points to be drawen in RVIZ
        std::vector<std::pair<float, float>> trackPointsList;


    public:
        Track(){}

        Track(int &TRACK_ID, Eigen::VectorXd x_)
        {
            ROS_INFO("\n****************\n Track ID: %d established as Tentavi @ \n x: %f y: %f vx: %f vy: %f \n****************", TRACK_ID, x_(0), x_(1), x_(2), x_(3));       
            setID(TRACK_ID);
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

        std::vector<float> calculateGij(std::vector<Eigen::VectorXd> measurementsList)
        {
            // std::cout << "calculate Gij \n";
            return trackKF.GijCalculation(measurementsList);
        }

        void kfUpdate(Eigen::VectorXd chosenMeasurement)
        {
            trackKF.update(chosenMeasurement);
        }


        void addTrackPointsToList(float xPoint, float yPoint)
        {
            trackPointsList.push_back(std::pair<float,float>(xPoint, yPoint) );

        }

        void addPredictedPointsToList(float xPoint, float yPoint)
        {
            trackPointsList.push_back(std::pair<float,float>(xPoint, yPoint) );

        }

};

class SensorClass
{
    private:

        ros::NodeHandle nh;
        ros::Time currentTime, prevTime;
        ros::Duration deltaTime; float dt;

        ros::Subscriber radarSubs;

        // Tracking tracking;
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

        const float PD = 0.75;
        const float BETA = 0.03;
 

    public:
        SensorClass()
        {
            radarSubs = nh.subscribe("lrrObjects",1, &SensorClass::radarCallback, this);
        
            currentTime = ros::Time::now();
            prevTime = currentTime;

        }

        static bool sortbysec(const std::pair<int,int> &a,
              const std::pair<int,int> &b)
        {
            return (a.second < b.second);
        }

        ~SensorClass(){}

    void radarCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        std::vector<Eigen::VectorXd> measurementsList;
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

            measurementsList.push_back(radarMeas);
        }

        std::cout << "meas size: " << measurementsList.size() << std::endl;

        // If there is no track in tracksList
        if(tracksList.size() == 0){
            // Add teach observation as "tentavi tracks" to tracksList
            for(auto measurement : measurementsList)
            {
            tracksList.emplace_back(Track(TRACK_ID,measurement));            
            }
        }

        // Calculate gij(s)
        std::vector<std::vector<float>> gijList; 
        for(auto track : tracksList)
        {
            std::cout << "----Track ID: " << track.getID() << " Status: " << std::boolalpha << track.getStatus() << "\nCalculating gij...\n";
            // std::cout << "Meas. List size: " << measurementsList.size() <<"\n";             

            gijList.push_back(track.calculateGij(measurementsList));
        }
        std::cout << "gijList size: " << gijList.size() <<"\n";             

        std::vector<std::vector<int>> hypothesisCombinations;
        hypothesisCombinations = utilities.hypothesisCombinations(gijList);

        // Hyp No. and it's calculated likelihood
        std::vector<std::pair<int,float>> hypothesisLikelihoods;
        std::vector<std::pair<int,float>> normHyptLikelihoods;

        //Hypothesis #
        for (int hypo = 0; hypo < hypothesisCombinations.size(); hypo++)
        {
            float hypLikelihood = 1;
            //Track
            for (int track = 0; track < hypothesisCombinations[hypo].size(); track++)
            {
                if(false){
                std::cout 
                << "Hypot.: " << hypo+1 
                << "\n Track: " << track+1 
                << "\n Selected Meas.: " << hypothesisCombinations[hypo][track]-1
                // << "\n Meas Value: " << measurementsList[hypothesisCombinations[hypo][track]-1]
                << "\n gij: " << gijList[hypo][track] 
                << std::endl;
                }                
                // Calc. each hypo. likelihood with each tracks gij                
                hypLikelihood  *= PD * gijList[hypo][track] * pow(BETA,(track-hypo));
            }      
            std::cout << "hypLikelihood.: " << hypLikelihood << std::endl;
            hypothesisLikelihoods.emplace_back(hypo,hypLikelihood);
        }

        // Calculate normalized hypLikelihoods

        // Sum of all hypothesis
        float totalLikelihood = 0;
        for(auto hyp : hypothesisLikelihoods)
        {
            totalLikelihood += hyp.second;
        }
        // Normalized hyp. likelihoods
        for(auto hyp : hypothesisLikelihoods)
        {
            normHyptLikelihoods.emplace_back(hyp.first,hyp.second/totalLikelihood);
            std::cout << "Hyp No: "<< hyp.first << " Norm likelihood: " << hyp.second/totalLikelihood << std::endl;
        }

        // Sort by highest likelihood
        for(auto hyp : normHyptLikelihoods)
        {
            std::sort(normHyptLikelihoods.begin(),normHyptLikelihoods.end(), sortbysec);
            // std::cout << "Hyp No: " hyp.first << "Track: " << hypothesisCombinations[hyp.first]
        }

        std::cout << "Highest score Hypt. No: " << normHyptLikelihoods.back().first << " Norm. Likelihood: " << normHyptLikelihoods.back().second <<std::endl;

        // std::cout << "Track Size: " << hypothesisCombinations[normHyptLikelihoods.back().first].size() <<std::endl;

        // Update KF with highest likelihood measurement
        for (int track = 0; track < hypothesisCombinations[normHyptLikelihoods.back().first].size(); track++)
        {
            int hypNo = normHyptLikelihoods.back().first;
            int selectedMeas = hypothesisCombinations[hypNo][track]-1;

            std::cout << "hypNo: " << hypNo << " Track: " << track << " selectedMeas: " << selectedMeas << std::endl;
            std::cout << "selected meas: "<< measurementsList[selectedMeas];
            tracksList[track].kfUpdate(measurementsList[selectedMeas]);
        }


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
