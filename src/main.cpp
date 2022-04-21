#include "ford_mot/main.h"

#include <stdio.h>
#include <iostream>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

#include "eigen3/Eigen/Dense"
#include "ford_mot/measurement_package.h"
#include "ford_mot/tracking.h"
#include "ford_mot/utilities.h"
#include "ford_mot/track.h"

#include <vector>
#include <map>

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
        bool log = false;

        // Predicted Points will be created when a track exits (x_, P_)
        std::vector<std::pair<Eigen::VectorXd, Eigen::MatrixXd>> predictedPoints;
        // Tentative tracks will exists until either deleted or converted into comfirmedTrack
        std::vector<Track> tracksList;

        // Global Counter, counts steps since tracker initialized
        int GLOBAL_COUNTER = 0;
        // Track ID's for track maintenance
        int TRACK_ID = 0;
        
        Utilities utilities;
        Track track;

        const float PD = 0.75;
        const float BETA = 0.03;
 

    public:
        SensorClass()
        {
            radarSubs = nh.subscribe("lrrObjects",1, &SensorClass::radarCallback, this);
        
            currentTime = ros::Time::now();
            prevTime = currentTime;
        }

        static bool sortbysec(const std::pair<int,int> &a, const std::pair<int,int> &b)
        {return (a.second < b.second);}

        ~SensorClass(){}

    void radarCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        std::vector<Eigen::VectorXd> measurementsList;
        GLOBAL_COUNTER++;

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

            // [x, y, vx, vy]
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
            tracksList.emplace_back(Track(TRACK_ID,measurement,GLOBAL_COUNTER));            
            }
        }


        std::cout << "Measurement size before gij calc.: " << measurementsList.size() <<"\n";             
        std::cout << "Track size before gij calc.: " << tracksList.size() <<"\n";             

        // Calculate gij(s)
        std::vector<std::vector<float>> gijList; 
        for(auto track : tracksList)
        {
            std::cout << "----Track ID: " << track.getID() << " Status: " << std::boolalpha << track.trackComfirmed() << "\nCalculating gij...\n";
            // std::cout << "Meas. List size: " << measurementsList.size() <<"\n";             

            gijList.push_back(track.calculateGij(measurementsList));
        }
        std::cout << "gijList size after gij calc (# of tracks): " << gijList.size() <<"\n";             

        std::vector<std::vector<int>> hypothesisCombinations;
        hypothesisCombinations = utilities.hypothesisCombinations(gijList);

        std::cout << "hypothesisCombinations size: " << hypothesisCombinations.size() <<"\n";     
        utilities.printHypothesis(gijList);
 
        if(log) std::cout << "log 4 \n";

        if(hypothesisCombinations.size()!=0)
        {
            // Hyp No. and it's calculated likelihood
            std::vector<std::pair<int,float>> hypothesisLikelihoods;
            std::vector<std::pair<int,float>> normHyptLikelihoods;
            if(log) std::cout << "log 4.1 \n";

            //Hypothesis #
            for (int hypo = 0; hypo < hypothesisCombinations.size(); hypo++)
            {
                if(log) std::cout << "log 4.2 \n";

                float hypLikelihood = 1;

                std::cout << "hypothesisCombinations[hypo] size (#track): " << hypothesisCombinations[hypo].size() <<"\n";    
                // For each Track
                for (int track = 0; track < hypothesisCombinations[hypo].size(); track++)
                {
                    if(log) std::cout << "log 4.2.1 \n";
                    if(true){
                    std::cout 
                    << "# Hypot.: " << hypo
                    << "\n# Track: " << track 
                    << "\n Selected Meas.: " << hypothesisCombinations[hypo][track]
                    // << "\n Meas Value: " << measurementsList[hypothesisCombinations[hypo][track]-1]
                    << "\n gij: " << gijList[track][hypo] 
                    << std::endl;
                    }                
                    if(log) std::cout << "log 4.2.2 \n";
                    // Calc. each hypo. likelihood with each tracks gij                
                    hypLikelihood  *= PD * gijList[track][hypo] * pow(BETA,(track-hypo));
                    if(log) std::cout << "log 4.2.3 \n";

                }      
                if(log) std::cout << "log 4.3 \n";

                hypothesisLikelihoods.emplace_back(hypo,hypLikelihood);
            }
            if(log) std::cout << "log 5 \n";

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
            if(log) std::cout << "log 6 \n";

            // Sort by highest likelihood
            for(auto hyp : normHyptLikelihoods)
            {
                std::sort(normHyptLikelihoods.begin(),normHyptLikelihoods.end(), sortbysec);
                // std::cout << "Hyp No: " hyp.first << "Track: " << hypothesisCombinations[hyp.first]
            }
            if(log) std::cout << "log 7 \n";

            std::cout << "Highest score Hypt. No: " << normHyptLikelihoods.back().first << " Norm. Likelihood: " << normHyptLikelihoods.back().second <<std::endl;

            // std::cout << "Track Size: " << hypothesisCombinations[normHyptLikelihoods.back().first].size() <<std::endl;
            if(log) std::cout << "log 8 \n";

            // Update KF with highest likelihood measurement
            for (int track = 0; track < hypothesisCombinations[normHyptLikelihoods.back().first].size(); track++)
            {
                int hypNo = normHyptLikelihoods.back().first;
                int selectedMeas = hypothesisCombinations[hypNo][track];

                tracksList[track].increaseCurrentCounter();

                // std::cout << "hypNo: " << hypNo << " Track: " << track << " selectedMeas: " << selectedMeas << std::endl;
                // std::cout << "selected meas: "<< measurementsList[selectedMeas];
                tracksList[track].kfUpdate(measurementsList[selectedMeas]);
                
                if(tracksList[track].trackMaintenance(GLOBAL_COUNTER))
                {
                    tracksList.erase(tracksList.begin()+track);
                    ROS_INFO("Track No: %d has been deleted!", tracksList.begin()+track);
                }

                // Remove selected observation
                measurementsList.erase(measurementsList.begin()+selectedMeas);
                std::cout << "Meas No: " << selectedMeas << " has been removed... "<< measurementsList.size() << " Meas. left." << std::endl;

                tracksList[track].pubPathMessage();
            }           
            // Add remaining observations as "tentavi tracks" to tracksList
            for(auto measurement : measurementsList)
            {
            tracksList.emplace_back(Track(TRACK_ID,measurement,GLOBAL_COUNTER));            
            }
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
