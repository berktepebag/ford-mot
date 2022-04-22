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

        for(int col=0;col<10;col++)
        {           
            if(msg->data[col+20] == 0) continue;
            
            // [x, y, vx, vy]
            radarMeas << 
            msg->data[col+30],
            msg->data[col+40],
            msg->data[col+50],
            msg->data[col+60];

            measurementsList.push_back(radarMeas);
        }

        // If there is no track in tracksList
        if(tracksList.size() == 0){
            // Add teach observation as "tentavi tracks" to tracksList
            for(auto measurement : measurementsList)
            {
            tracksList.emplace_back(Track(TRACK_ID,measurement,GLOBAL_COUNTER));            
            }
        }
       
        // Calculate gij(s)
        std::vector<std::vector<float>> gijList; 
        for(auto track : tracksList)
        {
            gijList.push_back(track.calculateGij(measurementsList));
        }

        std::vector<std::vector<int>> hypothesisCombinations;
        hypothesisCombinations = utilities.hypothesisCombinations(gijList);

        // utilities.printHypothesis(gijList);
 
        if(hypothesisCombinations.size()!=0)
        {
            // Hyp No. and it's calculated likelihood
            std::vector<std::pair<int,float>> hypothesisLikelihoods;
            std::vector<std::pair<int,float>> normHyptLikelihoods;

            //Hypothesis #
            for (int hypo = 0; hypo < hypothesisCombinations.size(); hypo++)
            {
                float hypLikelihood = 1;

                // For each Track
                for (int track = 0; track < hypothesisCombinations[hypo].size(); track++)
                {                    
                    // Calc. each hypo. likelihood with each tracks gij                
                    hypLikelihood  *= PD * gijList[track][hypo] * pow(BETA,(measurementsList.size()-tracksList.size()));
                }      
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
            }
            // Sort by highest likelihood
            for(auto hyp : normHyptLikelihoods)
            {
                std::sort(normHyptLikelihoods.begin(),normHyptLikelihoods.end(), sortbysec);
            }
       
            // Update KF with highest likelihood measurement
            for (int track = 0; track < hypothesisCombinations[normHyptLikelihoods.back().first].size(); track++)
            {
                int hypNo = normHyptLikelihoods.back().first;
                int selectedMeas = hypothesisCombinations[hypNo][track];

                tracksList[track].increaseCurrentCounter();

                // Update step of KF with selected measurement
                tracksList[track].kfUpdate(measurementsList[selectedMeas]);
                
                if(tracksList[track].trackMaintenance(GLOBAL_COUNTER))
                {
                    tracksList.erase(tracksList.begin()+track);
                }

                // Remove selected observation
                measurementsList.erase(measurementsList.begin()+selectedMeas);

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
