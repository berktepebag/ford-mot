#ifndef TRACK_H_
#define TRACK_H_

#include "ford_mot/tracking.h"
#include "eigen3/Eigen/Dense"

class Track
{
    private:
        Tracking trackKF;

        bool comfirmed, toBeDeleted = false;
        int ID = 0;

        // Hold x and y points to be drawen in RVIZ        
        std::vector<std::pair<float, float>> trackPointsList;

        int createdAtGlobalCounter = 0;
        int currentCounter = 0;

        int tentativeToComfirmedThreshold = 2;//3;
        int tentativeToComfirmedWindow = 3;//5;

        int toBeDeletedThreshold = 3;
        int toBeDeletedWindow = 5;

    public:
        Track();
        Track(int &TRACK_ID_, Eigen::VectorXd x_, int GLOBAL_COUNTER_);
        ~Track();

        void tentativeToComfirmed();

        void pubPathMessage();

        void increaseCurrentCounter();

        void createNewTrack(int &TRACK_ID, int GLOBAL_COUNTER_);

        int setID(int ID_);
        int getID();

        bool trackComfirmed();

        bool trackMaintenance(const int GLOBAL_COUNTER_);

        // Assign Observation point to the track points list

        std::vector<float> calculateGij(std::vector<Eigen::VectorXd> measurementsList);
        void kfUpdate(Eigen::VectorXd chosenMeasurement);

        void addTrackPointsToList(float xPoint, float yPoint);

        void addPredictedPointsToList(float xPoint, float yPoint);

};

#endif