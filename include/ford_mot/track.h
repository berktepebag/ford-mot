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

        int createdAtGlobalCounter = 0;
        int currentCounter = 0;

        int tentativeToComfirmedThreshold = 4;
        int tentativeToComfirmedWindow = 6;

        int toBeDeletedThreshold = 3;
        int toBeDeletedWindow = 5;

    public:
        Track();
        Track(int &TRACK_ID_, Eigen::VectorXd x_, const int GLOBAL_COUNTER_);
        ~Track();

        void tentativeToComfirmed();

        void pubPathMessage();

        void increaseCurrentCounter();

        int setID(int ID_);
        int getID();

        bool trackComfirmed();

        bool trackMaintenance(const int GLOBAL_COUNTER_);

        std::vector<float> calculateGij(std::vector<Eigen::VectorXd> measurementsList);
        
        void kfUpdate(Eigen::VectorXd chosenMeasurement);

};

#endif