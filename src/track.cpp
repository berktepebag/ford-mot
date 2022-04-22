#include "ford_mot/track.h"

Track::Track(){}

Track::Track(int &TRACK_ID_, Eigen::VectorXd x_, const int GLOBAL_COUNTER_)
{
    ROS_INFO("\n****************\n Track ID: %d established as Tentavi Loc-> \n x: %f y: %f vx: %f vy: %f @Counter: %d \n****************", TRACK_ID_, x_(0), x_(1), x_(2), x_(3), GLOBAL_COUNTER_ );       

    setID(TRACK_ID_);
    TRACK_ID_++;

    createdAtGlobalCounter = GLOBAL_COUNTER_;
    currentCounter = createdAtGlobalCounter ;

    trackKF.ekf_.x_ << x_(0), x_(1), x_(2), x_(3);
}

Track::~Track()
{
    // ROS_INFO("Track No: %d has been deleted!", ID);
}

void Track::tentativeToComfirmed()
{
    comfirmed = true;
}

void Track::pubPathMessage()
{
    if(comfirmed)
    {
        trackKF.PublishPathMessage();
    }
}

void Track::increaseCurrentCounter()
{
    ++currentCounter;
}

int Track::setID(int ID_){ ID = ID_;}
int Track::getID(){return ID;}

bool Track::trackComfirmed(){return comfirmed;}

// Track Maintenance changes tentatives to comfirmed and removes the untracked ones
bool Track::trackMaintenance(const int GLOBAL_COUNTER_)
{
    // If tentative
    if(trackComfirmed() == false)
    {                
        // If sufficient time passed to check tentative
        if(GLOBAL_COUNTER_ - createdAtGlobalCounter > tentativeToComfirmedWindow)
        {           
            // Comfirm if passed the threshold
            if(GLOBAL_COUNTER_ - currentCounter < tentativeToComfirmedThreshold)
            {
                comfirmed = true;
            }
        }
    }
    // If comfirmed & If sufficient time passed to be deleted
    if(GLOBAL_COUNTER_ - currentCounter > toBeDeletedWindow)
    {
        if(GLOBAL_COUNTER_ - currentCounter > toBeDeletedThreshold)
        {
            toBeDeleted = true;
        }
    }
    return toBeDeleted;
}

// Calls GijCalculation function from trackking that calculates gij values and return them in a vector to be calculated for likelihood
std::vector<float> Track::calculateGij(std::vector<Eigen::VectorXd> measurementsList)
{
    // std::cout << "calculate Gij \n";
    return trackKF.GijCalculation(measurementsList);
}

// Update step of Kalman Filter. Calls trackking class.
void Track::kfUpdate(Eigen::VectorXd chosenMeasurement)
{
    trackKF.update(chosenMeasurement);
}


