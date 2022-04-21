#include "ford_mot/track.h"

Track::Track(){}

Track::Track(int &TRACK_ID_, Eigen::VectorXd x_, int GLOBAL_COUNTER_)
{
    ROS_INFO("\n****************\n Track ID: %d established as Tentavi Loc-> \n x: %f y: %f vx: %f vy: %f @Counter: %d \n****************", TRACK_ID_, x_(0), x_(1), x_(2), x_(3), GLOBAL_COUNTER_ );       
    createNewTrack(TRACK_ID_, GLOBAL_COUNTER_);
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

void Track::createNewTrack(int &TRACK_ID, int GLOBAL_COUNTER_)
{
    setID(TRACK_ID);
    TRACK_ID++;

    createdAtGlobalCounter = GLOBAL_COUNTER_;
    currentCounter = createdAtGlobalCounter ;
}

int Track::setID(int ID_){ ID = ID_;}
int Track::getID(){return ID;}

bool Track::trackComfirmed(){return comfirmed;}

bool Track::trackMaintenance(const int GLOBAL_COUNTER_)
{
    std::cout << "\n Track ID: " << ID << " C@: " << createdAtGlobalCounter << "/" << GLOBAL_COUNTER_ << " Consecutive: " << (GLOBAL_COUNTER_ - currentCounter) << " Sts: " << comfirmed << std::endl;
    // If tentative
    if(trackComfirmed() == false)
    {                
        // If sufficient time passed to check tentative
        if(GLOBAL_COUNTER_ - createdAtGlobalCounter > tentativeToComfirmedWindow)
        {             
            // std::cout << "\n @@@@@ Track ID: " << ID << " Diff: " <<GLOBAL_COUNTER_ - createdAtGlobalCounter << std::endl;

            if(GLOBAL_COUNTER_ - currentCounter < tentativeToComfirmedThreshold)
            {
                // std::cout << "\n@@@@@ Comfirming Track " << ID << std::endl;
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

// Assign Observation point to the track points list

std::vector<float> Track::calculateGij(std::vector<Eigen::VectorXd> measurementsList)
{
    // std::cout << "calculate Gij \n";
    return trackKF.GijCalculation(measurementsList);
}

void Track::kfUpdate(Eigen::VectorXd chosenMeasurement)
{
    trackKF.update(chosenMeasurement);
}

void Track::addTrackPointsToList(float xPoint, float yPoint)
{
    trackPointsList.push_back(std::pair<float,float>(xPoint, yPoint) );
}

void Track::addPredictedPointsToList(float xPoint, float yPoint)
{
    trackPointsList.push_back(std::pair<float,float>(xPoint, yPoint) );
}
