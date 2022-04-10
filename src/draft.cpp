#include <stdio.h>

int main()
{
    // Draft()
    // {
            // Eigen::VectorXd observations;

            // Eigen::VectorXd tracks;
            // Eigen::VectorXd predicted_points;

            // bool initialized = false;

            // float beta = 0.03 (can be anything <=0.1)
            // float PD = 0.75; 
            // int M = 2; // 2 Dimensional
            // float Vc = ? ;  Field of view, 0.20 ...250 m far range, 0.20...70m/100m@0…±45° near range and 0.20…20m@±60° near range

    // }

    // 1- Subscribe to Observations
    // void radarCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) { >> observations }

    // Find the number of observations (No) (msg->data[col+20] != 0)
    // float No = 0;

    // if (!initialized && No > 0) // may receive zero observations at the beginning
        // for (size_t i = 0; i < No; i++){
        //  add all observations to >> tracks[]
        // }
        // initialized = true;
        // return;
    
    // Predict next state@(k+1) with Kalman Filter for each track in tracks@(k)
        // for (size_t i = 0; i < tracks.size(); i++)
        // {
        //    predicted_points.append( tracks[i] => KF(x,y,vx,vy) )
        // }

        // Gating can be done here... According to the process covariance? JPDA does not have gating, accepts as if tracks exits anyway?

        // for (size_t i = 0; i < predicted_points.size(); i++)
        // {
        //     for (size_t j = 0; j < observations.size(); j++)
        //     {
        //         //Create hypothesis matrix p.354
                    
        //     }
            
        // }

        // Normalize probabilities P(H)




        

    // 2- Assignment JPDA

        // for each track (in proven) look for all the observations

        // S = H P Ht + R
        // d^2 = y' S^-1 y => Mahalanobis distance (normalized statistical distance)
      
        // measurement matrix Eigen::MatrixXd H_; kalman_filter.h
        // // state covariance matrix Eigen::MatrixXd P_; kalman_filter.h

        // Eigen::VectorXd S = Eigen::VectorXd(4); // H_ * P_ * H_.transpose()

        // gij = e^(-dij^2/2) / (2Pi)^(M/2) sqrt(|Sij|)

        // Hypothesis Likelihood



         

    // 3- Track Maintenance

    // Eigen::VectorXd proven_tracks 
        // if track_value > threshold         
            // add to proven_tracks and remove from possible_tracks

    // 4- Kalman Filter

    // 5- Gating
 

        // for (size_t i = 0; i < tentative_tracks.size(); i++)
        // {
        //     tentative_tracks[i] => KF(x,y,vx,vy)
        //     
        // }
    

    return 0;
}