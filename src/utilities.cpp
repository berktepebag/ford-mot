#include "ford_mot/utilities.h"
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <eigen3/Eigen/Cholesky>
#include <vector>
#include <math.h>

Utilities::Utilities()
{   
    // std::cout << "Utilities Created";
}

Utilities::~Utilities()
{   
    // std::cout << "Utilities Deleted";
}


float Utilities::calculateMahalanobisDistance(std::vector<Eigen::VectorXd> measurements, std::vector<std::pair<Eigen::VectorXd, Eigen::MatrixXd>> predictedPoints)
{
    // MD = sqrt(Y_T * S_Inverse * Y)

    int n_cols = 2; // x,y
    int n_rows = measurements.size();

    Eigen::MatrixXd x = Eigen::MatrixXd(n_rows,n_cols);

    float pred_x, pred_y;
    // Eigen::VectorXd predictedP

    // Variance
    float sigma_x = 0;
    float sigma_y = 0;

    Eigen::RowVectorXd centroid = x.colwise().sum()/n_rows; 
    // std::cout << "centroid: " << centroid << std::endl;
    float cov_sum_x = 0;
    float cov_sum_y = 0;

    // Convert Vector to Eigen Matrix for calculations & Calc sigma2s'
    for (size_t track = 0; track < measurements.size(); track++)
    {
        for (size_t obs = 0; obs < 2; obs++)
        {
            x(track,obs) = measurements[track][obs];                        
        }     

        // Std Dev
        cov_sum_x += pow((measurements[track][0] - centroid[0]), 2);
        cov_sum_y += pow((measurements[track][1] - centroid[1]), 2);
    }
    // std::cout << "cov sum x: " << cov_sum_x << std::endl;

    sigma_x = sqrt(cov_sum_x / (measurements.size() - 1));
    sigma_y = sqrt(cov_sum_y / (measurements.size() - 1));
    // std::cout << "sigma x: " << sigma_x << std::endl;
    // std::cout << "sigma y: " << sigma_y << std::endl;

    Eigen::MatrixXd S = Eigen::MatrixXd(2,2);
    S << pow(sigma_x,2) , 0,
        0, pow(sigma_y,2);

    Eigen::MatrixXd S_Inverse = S.inverse();
   
    // Normalized predicted points
    float norm_x = 0;
    float norm_y = 0;

    // Mahalonbis Distance & vector
    float mahanobis_dist = 0; //dij^2
    std::vector<float> v_dij2;

    Eigen::VectorXd Y = Eigen::VectorXd(2);
    for (size_t track = 0; track < predictedPoints.size(); track++)
    {
        norm_x = predictedPoints[track].first[0] - centroid[0];
        norm_y = predictedPoints[track].first[1] - centroid[1];
        Y << norm_x, norm_y;
        // std::cout << "Y_t (" << Y.transpose().rows() << ", " << Y.transpose().cols() << ")" << std::endl;
        // std::cout << "S_inv (" << S_Inverse.rows() << ", " << S_Inverse.cols() << ")" << std::endl;
        // std::cout << "Y (" << Y.rows() << ", " << Y.cols() << ")"<< std::endl;
        mahanobis_dist = sqrt(Y.transpose() * S_Inverse * Y);
        // std::cout << "Point "<< track << " mahanobis dist: " << mahanobis_dist << std::endl;
    }

    
    

    return mahanobis_dist;
}


void Utilities::findHypothesis(std::vector<std::vector<float>> gijList)
{
    // Look for each track
    for (size_t track = 0; track < gijList.size(); track++)
    {
        // With each measurement
        for (size_t obs = 0; obs < gijList[track].size(); obs++)
        {
            std::cout << "gijList[" << track << "," << obs << "]" << gijList[track][obs] << std::endl;
        }        
    }   


}

std::vector<std::set<int>> Utilities::combinations(std::vector<std::vector<float>> gijList) 
{
    // gij list'in size() i track adedini
    // track size()'i observation adedini

    // Testing
    // std::vector<std::vector<float>> gijList {{0,1,2},{0,1,2,3,4,5},{0,3,4,5}};
    // gijList = gijList;
    // Testing

    std::vector<std::set<int>> combinationsList;
    std::set<int> observationSet;

    // observationSet.insert(0);
    // From first track, we are creating a vector of sets with each observation
    for (int obs = 0; obs < gijList[0].size(); obs++)
    {
        observationSet.insert(obs); // Since 0 observation is not included we should increase obs by one and zero observation as 
        combinationsList.push_back(observationSet);
    }  

    // Add rest of the tracks' observations to the combinationsList recursively
    for (size_t track = 1; track < gijList.size(); track++)
    {
        combinationsList = merge(combinationsList, gijList[track]);
    }

    // Return the resulting Hypothesis List
    for (size_t comb = 0; comb < combinationsList.size(); comb++)
    {
        if(gijList[0].size() == combinationsList[comb].size())
        {
            for(auto x : combinationsList[comb])
            {

                std::cout << x << ", ";
            }
        }
        std::cout << "\n *****" << std::endl;
    }   
}

std::vector<std::set<int>> Utilities::merge(std::vector<std::set<int>> combinations, std::vector<float> trackObservationList)
{
    std::vector<std::set<int>> retVec;
    for(auto combination : combinations)
    {
        for (size_t obs = 0; obs < trackObservationList.size(); obs++)
        {
            std::set<int> temp = combination;
            temp.insert(obs);
            retVec.push_back(temp);            
        }        
    }

    return retVec;
}
