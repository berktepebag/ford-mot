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

std::vector<std::vector<int>> Utilities::hypothesisCombinations(std::vector<std::vector<float>> gijList) 
{
    // std::vector<std::vector<float>> gijList {{1,2},{1,2}};//,{6,7,8}};

    int nTracks = gijList.size();
    int nObservations = gijList[0].size();

    std::vector<float> listObs;
    for(int i=1; i<nObservations+1; i++)
    {
        // std::cout << "i: " << i << "\n";
        listObs.push_back(i);
    }

    std::cout << "*--* \nnTracks: " << nTracks << " nObservations: " << nObservations << "\n"; // 1,2

    std::vector<std::unordered_set<int>> combinationsList;
    std::vector<int> observationSet;

    for(int i=1; i<nObservations+1; i++)
    {
        combinationsList.push_back(std::unordered_set<int> {i});
    }

    for (size_t track = 1; track < nTracks; track++)
    {        
        combinationsList = hypothesisMerge(combinationsList, listObs);
    }

    std::vector<std::vector<int>> retVec;

    // // Return the resulting Hypothesis List
    for (int comb = 0; comb < combinationsList.size(); comb++)
    {
        if(combinationsList[comb].size() == gijList.size())
        {            
            std::vector<int> trackVec;
            for(auto x : combinationsList[comb])
            {
                // std::cout << x << ", ";
                trackVec.push_back(x);
            }
            retVec.push_back(trackVec);
        }        
    }
    
    return retVec;    
}

std::vector<std::unordered_set<int>> Utilities::hypothesisMerge(std::vector<std::unordered_set<int>> combinations, std::vector<float> trackObservationList)
{
    std::vector<std::unordered_set<int>> retVec;
    for(auto combination : combinations)
    {        
        for (auto obs : trackObservationList)
        {
            std::unordered_set<int> temp = combination;
            temp.insert(obs);
            retVec.push_back(temp);            
        }        
    }
   
    return retVec;
}


