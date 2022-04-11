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


float Utilities::CalculateMahalanobisDistance(std::vector<Eigen::VectorXd> measurements, std::vector<std::pair<Eigen::VectorXd, Eigen::MatrixXd>> predictedPoints)
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
    for (size_t i = 0; i < measurements.size(); i++)
    {
        for (size_t j = 0; j < 2; j++)
        {
            x(i,j) = measurements[i][j];                        
        }     

        // Std Dev
        cov_sum_x += pow((measurements[i][0] - centroid[0]), 2);
        cov_sum_y += pow((measurements[i][1] - centroid[1]), 2);
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
    for (size_t i = 0; i < predictedPoints.size(); i++)
    {
        norm_x = predictedPoints[i].first[0] - centroid[0];
        norm_y = predictedPoints[i].first[1] - centroid[1];
        Y << norm_x, norm_y;
        // std::cout << "Y_t (" << Y.transpose().rows() << ", " << Y.transpose().cols() << ")" << std::endl;
        // std::cout << "S_inv (" << S_Inverse.rows() << ", " << S_Inverse.cols() << ")" << std::endl;
        // std::cout << "Y (" << Y.rows() << ", " << Y.cols() << ")"<< std::endl;
        mahanobis_dist = sqrt(Y.transpose() * S_Inverse * Y);
        // std::cout << "Point "<< i << " mahanobis dist: " << mahanobis_dist << std::endl;
    }

    
    

    return mahanobis_dist;
}