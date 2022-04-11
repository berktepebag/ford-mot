#include "ford_mot/utilities.h"
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <eigen3/Eigen/Cholesky>

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

    int n_cols = 2; // x,y
    int n_rows = measurements.size();

    Eigen::MatrixXd mah_mtx = Eigen::MatrixXd(n_rows,n_cols);

    float pred_x, pred_y;
    // Eigen::VectorXd predictedP

    // Convert Vector to Eigen Matrix for calculations
    for (size_t i = 0; i < measurements.size(); i++)
    {
        for (size_t j = 0; j < 2; j++)
        {
            mah_mtx(i,j) = measurements[i][j];            
        }     
    }
    Eigen::RowVectorXd centroid = mah_mtx.colwise().sum()/n_rows;   
    
    // Normalized predicted point
    float norm_x = 0;
    float norm_y = 0;

    for (size_t i = 0; i < predictedPoints.size(); i++)
    {
        norm_x = predictedPoints[i].first[0] - centroid[0];
        norm_y = predictedPoints[i].first[1] - centroid[1];

        Eigen::Vector y = Eigen::VectorXd(2);
        

    }

    // std::cout << "Centroid: "<<  centroid << "\n";
    // std::cout << "x: " << norm_x << " y: " << norm_y << "\n";





    // Maho Distance
    int d2 = 0;

    return d2;
}