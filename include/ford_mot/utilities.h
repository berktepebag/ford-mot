#ifndef UTILITIES_H_
#define UTILITIES_H_

#include "eigen3/Eigen/Dense"
#include <vector>

class Utilities
{
    public:

        Utilities();
        virtual ~Utilities();

        float CalculateMahalanobisDistance(std::vector<Eigen::VectorXd> measurements, std::vector<std::pair<Eigen::VectorXd, Eigen::MatrixXd>> predictedPoints);

    private:

        Eigen::MatrixXd mah_measurements;
        Eigen::VectorXd centroid; // mean of measurements



};




#endif