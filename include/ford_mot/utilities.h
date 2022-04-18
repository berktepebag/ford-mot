#ifndef UTILITIES_H_
#define UTILITIES_H_

#include "eigen3/Eigen/Dense"
#include <vector>
#include <set>
#include <bits/stdc++.h>

class Utilities
{
    public:

        Utilities();
        virtual ~Utilities();

        float calculateMahalanobisDistance(std::vector<Eigen::VectorXd> measurements, std::vector<std::pair<Eigen::VectorXd, Eigen::MatrixXd>> predictedPoints);

        void findHypothesis(std::vector<std::vector<float>> gij);

        std::vector<std::set<int>> combinations(std::vector<std::vector<float>> gijList);
        std::vector<std::unordered_set<int>> merge(std::vector<std::unordered_set<int>> combinations, std::vector<float> trackObservationList);


    private:





};




#endif