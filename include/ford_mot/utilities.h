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

        void printHypothesis(std::vector<std::vector<float>> gij);

        std::vector<std::vector<int>> hypothesisCombinations(std::vector<std::vector<float>> gijList);
        std::vector<std::unordered_set<int>> hypothesisMerge(std::vector<std::unordered_set<int>> combinations, std::vector<float> trackObservationList);

        // bool sortbysec(const std::pair<int,int> &a,const std::pair<int,int> &b);

        bool log = false;


    private:

};

#endif