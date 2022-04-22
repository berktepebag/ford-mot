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

void Utilities::printHypothesis(std::vector<std::vector<float>> gijList)
{
    // Look for each track
    for (size_t track = 0; track < gijList.size(); track++)
    {
        // With each measurement
        for (size_t obs = 0; obs < gijList[track].size(); obs++)
        {
            std::cout << "[] [] [] T,O[" << track << ","<< obs <<"] -> " << "gij: " << gijList[track][obs] << std::endl;
        }        
    }   
}

std::vector<std::vector<int>> Utilities::hypothesisCombinations(std::vector<std::vector<float>> gijList) 
{
    // std::vector<std::vector<float>> gijList {{1,2,3},{4,2,3},{5,6,7}};
    int nTracks = gijList.size();
    int nObservations = gijList[0].size();

    std::vector<int> listObs;
    std::vector<std::unordered_set<int>> combinationsList;

    for(int i=0; i<nObservations; i++)
    {
        combinationsList.push_back(std::unordered_set<int> {i});
        listObs.push_back(i); 
    }

    for (size_t track = 1; track < nTracks; track++)
    {        
        combinationsList = hypothesisMerge(combinationsList, listObs);
    }

    std::vector<std::vector<int>> retVec;

    // // Return the resulting Hypothesis List as a vector
    for (int comb = 0; comb < combinationsList.size(); comb++)
    {
        // if(combinationsList[comb].size() == gijList.size())
        {            
            std::vector<int> trackVec;
            for(auto x : combinationsList[comb])
            {
                trackVec.push_back(x);
            }
            retVec.push_back(trackVec);
        }        
    }
    
    return retVec;    
}

std::vector<std::unordered_set<int>> Utilities::hypothesisMerge(std::vector<std::unordered_set<int>> combinations, std::vector<int> trackObservationList)
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


