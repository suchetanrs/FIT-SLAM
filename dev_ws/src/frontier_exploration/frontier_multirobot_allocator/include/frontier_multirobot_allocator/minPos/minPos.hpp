#ifndef MINPOSALGO_HPP
#define MINPOSALGO_HPP

#include <iostream>
#include <vector>
#include <limits>
#include "frontier_multirobot_allocator/hungarian/Hungarian.h"

class MinPosAlgo {
public:
    MinPosAlgo(std::vector<std::vector<double>>& distanceMat, std::vector<std::vector<double>>& costMat);

    void calculateDistanceCounts();
    void makeModifiedCostMatrix();
    void prinDistanceMatrix() const;
    void printDistanceCounts() const;
    void prinCostMatrix() const;
    void printModifiedCostMatrix() const;
    double getAssignmentMinPos(std::vector<int>& assignment);

private:
    std::vector<std::vector<double>> distanceMatrix;
    std::vector<std::vector<double>> costMatrix;
    std::vector<std::vector<double>> modifiedCostMatrix;
    std::vector<std::vector<int>> P;
    int numRobots;
    int numFrontiers;
};

#endif // MINPOSALGO_HPP