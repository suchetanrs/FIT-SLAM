#include <iostream>
#include <vector>
#include <limits>
#include "frontier_multirobot_allocator/minPos/minPos.hpp"

MinPosAlgo::MinPosAlgo(std::vector<std::vector<double>>& distanceMat, std::vector<std::vector<double>>& costMat)
    : distanceMatrix(distanceMat), 
        costMatrix(costMat),
        numRobots(distanceMat.size()), 
        numFrontiers(distanceMat[0].size()) {
    // Initialize the count matrix P with zeros
    P = std::vector<std::vector<int>>(numRobots, std::vector<int>(numFrontiers, 0));
    modifiedCostMatrix = std::vector<std::vector<double>>(numRobots, std::vector<double>(numFrontiers, std::numeric_limits<double>::max()));
    if(costMat.size() != distanceMat.size() || costMat[0].size() != distanceMat[0].size())
    {
        throw std::runtime_error("Please check your costMat and distanceMat sizes");
    }
}

void MinPosAlgo::calculateDistanceCounts() {
    // Calculate the count matrix P
    for (int i = 0; i < numRobots; ++i) {
        for (int j = 0; j < numFrontiers; ++j) {
            int count = 0;
            for (int k = 0; k < numRobots; ++k) {
                if (k != i && distanceMatrix[k][j] < distanceMatrix[i][j]) {
                    ++count;
                }
            }
            P[i][j] = count;
        }
    }
}

void MinPosAlgo::makeModifiedCostMatrix() {
    for (int i = 0; i < numRobots; ++i) {
        for (int j = 0; j < numFrontiers; ++j) {
            if (P[i][j] == 0) {
                modifiedCostMatrix[i][j] = costMatrix[i][j];
            }
        }
    }

}

void MinPosAlgo::prinDistanceMatrix() const {
    std::cout << "Distance Matrix" << std::endl;
    for (int i = 0; i < numRobots; ++i) {
        for (int j = 0; j < numFrontiers; ++j) {
            std::cout << distanceMatrix[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

void MinPosAlgo::printDistanceCounts() const {
    std::cout << "Distance counts" << std::endl;
    for (int i = 0; i < numRobots; ++i) {
        for (int j = 0; j < numFrontiers; ++j) {
            std::cout << P[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

void MinPosAlgo::prinCostMatrix() const {
    std::cout << "Cost Matrix" << std::endl;
    for (int i = 0; i < numRobots; ++i) {
        for (int j = 0; j < numFrontiers; ++j) {
            std::cout << costMatrix[i][j] << " ";
        }
        std::cout << std::endl;
    }
}


void MinPosAlgo::printModifiedCostMatrix() const {
    // Print the modified cost matrix
    std::cout << "Modified Cost Matrix" << std::endl;
    for (int i = 0; i < numRobots; ++i) {
        for (int j = 0; j < numFrontiers; ++j) {
            std::cout << modifiedCostMatrix[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

double MinPosAlgo::getAssignmentMinPos(std::vector<int>& assignment) {
    calculateDistanceCounts();
    prinCostMatrix();
    prinDistanceMatrix();
    printDistanceCounts();
    makeModifiedCostMatrix();
    printModifiedCostMatrix();

    HungarianAlgorithm HungAlgo;
    return HungAlgo.Solve(modifiedCostMatrix, assignment);
}