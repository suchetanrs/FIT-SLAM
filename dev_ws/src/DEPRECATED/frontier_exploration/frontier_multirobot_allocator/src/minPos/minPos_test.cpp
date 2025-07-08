#include <iostream>
#include <vector>
#include <limits>
#include "frontier_multirobot_allocator/minPos/minPos.hpp"

int main() {
    // Example distance matrix initialization
    std::vector<std::vector<double>> distances = {
        {0.2, 3.4, 2.2, 5.1},
        {0.2, 2.3, 1.7, 3.3},
        {0.2, 1.1, 4.5, 2.0},
        {0.2, 1.1, 4.5, 2.0}
    };

    std::vector<std::vector<double>> costMat = {
        {0.1, 3.9, 8.2, 10.1},
        {0.2, 2.5, 6.7, 1.34},
        {1.0, 0.1, 4.5, 9.0},
        {0.8, 9.1, 5.5, 21.0}
    };

    // Create an instance of MinPosAlgo
    MinPosAlgo minPosCalc(distances, costMat);

    std::vector<int> assignment;

    std::cout << "Cost: " << minPosCalc.getAssignmentMinPos(assignment) << std::endl;

    return 0;
}
