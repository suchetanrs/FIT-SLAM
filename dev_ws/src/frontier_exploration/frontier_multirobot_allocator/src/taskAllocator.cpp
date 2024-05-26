#include "frontier_multirobot_allocator/taskAllocator.hpp"
#include "frontier_multirobot_allocator/hungarian/Hungarian.h"
#include <iostream>

TaskAllocator::TaskAllocator() {}

void TaskAllocator::addRobotTasks(std::vector<double> tasks, std::string robotName) {
    costMatrix.push_back(tasks);
    robotNames.push_back(robotName);
}

void TaskAllocator::solveAllocationHungarian() {
    HungarianAlgorithm HungAlgo;
    std::vector<int> assignment;
    std::cout << "Cost matrix size: " << costMatrix.size() << std::endl;
    std::cout << "robotNames size: " << robotNames.size() << std::endl;
    for (auto elem : costMatrix)
    {
        std::cout << "Element size: " << elem.size() << std::endl;
    }
    double cost = HungAlgo.Solve(costMatrix, assignment);
    std::cout << "Assignment size: " << assignment.size() << std::endl;
    std::cout << "Optimal Assignment:" << std::endl;
    // for (unsigned int i = 0; i < costMatrix.size(); ++i) {
    //     int robotIndex = assignment[i];
    //     std::cout << "Task " << i << " allocated to Robot " << robotNames[robotIndex] << std::endl;
    //     // tasksAllocated.push_back({(double)i + 1, (double)robotIndex + 1});
    // }
    // std::cout << "Total cost: " << cost << std::endl;
}

std::vector<std::vector<double>> TaskAllocator::getAllocatedTasks() {
    return tasksAllocated;
}