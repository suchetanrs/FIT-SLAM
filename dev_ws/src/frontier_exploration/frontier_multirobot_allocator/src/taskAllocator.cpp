#include "frontier_multirobot_allocator/taskAllocator.hpp"
#include <iostream>
#include <limits>

TaskAllocator::TaskAllocator() {}

void TaskAllocator::addRobotTasks(std::vector<double> costs, std::vector<double> distances, std::string robotName) {
    costMatrix.push_back(costs);
    distanceMatrix.push_back(distances);
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
    for (unsigned int i = 0; i < costMatrix.size(); ++i) {
        int robotIndex = assignment[i];
        std::cout << "Task " << robotIndex << " allocated to Robot " << robotNames[i] << std::endl;
        tasksAllocated[robotNames[i]] = robotIndex;
    }
    std::cout << "Total cost: " << cost << std::endl;
    if (cost == std::numeric_limits<double>::max())
    {
        // throw std::runtime_error("Infinite cost");
    }
}

void TaskAllocator::solveAllocationMinPos() {
    // Create an instance of MinPosAlgo
    MinPosAlgo minPosCalc(distanceMatrix, costMatrix);
    std::vector<int> assignment;
    std::cout << "Cost matrix size: " << costMatrix.size() << std::endl;
    std::cout << "robotNames size: " << robotNames.size() << std::endl;
    for (auto elem : costMatrix)
    {
        std::cout << "Element size: " << elem.size() << std::endl;
    }
    double cost = minPosCalc.getAssignmentMinPos(assignment);
    std::cout << "Assignment size: " << assignment.size() << std::endl;
    std::cout << "Optimal Assignment:" << std::endl;
    for (unsigned int i = 0; i < costMatrix.size(); ++i) {
        int robotIndex = assignment[i];
        std::cout << "Task " << robotIndex << " allocated to Robot " << robotNames[i] << std::endl;
        tasksAllocated[robotNames[i]] = robotIndex;
    }
    std::cout << "Total cost: " << cost << std::endl;
    if (cost == std::numeric_limits<double>::max())
    {
        // throw std::runtime_error("Infinite cost");
    }
}

std::map<std::string, int> TaskAllocator::getAllocatedTasks() {
    return tasksAllocated;
}