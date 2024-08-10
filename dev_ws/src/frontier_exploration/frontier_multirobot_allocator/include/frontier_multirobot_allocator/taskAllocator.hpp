#ifndef TASK_ALLOCATOR_HPP
#define TASK_ALLOCATOR_HPP

#include <vector>
#include <string>
#include <map>
#include "frontier_multirobot_allocator/hungarian/Hungarian.h"
#include "frontier_multirobot_allocator/minPos/minPos.hpp"

class TaskAllocator {
private:
    std::vector<std::vector<double>> costMatrix;
    std::vector<std::vector<double>> distanceMatrix;
    std::vector<std::string> robotNames;
    std::map<std::string, int> tasksAllocated;

public:
    TaskAllocator();

    void addRobotTasks(std::vector<double> costs, std::vector<double> distances, std::string robotName);
    void solveAllocationHungarian();
    void solveAllocationMinPos();
    std::map<std::string, int> getAllocatedTasks();
    void reset()
    {
        costMatrix.clear();
        distanceMatrix.clear();
        robotNames.clear();
        tasksAllocated.clear();
    };
};

#endif // TASK_ALLOCATOR_HPP