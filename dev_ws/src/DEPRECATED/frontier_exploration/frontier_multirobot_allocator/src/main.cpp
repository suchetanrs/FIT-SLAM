#include <iostream>
#include <vector>
#include <string>
#include "frontier_multirobot_allocator/hungarian/Hungarian.h"
#include "frontier_multirobot_allocator/taskAllocator.hpp"

int main() {
    TaskAllocator taskAllocator;

    // Add tasks and their costs for each robot
    std::vector<double> tasks1 = {0, 15.1, 18, 20, 25};
    taskAllocator.addRobotTasks(tasks1, "Robot A");

    std::vector<double> tasks2 = {0, 20, 12.1, 14, 15};
    taskAllocator.addRobotTasks(tasks2, "Robot B");

    std::vector<double> tasks3 = {0, 23, 25.1, 27, 25};
    taskAllocator.addRobotTasks(tasks3, "Robot C");

    std::vector<double> tasks4 = {0, 18, 21.1, 23, 20};
    taskAllocator.addRobotTasks(tasks4, "Robot D");

    std::vector<double> tasks5 = {0, 18, 16.1, 19, 20};
    taskAllocator.addRobotTasks(tasks5, "Robot E");

    // Solve allocation problem
    taskAllocator.solveAllocationHungarian();

    // Get the allocated tasks
    std::vector<std::vector<double>> allocatedTasks = taskAllocator.getAllocatedTasks();
    std::cout << "\nAllocated Tasks:" << std::endl;
    for (const auto& task : allocatedTasks) {
        std::cout << "Task " << task[0] << " allocated to Robot " << task[1] << std::endl;
    }

    return 0;
}