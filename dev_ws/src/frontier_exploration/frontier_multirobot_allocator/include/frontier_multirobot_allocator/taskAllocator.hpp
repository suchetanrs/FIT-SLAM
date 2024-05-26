#ifndef TASK_ALLOCATOR_HPP
#define TASK_ALLOCATOR_HPP

#include <vector>
#include <string>

class TaskAllocator {
private:
    std::vector<std::vector<double>> costMatrix;
    std::vector<std::string> robotNames;
    std::vector<std::vector<double>> tasksAllocated;

public:
    TaskAllocator();

    void addRobotTasks(std::vector<double> tasks, std::string robotName);
    void solveAllocationHungarian();
    std::vector<std::vector<double>> getAllocatedTasks();
};

#endif // TASK_ALLOCATOR_HPP