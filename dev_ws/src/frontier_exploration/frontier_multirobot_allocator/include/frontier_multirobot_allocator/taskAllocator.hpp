#ifndef TASK_ALLOCATOR_HPP
#define TASK_ALLOCATOR_HPP

#include <vector>
#include <string>
#include <map>

class TaskAllocator {
private:
    std::vector<std::vector<double>> costMatrix;
    std::vector<std::string> robotNames;
    std::map<std::string, int> tasksAllocated;

public:
    TaskAllocator();

    void addRobotTasks(std::vector<double> tasks, std::string robotName);
    void solveAllocationHungarian();
    std::map<std::string, int> getAllocatedTasks();
};

#endif // TASK_ALLOCATOR_HPP