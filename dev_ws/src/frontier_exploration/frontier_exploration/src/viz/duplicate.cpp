#include <vector>
#include <algorithm>
#include <iostream>

int main() {
    std::vector<std::vector<double>> every_frontier_list = {
        {1.0, 2.0, 3.0},
        {1.0, 2.0, 3.0},
        {2.0, 3.0, 4.0},
        {2.0, 3.0, 4.0},
        {3.0, 4.0, 5.0},
        {3.0, 4.0, 5.0}
    };

    // Use std::unique to remove duplicates in a sorted vector
    auto it = std::unique(every_frontier_list.begin(), every_frontier_list.end());

    // Resize the vector to remove the duplicates
    every_frontier_list.resize(std::distance(every_frontier_list.begin(), it));

    // every_frontier_list now contains the unique elements
    for(const auto& list : every_frontier_list) {
        for(double num : list) {
            std::cout << num << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}