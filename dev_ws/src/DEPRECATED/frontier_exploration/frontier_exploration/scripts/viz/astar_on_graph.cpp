#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>

// Define a Node structure
struct Node {
    int x, y; // Position on the grid
    float gCost, hCost; // Costs for A* algorithm
    float fCost() const { return gCost + hCost; } // fCost = gCost + hCost
    Node* parent; // Pointer to parent node

    Node(int x, int y) : x(x), y(y), gCost(INFINITY), hCost(INFINITY), parent(nullptr) {}

    // Compare nodes based on fCost for priority queue
    bool operator<(const Node& other) const {
        return fCost() > other.fCost(); // Inverted for min-heap
    }
};

// // Function to calculate heuristicManhattan (Manhattan Distance)
// float heuristicManhattan(const Node& a, const Node& b) {
//     return std::abs(a.x - b.x) + std::abs(a.y - b.y);
// }

// Function to calculate heuristicEuclidean (Euclidean Distance)
float heuristicEuclidean(const Node& a, const Node& b) {
    return std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2);
}

// Check if a given position is within the grid boundaries and traversable
bool isTraversable(int x, int y, const std::vector<std::vector<int>>& grid) {
    return x >= 0 && x < grid.size() && y >= 0 && y < grid[0].size() && grid[x][y] == 0;
}

// Reconstruct path from end to start node
std::vector<Node*> reconstructPath(Node* endNode) {
    std::vector<Node*> path;
    Node* current = endNode;
    while (current) {
        path.push_back(current);
        current = current->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

// A* Algorithm
std::vector<Node*> aStar(Node* startNode, Node* targetNode, const std::vector<std::vector<int>>& grid) {
    std::priority_queue<Node> openSet; // Priority queue for open set
    std::unordered_map<int, Node*> allNodes; // Store all nodes created (for memory management)

    startNode->gCost = 0;
    startNode->hCost = heuristicEuclidean(*startNode, *targetNode);
    openSet.push(*startNode);

    std::vector<std::vector<bool>> closedSet(grid.size(), std::vector<bool>(grid[0].size(), false)); // Closed set

    while (!openSet.empty()) {
        Node current = openSet.top(); // Node with the lowest fCost
        openSet.pop();

        if (current.x == targetNode->x && current.y == targetNode->y) {
            return reconstructPath(&current); // Path found
        }

        closedSet[current.x][current.y] = true;

        // // Generate neighbors (up, down, left, right)
        // std::vector<std::pair<int, int>> directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
        // Generate neighbors (up, down, left, right)
        std::vector<std::pair<int, int>> directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, -1}, {-1, 1}};
        for (const auto& dir : directions) {
            int newX = current.x + dir.first;
            int newY = current.y + dir.second;

            if (!isTraversable(newX, newY, grid) || closedSet[newX][newY]) continue;

            // Calculate tentative gCost
            float tentativeGCost = current.gCost + 1;

            Node* neighbor;
            int index = newX * grid[0].size() + newY; // Unique index for the node

            if (allNodes.find(index) == allNodes.end()) {
                neighbor = new Node(newX, newY);
                allNodes[index] = neighbor;
            } else {
                neighbor = allNodes[index];
            }

            if (tentativeGCost < neighbor->gCost) {
                neighbor->gCost = tentativeGCost;
                neighbor->hCost = heuristicEuclidean(*neighbor, *targetNode);
                neighbor->parent = allNodes[current.x * grid[0].size() + current.y];
                openSet.push(*neighbor);
            }
        }
    }

    // No path found
    return {};
}

int main() {
    // Example grid: 0 = traversable, 1 = obstacle
    std::vector<std::vector<int>> grid = {
        {0, 1, 0, 0, 0},
        {0, 1, 0, 1, 0},
        {0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0}
    };

    Node start(0, 0);
    Node target(4, 4);

    std::vector<Node*> path = aStar(&start, &target, grid);

    if (!path.empty()) {
        std::cout << "Path found:" << std::endl;
        for (auto node : path) {
            std::cout << "(" << node->x << ", " << node->y << ") ";
        }
    } else {
        std::cout << "No path found." << std::endl;
    }

    // // Free memory
    // for (auto& pair : allNodes) {
    //     delete pair.second;
    // }

    return 0;
}