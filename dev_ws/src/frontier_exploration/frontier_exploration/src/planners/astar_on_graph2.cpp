#include <iostream>
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <cmath>
#include <functional>
#include "frontier_exploration/Frontier.hpp"

using namespace std;

// Define a Node structure
struct Node {
    Frontier frontier;
    double g; // Cost from the start node to this node
    double h; // Heuristic cost estimate to the goal
    double f; // Total cost (f = g + h)
    Node* parent; // Pointer to the parent node

    Node(int x, int y, double g, double h, Node* parent = nullptr) 
        : x(x), y(y), g(g), h(h), f(g + h), parent(parent) {}

    // Comparator for priority queue to order by f value
    bool operator>(const Node& other) const {
        return f > other.f;
    }
};

// Function to calculate heuristic (Euclidean distance in this case)
double heuristic(const Node& a, const Node& b) {
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

// Function to get the successors of a node (example for a grid)
vector<Node> getSuccessors(const Node& current, const Node& goal, const vector<vector<int>>& grid) {
    vector<Node> successors;
    int directions[4][2] = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}}; // 4 possible directions (up, down, left, right)

    for (auto& dir : directions) {
        int newX = current.x + dir[0];
        int newY = current.y + dir[1];
        if (newX >= 0 && newX < grid.size() && newY >= 0 && newY < grid[0].size() && grid[newX][newY] == 0) {
            double newG = current.g + 1; // Assuming uniform cost for each move
            double newH = heuristic(Node(newX, newY, 0, 0), goal);
            successors.emplace_back(newX, newY, newG, newH);
        }
    }
    return successors;
}

// A* Algorithm function
vector<Node> aStar(const Node& start, const Node& goal, const vector<vector<int>>& grid) {
    priority_queue<Node, vector<Node>, greater<Node>> openList; // Min-heap priority queue
    unordered_set<int> closedSet;
    unordered_map<int, Node*> allNodes; // To store all nodes and their costs

    openList.push(start);
    allNodes[start.x * grid[0].size() + start.y] = new Node(start);

    while (!openList.empty()) {
        Node current = openList.top();
        openList.pop();

        // If goal is reached
        if (current.x == goal.x && current.y == goal.y) {
            vector<Node> path;
            Node* node = allNodes[current.x * grid[0].size() + current.y];
            while (node != nullptr) {
                path.push_back(*node);
                node = node->parent;
            }
            reverse(path.begin(), path.end());
            return path;
        }

        closedSet.insert(current.x * grid[0].size() + current.y);

        // Generate successors
        vector<Node> successors = getSuccessors(current, goal, grid);

        for (auto& successor : successors) {
            int successorHash = successor.x * grid[0].size() + successor.y;

            if (closedSet.find(successorHash) != closedSet.end()) {
                continue;
            }

            if (allNodes.find(successorHash) == allNodes.end() || allNodes[successorHash]->g > successor.g) {
                successor.parent = allNodes[current.x * grid[0].size() + current.y];
                allNodes[successorHash] = new Node(successor);
                openList.push(successor);
            }
        }
    }

    // If no path found, return an empty path
    return vector<Node>();
}

int main() {
    // Define the grid (0 = empty, 1 = obstacle)
    vector<vector<int>> grid = {
        {0, 1, 0, 0, 0},
        {0, 1, 0, 1, 0},
        {0, 0, 0, 0, 0},
        {0, 1, 0, 1, 1},
        {0, 0, 0, 0, 0}
    };

    Node start(0, 0, 0, 0); // Start node (x, y, g, h)
    Node goal(4, 4, 0, 0); // Goal node (x, y, g, h)

    vector<Node> path = aStar(start, goal, grid);

    // Print the path
    if (!path.empty()) {
        cout << "Path found:" << endl;
        for (auto& node : path) {
            cout << "(" << node.x << ", " << node.y << ") ";
        }
        cout << endl;
    } else {
        cout << "No path found." << endl;
    }

    return 0;
}