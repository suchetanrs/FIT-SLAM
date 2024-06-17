#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <fstream>
#include <algorithm>

// Define a structure to represent a point in the grid
struct Point {
    int x, y;
    Point(int x, int y) : x(x), y(y) {}
    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }
};

// Define a structure to represent a node in the A* algorithm
struct Node {
    Point point;
    double g, h; // g: cost from start to current node, h: heuristic cost to goal
    Node* parent;
    Node(Point p, double g, double h, Node* parent) : point(p), g(g), h(h), parent(parent) {}
    double f() const { return g + h; }
};

// Comparison function for the priority queue (min-heap based on f value)
struct CompareNodes {
    bool operator()(const Node* a, const Node* b) const {
        return a->f() > b->f();
    }
};

// Heuristic function: Euclidean distance
double heuristic(const Point& a, const Point& b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

// Check if a point is within the grid and not an obstacle
bool isValidPoint(const Point& p, const std::vector<std::vector<int>>& grid) {
    return p.x >= 0 && p.x < grid.size() && p.y >= 0 && p.y < grid[0].size() && grid[p.x][p.y] == 0;
}

// Get the path from start to goal by backtracking the parent pointers
std::vector<Point> constructPath(Node* node) {
    std::vector<Point> path;
    while (node != nullptr) {
        path.push_back(node->point);
        node = node->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

// A* pathfinding algorithm
std::vector<Point> aStar(const Point& start, const Point& goal, const std::vector<std::vector<int>>& grid) {
    std::priority_queue<Node*, std::vector<Node*>, CompareNodes> openSet;
    std::vector<std::vector<bool>> closedSet(grid.size(), std::vector<bool>(grid[0].size(), false));
    
    Node* startNode = new Node(start, 0, heuristic(start, goal), nullptr);
    openSet.push(startNode);

    // Direction vectors for the 4-connected grid
    std::vector<Point> directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    
    while (!openSet.empty()) {
        Node* current = openSet.top();
        openSet.pop();

        if (current->point == goal) {
            std::vector<Point> path = constructPath(current);
            // Clean up dynamically allocated nodes
            while (!openSet.empty()) {
                delete openSet.top();
                openSet.pop();
            }
            return path;
        }

        closedSet[current->point.x][current->point.y] = true;

        for (const auto& dir : directions) {
            Point neighbor(current->point.x + dir.x, current->point.y + dir.y);

            if (isValidPoint(neighbor, grid) && !closedSet[neighbor.x][neighbor.y]) {
                double tentativeG = current->g + 1; // Each step cost is 1

                Node* neighborNode = new Node(neighbor, tentativeG, heuristic(neighbor, goal), current);
                openSet.push(neighborNode);
            }
        }
    }

    return {}; // Return empty path if no path is found
}

int main() {
    // Define the grid (0: free, 1: obstacle)
    std::vector<std::vector<int>> grid = {
        {0, 0, 0, 0, 0, 1, 0, 0, 0, 0},
        {0, 1, 1, 0, 0, 1, 0, 1, 1, 0},
        {0, 0, 0, 0, 0, 1, 0, 0, 0, 0},
        {0, 0, 1, 1, 1, 1, 1, 1, 0, 0},
        {0, 0, 0, 0, 1, 0, 0, 0, 0, 0}
    };

    Point start(0, 0);
    Point goal(4, 5);

    std::vector<Point> path = aStar(start, goal, grid);

    // Output path to a file for gnuplot
    std::ofstream outFile("path.dat");
    for (const auto& point : path) {
        outFile << point.x << " " << point.y << "\n";
    }
    outFile.close();

    // Output grid to a file for gnuplot
    std::ofstream gridFile("grid.dat");
    for (int x = 0; x < grid.size(); ++x) {
        for (int y = 0; y < grid[0].size(); ++y) {
            if (grid[x][y] == 1) {
                gridFile << x << " " << y << "\n";
            }
        }
    }
    gridFile.close();

    std::cout << "Pathfinding complete. Check path.dat and grid.dat for visualization.\n";
    return 0;
}
