#include "frontier_exploration/planners/astar.hpp"

FrontierRoadmapAStar::FrontierRoadmapAStar()
{
};

// Function to calculate heuristic (Euclidean distance in this case)
double FrontierRoadmapAStar::heuristic(const Node& a, const Node& b) {
    return sqrt((a.frontier.getGoalPoint().x - b.frontier.getGoalPoint().x) * (a.frontier.getGoalPoint().x - b.frontier.getGoalPoint().x) + (a.frontier.getGoalPoint().y - b.frontier.getGoalPoint().y) * (a.frontier.getGoalPoint().y - b.frontier.getGoalPoint().y));
}

// Function to get the successors of a node (example for a grid)
std::vector<Node> FrontierRoadmapAStar::getSuccessors(const Node& current, const Node& goal, std::unordered_map<Frontier, std::vector<Frontier>, FrontierHash>& roadmap_) {
    std::vector<Node> successors;
    if(roadmap_.count(current.frontier) == 0) {
        throw std::runtime_error("This should never happen. Frontier not found in roadmap.");
    }

    for (auto& dir : roadmap_[current.frontier]) {
        double newG = current.g + frontier_exploration::distanceBetweenFrontiers(current.frontier, dir); // Assuming uniform cost for each move
        double newH = heuristic(Node(dir, 0, 0), goal);
        successors.emplace_back(dir, newG, newH);
    }
    return successors;
}

// A* Algorithm function
std::vector<Node> FrontierRoadmapAStar::getPlan(const Frontier& start, const Frontier& goal, std::unordered_map<Frontier, std::vector<Frontier>, FrontierHash>& roadmap_) {
    auto start_ = Node(start, 0, 0);
    auto goal_ = Node(goal, 0, 0);
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openList; // Min-heap priority queue
    std::unordered_set<int> closedSet;
    std::unordered_map<int, Node*> allNodes; // To store all nodes and their costs

    openList.push(start_);
    allNodes[start_.frontier.getUID()] = new Node(start_);

    while (!openList.empty()) {
        Node current = openList.top();
        openList.pop();

        // If goal is reached
        if (current.frontier.getGoalPoint().x == goal_.frontier.getGoalPoint().x && current.frontier.getGoalPoint().y == goal_.frontier.getGoalPoint().y) {
            std::vector<Node> path;
            Node* node = allNodes[current.frontier.getUID()];
            while (node != nullptr) {
                path.push_back(*node);
                node = node->parent;
            }
            reverse(path.begin(), path.end());
            return path;
        }

        closedSet.insert(current.frontier.getGoalPoint().x);

        // Generate successors
        std::vector<Node> successors = getSuccessors(current, goal_, roadmap_);

        for (auto& successor : successors) {
            int successorHash = successor.frontier.getUID();

            if (closedSet.find(successorHash) != closedSet.end()) {
                continue;
            }

            if (allNodes.find(successorHash) == allNodes.end() || allNodes[successorHash]->g > successor.g) {
                successor.parent = allNodes[current.frontier.getUID()];
                allNodes[successorHash] = new Node(successor);
                openList.push(successor);
            }
        }
    }

    // If no path found, return an empty path
    return std::vector<Node>();
}

// int main() {
//     // Define the grid (0 = empty, 1 = obstacle)
//     std::vector<std::vector<int>> grid = {
//         {0, 1, 0, 0, 0},
//         {0, 1, 0, 1, 0},
//         {0, 0, 0, 0, 0},
//         {0, 1, 0, 1, 1},
//         {0, 0, 0, 0, 0}
//     };

//     Node start(0, 0, 0, 0); // Start node (x, y, g, h)
//     Node goal(4, 4, 0, 0); // Goal node (x, y, g, h)

//     std::vector<Node> path = aStar(start, goal, grid);

//     // Print the path
//     if (!path.empty()) {
//         cout << "Path found:" << endl;
//         for (auto& node : path) {
//             cout << "(" << node.x << ", " << node.y << ") ";
//         }
//         cout << endl;
//     } else {
//         cout << "No path found." << endl;
//     }

//     return 0;
// }