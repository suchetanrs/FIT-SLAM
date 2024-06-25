#include <iostream>
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <cmath>
#include <functional>
#include "frontier_exploration/Frontier.hpp"
#include "frontier_exploration/colorize.hpp"
#include "frontier_exploration/Helpers.hpp"

// Define a Node structure
struct Node {
    Frontier frontier;
    double g; // Cost from the start node to this node
    double h; // Heuristic cost estimate to the goal
    double f; // Total cost (f = g + h)
    Node* parent; // Pointer to the parent node

    Node();

    Node(Frontier frontier_in, double g, double h, Node* parent = nullptr) 
        : frontier(frontier_in), g(g), h(h), f(g + h), parent(parent) {}

    // Comparator for priority queue to order by f value
    bool operator>(const Node& other) const {
        return f > other.f;
    }
};

class FrontierRoadmapAStar {
public:
    FrontierRoadmapAStar();

    std::vector<Node> getPlan(const Frontier& start, const Frontier& goal, std::unordered_map<Frontier, std::vector<Frontier>, FrontierHash>& roadmap_);

protected:
    double heuristic(const Node& a, const Node& b);

    std::vector<Node> getSuccessors(const Node& current, const Node& goal, std::unordered_map<Frontier, std::vector<Frontier>, FrontierHash>& roadmap_);

};