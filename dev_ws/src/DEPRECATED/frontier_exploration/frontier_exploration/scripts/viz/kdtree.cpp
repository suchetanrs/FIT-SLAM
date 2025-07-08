#include <iostream>
#include <vector>
#include <limits>
#include <algorithm>
#include <cmath>

struct Point {
    double x, y;
};

double squaredDistance(const Point& a, const Point& b) {
    return sqrt(((a.x - b.x) * (a.x - b.x)) + ((a.y - b.y) * (a.y - b.y)));
}

struct KDNode {
    Point point;
    KDNode *left, *right;

    KDNode(const Point& pt) : point(pt), left(nullptr), right(nullptr) {}
};

KDNode* buildKDTree(std::vector<Point>& points, int depth = 0) {
    if (points.empty()) return nullptr;

    int axis = depth % 2;

    if (axis == 0)
        std::sort(points.begin(), points.end(), [](const Point& a, const Point& b) { return a.x < b.x; });
    else
        std::sort(points.begin(), points.end(), [](const Point& a, const Point& b) { return a.y < b.y; });

    size_t medianIndex = points.size() / 2;
    Point medianPoint = points[medianIndex];

    KDNode* node = new KDNode(medianPoint);

    std::vector<Point> leftPoints(points.begin(), points.begin() + medianIndex);
    std::vector<Point> rightPoints(points.begin() + medianIndex + 1, points.end());

    node->left = buildKDTree(leftPoints, depth + 1);
    node->right = buildKDTree(rightPoints, depth + 1);

    return node;
}

void nearestNeighborSearch(KDNode* root, const Point& target, int depth, KDNode*& best, double& bestDist) {
    if (!root) return;

    double dist = squaredDistance(target, root->point);
    
    if (dist < bestDist) {
        bestDist = dist;
        best = root;
    }

    int axis = depth % 2;

    KDNode* nextBranch = nullptr;
    KDNode* oppositeBranch = nullptr;

    if ((axis == 0 && target.x < root->point.x) || (axis == 1 && target.y < root->point.y)) {
        nextBranch = root->left;
        oppositeBranch = root->right;
    } else {
        nextBranch = root->right;
        oppositeBranch = root->left;
    }

    nearestNeighborSearch(nextBranch, target, depth + 1, best, bestDist);

    double axisDist = (axis == 0) ? (target.x - root->point.x) : (target.y - root->point.y);
    if (axisDist * axisDist < bestDist) {
        nearestNeighborSearch(oppositeBranch, target, depth + 1, best, bestDist);
    }
}

Point findNearestNeighbor(KDNode* root, const Point& target) {
    KDNode* best = nullptr;
    double bestDist = std::numeric_limits<double>::max();
    nearestNeighborSearch(root, target, 0, best, bestDist);
    return best ? best->point : Point{0, 0};
}

int main() {
    // Example points
    std::vector<Point> points = {{1.0, 2.0}, {3.0, 6.0}, {7.0, 1.0}, {4.0, 5.0}, {9.0, 7.0}};
    
    // Build KD-Tree
    KDNode* root = buildKDTree(points);

    // Target point
    Point target = {5.0, 5.0};

    // Find the nearest neighbor
    Point nearest = findNearestNeighbor(root, target);
    std::cout << "Nearest point to (" << target.x << ", " << target.y << ") is (" 
              << nearest.x << ", " << nearest.y << ")" << std::endl;

    // Free allocated memory (optional in this example but necessary in a real program)
    // You'd need to implement a tree traversal to delete all nodes

    return 0;
}



















            // ----------------construct KD-tree-------------------------
            Kdtree::KdNodeVector nodes;
            for (int i = 0; i < frontierCostsRequestPtr->frontier_list.size(); ++i) {
                nodes.push_back(Kdtree::KdNode(frontierCostsRequestPtr->frontier_list[i]));
            }
            Kdtree::KdTree tree(&nodes);
            // ----------------Print the nodes-------------------------
            cout << "Points in kd-tree:\n  ";
            print_nodes(tree.allnodes);

            // ----------------Search nearest nodes-------------------------
            Kdtree::KdNodeVector result;
            std::vector<double> test_point(2);
            test_point[0] = 8;
            test_point[1] = 3;
            tree.k_nearest_neighbors(test_point, 3, &result);
            cout << "3NNs of (" << test_point[0] << "," << test_point[1] << "):\n  ";
            print_nodes(result);
            RCLCPP_WARN_STREAM(this->get_logger(), COLOR_STR("Clusterred frontier size: " + std::to_string(frontierCostsRequestPtr->frontier_list.size()), this->get_logger().get_name()));
            RCLCPP_WARN_STREAM(this->get_logger(), COLOR_STR("Every frontier size: " + std::to_string(frontierCostsRequestPtr->every_frontier.size()), this->get_logger().get_name()));

            test_point[0] = 8;
            test_point[1] = 3;
            tree.range_nearest_neighbors(test_point, 0.1, &result);
            cout << "Neighbors of (" << test_point[0] << "," << test_point[1] << ") with distance <= 0.1:\n  ";
            print_nodes(result);
            cout << endl << result.size() << endl;