#ifndef FRONTIER_HPP_
#define FRONTIER_HPP_

// Include necessary headers
#include <vector>
#include <memory>
#include <map>
#include <limits>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav2_util/geometry_utils.hpp>

// Constants for weights
#define ALPHA 0.5
#define BETA 0.5 

template <typename T>
inline void setValue(std::shared_ptr<T>& ptr, T value) {
    if (ptr == nullptr) {
        ptr = std::make_shared<T>(value);
    } else {
        *ptr = value;
    }
}

inline void setMapValue(std::shared_ptr<std::map<std::string, double>>& mapPtr, std::string key, double value) {
    // Check if the map itself is initialized
    if (mapPtr == nullptr) {
        mapPtr = std::make_shared<std::map<std::string, double>>();
    }

    // Set or update the value for the given key in the map
    (*mapPtr)[key] = value;
}

class Frontier {
private:
    // Variables not unique to a robot
    std::shared_ptr<size_t> unique_id;
    std::shared_ptr<int> size;
    std::shared_ptr<geometry_msgs::msg::Point> goal_point;
    std::shared_ptr<geometry_msgs::msg::Quaternion> best_orientation;
    std::shared_ptr<double> theta_s_star;
    std::shared_ptr<double> information;

    // Variables unique to a robot
    std::shared_ptr<double> path_length;
    std::shared_ptr<double> path_length_m;
    std::shared_ptr<double> path_heading;
    std::shared_ptr<double> fisher_information_in_path;
    std::shared_ptr<bool> is_achievable;

    // Individual costs
    std::shared_ptr<std::map<std::string, double>> costs;
    // Weighted cost
    std::shared_ptr<double> weighted_cost;

public:
    // Constructor
    Frontier();

    void setUID(size_t uid);

    void setSize(int sz);

    void setGoalPoint(geometry_msgs::msg::Point gp);

    void setGoalPoint(double x, double y);

    void setGoalOrientation(double theta);

    void setArrivalInformation(double info);
    
    void setPathLength(double pl);

    void setPathLengthInM(double pl);

    void setPathHeading(double heading_rad);

    void setFisherInformation(double fi);

    void setCost(std::string costName, double value);

    void setWeightedCost(double cost);

    void setAchievability(bool value);

    bool operator==(const Frontier& other) const;

    size_t getUID() const;

    int getSize() const;

    geometry_msgs::msg::Point getGoalPoint() const;

    geometry_msgs::msg::Quaternion getGoalOrientation() const;

    double getArrivalInformation() const;
    
    double getPathLength() const;

    double getPathLengthInM() const;

    double getPathHeading() const;

    double getFisherInformation() const;

    double getCost(const std::string& costName) const;

    double getWeightedCost() const;

    bool isAchievable() const;

    bool operator<(const Frontier& other) const {
        return getUID() < other.getUID();
    };

    friend std::ostream& operator<<(std::ostream& os, const Frontier& obj) {
        // Customize the output format here
        os << "Frontier(x: " << obj.getGoalPoint().x << ", y: " << obj.getGoalPoint().y << ")";
        os << "Frontier Path Length(PL: " << obj.getPathLength() << ", PLm: " << obj.getPathLength() << ")";
        return os;
    }
};

// Custom equality function
struct FrontierGoalPointEquality {
    bool operator()(const Frontier& lhs, const Frontier& rhs) const {
        return lhs.getGoalPoint() == rhs.getGoalPoint();
    }
};

struct FrontierGoalPointEqualityInRange {
    bool operator()(const Frontier& lhs, const Frontier& rhs) const {
        return abs(lhs.getGoalPoint().x - rhs.getGoalPoint().x) < 0.3 && abs(lhs.getGoalPoint().y - rhs.getGoalPoint().y) < 0.3;
    }
};

inline size_t generateUID(const Frontier& output)
{
    std::hash<double> hash_fn;
    // std::cout << "Generating UID" << std::endl;
    // Hash each double value
    std::size_t hash1 = hash_fn(output.getGoalPoint().x);
    std::size_t hash2 = hash_fn(output.getGoalPoint().y);

    // return hash1 ^ (hash2 << 1);
    return hash1 ^ (hash2 << 1);
};
#endif