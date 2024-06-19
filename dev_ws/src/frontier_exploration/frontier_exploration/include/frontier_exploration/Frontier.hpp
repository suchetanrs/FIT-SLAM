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
    std::shared_ptr<double> fisher_information_in_path;

    // Individual costs
    std::map<std::string, double> costs;
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

    void setFisherInformation(double fi);

    void setCost(const std::string& costName, double value);

    void setWeightedCost(double cost);

    bool operator==(const Frontier& other) const;

    size_t getUID() const;

    int getSize() const;

    geometry_msgs::msg::Point getGoalPoint() const;

    geometry_msgs::msg::Quaternion getGoalOrientation() const;

    double getArrivalInformation() const;
    
    double getPathLength() const;

    double getFisherInformation() const;

    double getCost(const std::string& costName) const;

    double getWeightedCost() const;
};

// Custom equality function
struct FrontierGoalPointEquality {
    bool operator()(const Frontier& lhs, const Frontier& rhs) const {
        return lhs.getGoalPoint() == rhs.getGoalPoint();
    }
};

inline size_t generateUID(const Frontier& output)
{
    std::hash<double> hash_fn;
    
    // Hash each double value
    std::size_t hash1 = hash_fn(output.getGoalPoint().x);
    std::size_t hash2 = hash_fn(output.getGoalPoint().y);

    // return hash1 ^ (hash2 << 1);
    return hash1 ^ (hash2 << 1);
};
#endif