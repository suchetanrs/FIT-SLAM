// Include necessary headers
#include <frontier_exploration/Frontier.hpp>

// Constructor
Frontier::Frontier()
{
    unique_id = nullptr;
    size = nullptr;
    goal_point = nullptr;
    best_orientation = nullptr;
    theta_s_star = nullptr;
    information = nullptr;
    path_length = nullptr;
    weighted_cost = nullptr;
    is_achievable = true;
}

void Frontier::setUID(size_t uid)
{
    unique_id = std::make_shared<size_t>(uid);
}

void Frontier::setSize(int sz)
{
    size = std::make_shared<int>(sz);
}

void Frontier::setGoalPoint(geometry_msgs::msg::Point gp)
{
    goal_point = std::make_shared<geometry_msgs::msg::Point>(gp);
}

void Frontier::setGoalPoint(double x, double y)
{
    geometry_msgs::msg::Point pnt;
    pnt.x = x;
    pnt.y = y;
    goal_point = std::make_shared<geometry_msgs::msg::Point>(pnt);
}

void Frontier::setGoalOrientation(double theta)
{
    theta_s_star = std::make_shared<double>(theta);
    best_orientation = std::make_shared<geometry_msgs::msg::Quaternion>(nav2_util::geometry_utils::orientationAroundZAxis(theta));
}

void Frontier::setArrivalInformation(double info)
{
    information = std::make_shared<double>(info);
}

void Frontier::setPathLength(double pl)
{
    path_length = std::make_shared<double>(pl);
}

void Frontier::setFisherInformation(double fi)
{
    fisher_information_in_path = std::make_shared<double>(fi);
}

void Frontier::setCost(const std::string &costName, double value)
{
    costs[costName] = value;
}

void Frontier::setWeightedCost(double cost)
{
    weighted_cost = std::make_shared<double>(cost);
}

void Frontier::setAchievability(bool value)
{
    is_achievable = value;
}

// Equality operator definition
bool Frontier::operator==(const Frontier &other) const
{
    if (unique_id != other.unique_id ||
        size != other.size)
    {
        return false;
    }

    if (goal_point != other.goal_point)
    {
        return false;
    }

    if (best_orientation != other.best_orientation)
    {
        return false;
    }

    return true;
}

size_t Frontier::getUID() const
{
    if (unique_id == nullptr)
        throw std::runtime_error("Goal uid frontier property is null");
    return *unique_id;
}

int Frontier::getSize() const
{
    if (size == nullptr)
        throw std::runtime_error("Size frontier property is null");
    return *size;
}

geometry_msgs::msg::Point Frontier::getGoalPoint() const
{
    if (goal_point == nullptr)
        throw std::runtime_error("Goal point frontier property is null");
    return *goal_point;
}

geometry_msgs::msg::Quaternion Frontier::getGoalOrientation() const
{
    if (best_orientation == nullptr || theta_s_star == nullptr)
        throw std::runtime_error("Goal orientation frontier property is null");
    return *best_orientation;
}

double Frontier::getArrivalInformation() const
{
    if (information == nullptr)
        throw std::runtime_error("Arrival information frontier property is null");
    return *information;
}

double Frontier::getPathLength() const
{
    if (path_length == nullptr)
        throw std::runtime_error("Path length frontier property is null");
    return *path_length;
}

double Frontier::getFisherInformation() const
{
    if (fisher_information_in_path == nullptr)
        throw std::runtime_error("A frontier property is null");
    return *fisher_information_in_path;
}

double Frontier::getCost(const std::string &costName) const
{
    auto it = costs.find(costName);
    if (it != costs.end())
    {
        return it->second;
    }
    throw std::runtime_error("Cost requested for not a defined field");
}

double Frontier::getWeightedCost() const
{
    if (weighted_cost == nullptr)
        throw std::runtime_error("Weighted cost frontier property is null");
    return *weighted_cost;
}

bool Frontier::isAchievable() const
{
    return is_achievable;
}

// double Frontier::getWeightedCost() {
//     // Retrieve individual costs from the map
//     double distance_cost = getCost("distance_cost");
//     double arrival_cost = getCost("arrival_cost");
//     double FIMs = getCost("landmark_cost");

//     // Check if costs are valid (not NaN)
//     if (std::isnan(distance_cost) || std::isnan(arrival_cost) || std::isnan(FIMs)) {
//         return std::numeric_limits<double>::quiet_NaN(); // Return NaN if any cost is undefined
//     }

//     // Calculate weighted cost using the formula
//     double weighted_cost = (BETA * ((ALPHA * arrival_cost) + ((1 - ALPHA) * distance_cost))) + ((1 - BETA) * FIMs);
//     return weighted_cost;
// }