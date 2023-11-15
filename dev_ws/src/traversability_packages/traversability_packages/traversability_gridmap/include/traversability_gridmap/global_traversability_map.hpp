#ifndef GLOBAL_TRAVERSABILITY_MAP_HPP
#define GLOBAL_TRAVERSABILITY_MAP_HPP

#include <vector>
#include <mutex>
#include <grid_map_ros/grid_map_ros.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace traversability_gridmap {


class GridMapWithNodeIDPCL {
public:
    GridMapWithNodeIDPCL(const grid_map::GridMap& gridMsg, const int node_id, 
    sensor_msgs::msg::PointCloud2 pcl, geometry_msgs::msg::Pose pose)
        : gridMsg(gridMsg), node_id(node_id), pcl(pcl), pose(pose) {}

    grid_map::GridMap gridMsg;
    int node_id;
    sensor_msgs::msg::PointCloud2 pcl;
    geometry_msgs::msg::Pose pose;
};



class GlobalTraversabilityMap {
public:

    // Constructor
    GlobalTraversabilityMap() {}

    // Destructor
    ~GlobalTraversabilityMap() {}

    // Insert function for adding a grid map to the vector
    void insertGridMapNodeIDPCL(const grid_map::GridMap& map, int node_id, sensor_msgs::msg::PointCloud2 pcl, geometry_msgs::msg::Pose pose);

    // Access functions for the gridmap and poses related.
    const std::map<int, std::shared_ptr<GridMapWithNodeIDPCL>>& getGridMapsWithIDPCL() const;

private:

    // std::vector<GridMapWithNodeIDPCL> gridMapsWithNodeIds_;
    std::map<int, std::shared_ptr<GridMapWithNodeIDPCL>> gridMapsWithNodeIds_;
    mutable std::mutex gridMapMutex_;
};

} // namespace traversability_gridmap

#endif
