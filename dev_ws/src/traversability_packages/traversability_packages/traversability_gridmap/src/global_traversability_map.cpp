#include <traversability_gridmap/global_traversability_map.hpp>

namespace traversability_gridmap {


    // Insert function for adding a grid map to the vector
    void GlobalTraversabilityMap::insertGridMapNodeIDPCL(const grid_map::GridMap& map, int node_id, sensor_msgs::msg::PointCloud2 pcl, geometry_msgs::msg::Pose pose) {
        std::lock_guard<std::mutex> lock(gridMapMutex_);

        // Create a shared_ptr to GridMsgWithHeader and initialize it
        std::shared_ptr<GridMapWithNodeIDPCL> gridMapWithHeaderPtr = std::make_shared<GridMapWithNodeIDPCL>(map, node_id, pcl, pose);

        // Populate the map.
        gridMapsWithNodeIds_[node_id] = gridMapWithHeaderPtr;
    }

    // Access functions for the gridmap and poses related.
    const std::map<int, std::shared_ptr<GridMapWithNodeIDPCL>>& GlobalTraversabilityMap::getGridMapsWithIDPCL() const {
        std::lock_guard<std::mutex> lock(gridMapMutex_);
        return gridMapsWithNodeIds_;
    }


} // namespace traversability_gridmap