#include <frontier_exploration/Parameters.hpp>

std::unique_ptr<ParameterHandler> ParameterHandler::parameterHandlerPtr_ = nullptr;
std::mutex ParameterHandler::instanceMutex_;

ParameterHandler::ParameterHandler()
{
    // Load YAML file and retrieve parameters
    YAML::Node loaded_node = YAML::LoadFile("/root/dev_ws/src/frontier_exploration/frontier_exploration/params/exploration.yaml");

    parameter_map_["frontierSearch/min_frontier_cluster_size"] = loaded_node["frontierSearch"]["min_frontier_cluster_size"].as<double>();
    parameter_map_["frontierSearch/max_frontier_cluster_size"] = loaded_node["frontierSearch"]["max_frontier_cluster_size"].as<double>();
    parameter_map_["frontierSearch/max_frontier_distance"] = loaded_node["frontierSearch"]["max_frontier_distance"].as<double>();
    parameter_map_["frontierSearch/lethal_threshold"] = loaded_node["frontierSearch"]["lethal_threshold"].as<int>();

    parameter_map_["costCalculator/max_camera_depth"] = loaded_node["costCalculator"]["max_camera_depth"].as<double>();
    parameter_map_["costCalculator/delta_theta"] = loaded_node["costCalculator"]["delta_theta"].as<double>();
    parameter_map_["costCalculator/camera_fov"] = loaded_node["costCalculator"]["camera_fov"].as<double>();

    parameter_map_["frontierCostsManager/alpha"] = loaded_node["frontierCostsManager"]["alpha"].as<double>();
    parameter_map_["frontierCostsManager/beta"] = loaded_node["frontierCostsManager"]["beta"].as<double>();
    parameter_map_["frontierCostsManager/planner_allow_unknown"] = loaded_node["frontierCostsManager"]["planner_allow_unknown"].as<bool>();
    parameter_map_["frontierCostsManager/N_best_for_u2"] = loaded_node["frontierCostsManager"]["N_best_for_u2"].as<int>();
    parameter_map_["frontierCostsManager/add_heading_cost"] = loaded_node["frontierCostsManager"]["add_heading_cost"].as<bool>();
    parameter_map_["frontierCostsManager/vx_max"] = loaded_node["frontierCostsManager"]["vx_max"].as<double>();
    parameter_map_["frontierCostsManager/wz_max"] = loaded_node["frontierCostsManager"]["wz_max"].as<double>();

    parameter_map_["frontierRoadmap/max_frontier_distance"] = loaded_node["frontierRoadmap"]["max_frontier_distance"].as<double>();
    parameter_map_["frontierRoadmap/grid_cell_size"] = loaded_node["frontierRoadmap"]["grid_cell_size"].as<double>();
    parameter_map_["frontierRoadmap/radius_to_decide_edges"] = loaded_node["frontierRoadmap"]["radius_to_decide_edges"].as<double>();
    parameter_map_["frontierRoadmap/min_distance_between_two_frontier_nodes"] = loaded_node["frontierRoadmap"]["min_distance_between_two_frontier_nodes"].as<double>();
    parameter_map_["frontierRoadmap/min_distance_between_robot_pose_and_node"] = loaded_node["frontierRoadmap"]["min_distance_between_robot_pose_and_node"].as<double>();

    parameter_map_["goalHysteresis/use_euclidean_distance"] = loaded_node["goalHysteresis"]["use_euclidean_distance"].as<bool>();
    parameter_map_["goalHysteresis/use_roadmap_planner_distance"] = loaded_node["goalHysteresis"]["use_roadmap_planner_distance"].as<bool>();

    parameter_map_["explorationBT/bt_sleep_ms"] = loaded_node["explorationBT"]["bt_sleep_ms"].as<int>();
    parameter_map_["explorationBT/recover_on_blacklist"] = loaded_node["explorationBT"]["recover_on_blacklist"].as<bool>();

    parameter_map_["fisherInformation/fisher_information_threshold"] = loaded_node["fisherInformation"]["fisher_information_threshold"].as<double>();


    // sanity checks.
    if(loaded_node["goalHysteresis"]["use_euclidean_distance"].as<bool>() == true && loaded_node["goalHysteresis"]["use_roadmap_planner_distance"].as<bool>() == true)
    {
        throw std::runtime_error("Both use_euclidean_distance and use_roadmap_planner_distance are set to true. Please set only one of them to true.");
    }
    else if(loaded_node["goalHysteresis"]["use_euclidean_distance"].as<bool>() == false && loaded_node["goalHysteresis"]["use_roadmap_planner_distance"].as<bool>() == false)
    {
        throw std::runtime_error("Both use_euclidean_distance and use_roadmap_planner_distance are set to false. Please set only one of them to true.");
    }

    if(loaded_node["frontierSearch"]["lethal_threshold"].as<int>() > 255 || loaded_node["frontierSearch"]["lethal_threshold"].as<int>() < 0)
    {
        throw std::runtime_error("Lethal thresholds out of unsigned char range.");
    }
}