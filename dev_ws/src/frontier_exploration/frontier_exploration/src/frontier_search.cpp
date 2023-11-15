#include <vector>
#include <queue>
#include <limits>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <frontier_exploration/costmap_tools.hpp>
#include <frontier_msgs/msg/frontier.hpp>

#include <frontier_exploration/frontier_search.hpp>

namespace frontier_exploration{

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;

FrontierSearch::FrontierSearch(nav2_costmap_2d::Costmap2D &costmap, int min_frontier_cluster_size) : costmap_(costmap) , min_frontier_cluster_size_(min_frontier_cluster_size) { 
    RCLCPP_INFO(rclcpp::get_logger("frontier_search"), "Frontier search constructor");
}

std::list<frontier_msgs::msg::Frontier> FrontierSearch::searchFrom(geometry_msgs::msg::Point position){

    //  frontier_list to store the detected frontiers.
    std::list<frontier_msgs::msg::Frontier> frontier_list;

    //Sanity check that robot is inside costmap bounds before searching
    unsigned int mx,my;
    if (!costmap_.worldToMap(position.x,position.y,mx,my)){
        RCLCPP_ERROR(rclcpp::get_logger("frontier_search"), "Robot out of costmap bounds, cannot search for frontiers");
        return frontier_list;
    }

    //make sure map is consistent and locked for duration of search
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_.getMutex()));

    map_ = costmap_.getCharMap();
    size_x_ = costmap_.getSizeInCellsX();
    size_y_ = costmap_.getSizeInCellsY();

    //initialize flag arrays to keep track of visited and frontier cells
    std::vector<bool> frontier_flag(size_x_ * size_y_, false);
    std::vector<bool> visited_flag(size_x_ * size_y_, false);

    //initialize breadth first search queue
    std::queue<unsigned int> bfs;

    //find closest clear cell to start search
    unsigned int clear, pos = costmap_.getIndex(mx,my);
    if(nearestCell(clear, pos, FREE_SPACE, costmap_)){
        bfs.push(clear);
    }else{
        bfs.push(pos);
        RCLCPP_WARN(rclcpp::get_logger("frontier_search"), "Could not find nearby clear cell to start search, pushing current position of robot to start search");
    }
    visited_flag[bfs.front()] = true;

    while(!bfs.empty()){
        unsigned int idx = bfs.front();
        bfs.pop();

        //iterate over 4-connected neighbourhood
        for (unsigned nbr : nhood4(idx, costmap_)) {
            //add to queue all free, unvisited cells, use descending search in case initialized on non-free cell
            if(map_[nbr] <= map_[idx] && !visited_flag[nbr]) {
                visited_flag[nbr] = true;
                bfs.push(nbr);
                //check if cell is new frontier cell (unvisited, NO_INFORMATION, free neighbour)
            }
            else if(isNewFrontierCell(nbr, frontier_flag)) {
                frontier_flag[nbr] = true;
                frontier_msgs::msg::Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag);
                if(new_frontier.size > min_frontier_cluster_size_) {
                    frontier_list.push_back(new_frontier);
                }
            }
        }
    }

    return frontier_list;

}

frontier_msgs::msg::Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell, unsigned int reference, std::vector<bool>& frontier_flag){

    //initialize frontier structure
    frontier_msgs::msg::Frontier output;
    output.centroid.x = 0;
    output.centroid.y = 0;
    output.size = 1;
    output.min_distance = std::numeric_limits<double>::infinity();

    //record initial contact point for frontier
    unsigned int ix, iy;
    costmap_.indexToCells(initial_cell,ix,iy);
    costmap_.mapToWorld(ix,iy,output.initial.x,output.initial.y);

    //push initial gridcell onto queue
    std::queue<unsigned int> bfs;
    bfs.push(initial_cell);

    //cache reference position in world coords
    unsigned int rx,ry;
    double reference_x, reference_y;
    costmap_.indexToCells(reference,rx,ry);
    costmap_.mapToWorld(rx,ry,reference_x,reference_y);

    while(!bfs.empty()){
        unsigned int idx = bfs.front();
        bfs.pop();

        //try adding cells in 8-connected neighborhood to frontier
        for (unsigned int nbr : nhood8(idx, costmap_)) {
            //check if neighbour is a potential frontier cell
            if(isNewFrontierCell(nbr,frontier_flag)){

                //mark cell as frontier
                frontier_flag[nbr] = true;
                unsigned int mx,my;
                double wx,wy;
                costmap_.indexToCells(nbr,mx,my);
                costmap_.mapToWorld(mx,my,wx,wy);

                //add to every frontier list
                std::vector<double> coord_val;
                coord_val.push_back(wx);
                coord_val.push_back(wy);

                every_frontier_list.push_back(coord_val);

                //update frontier size
                output.size++;

                //update centroid of frontier
                output.centroid.x += wx;
                output.centroid.y += wy;

                //determine frontier's distance from robot, going by closest gridcell to robot
                double distance = sqrt(pow((double(reference_x)-double(wx)),2.0) + pow((double(reference_y)-double(wy)),2.0));
                if(distance < output.min_distance){
                    output.min_distance = distance;
                    output.middle.x = wx;
                    output.middle.y = wy;
                }

                //add to queue for breadth first search
                bfs.push(nbr);
            }
        }
    }

    //average out frontier centroid
    output.centroid.x /= output.size;
    output.centroid.y /= output.size;
    return output;
}

bool FrontierSearch::isNewFrontierCell(unsigned int idx, const std::vector<bool>& frontier_flag){

    //check that cell is unknown and not already marked as frontier
    if(map_[idx] != NO_INFORMATION || frontier_flag[idx]){
        return false;
    }

    //frontier cells should have at least one cell in 4-connected neighbourhood that is free
    for (unsigned int nbr : nhood4(idx, costmap_)) {
        if(map_[nbr] == FREE_SPACE){
            return true;
        }
    }

    return false;

}

std::vector<std::vector<double>> FrontierSearch::getAllFrontiers() {
    return every_frontier_list;
}

}


#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"


nav2_costmap_2d::Costmap2D costmap;
int frontier_count = 0;

rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_initial;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_centroid;

void processFrontierListInitial(const std::list<frontier_msgs::msg::Frontier>& frontier_list) {
  visualization_msgs::msg::MarkerArray marker_array;

  int marker_id = 0;
  for (const auto& frontier : frontier_list) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "frontier_initial_points";
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = frontier.initial;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker_array.markers.push_back(marker);
  }
  marker_pub_initial->publish(marker_array);
}

void processFrontierListCentroid(const std::list<frontier_msgs::msg::Frontier>& frontier_list) {
  visualization_msgs::msg::MarkerArray marker_array;

  int marker_id = 0;
  for (const auto& frontier : frontier_list) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "frontier_centroid_points";
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = frontier.centroid;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0; // Set blue color
    marker.color.a = 1.0;

    if(frontier.size > 50){
    marker_array.markers.push_back(marker);
    }
  }
  marker_pub_centroid->publish(marker_array);
}


void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
  // Update the costmap with the new map
  costmap = nav2_costmap_2d::Costmap2D(*map);
}

void localizationPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose) {
  geometry_msgs::msg::Point robot_position = pose->pose.pose.position;
  frontier_exploration::FrontierSearch frontier_search(costmap, 5);
  std::list<frontier_msgs::msg::Frontier> frontier_list = frontier_search.searchFrom(robot_position);
  processFrontierListInitial(frontier_list);
  processFrontierListCentroid(frontier_list);
//   RCLCPP_INFO(rclcpp::get_logger("frontier_search"), "Search complete. Waiting for new pose message");
}

// int main(int argc, char** argv) {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<rclcpp::Node>("frontier_search");

//   auto map_sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
//     "map", 10, mapCallback);

//   auto localization_pose_sub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
//     "localization_pose", 10, localizationPoseCallback);

//   marker_pub_initial = node->create_publisher<visualization_msgs::msg::MarkerArray>("frontier_markers_initial", 10);
//   marker_pub_centroid = node->create_publisher<visualization_msgs::msg::MarkerArray>("frontier_markers_centroid", 10);
//   rclcpp::spin(node);
//   rclcpp::shutdown();

//   return 0;
// }