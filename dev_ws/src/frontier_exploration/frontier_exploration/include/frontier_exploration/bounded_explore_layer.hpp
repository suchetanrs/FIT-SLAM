#ifndef BOUNDED_EXPLORE_LAYER_HPP_
#define BOUNDED_EXPLORE_LAYER_HPP_

#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_events_filter.hpp>

#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav2_util/geometry_utils.hpp>

#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <frontier_msgs/msg/frontier.hpp>
#include <frontier_msgs/srv/get_frontier_costs.hpp>

#include <frontier_exploration/frontier_selection.hpp>
#include <frontier_exploration/colorize.hpp>

#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>

#include <slam_msgs/srv/get_map.hpp>

namespace frontier_exploration
{

    struct GetNextFrontierRequest
    {
        geometry_msgs::msg::PoseStamped start_pose;
        std::vector<frontier_msgs::msg::Frontier> frontier_list;
        std::vector<std::vector<double>> every_frontier;
        std::vector<frontier_msgs::msg::Frontier> prohibited_frontiers;       
    };

    struct GetNextFrontierResponse
    {
        bool success;                                           
        frontier_msgs::msg::Frontier next_frontier;             
        std::vector<frontier_msgs::msg::Frontier> frontier_list;
        std::vector<double> frontier_costs;                     
        std::vector<double> frontier_distances;                 
        std::vector<double> frontier_arrival_information;       
        std::vector<double> frontier_path_information;          
    };

    /**
     * @brief costmap_2d layer plugin that holds the state for a bounded frontier exploration task.
     * Manages the boundary polygon, superimposes the polygon on the overall exploration costmap,
     * and processes costmap to find next frontier to explore.
     */
    class BoundedExploreLayer
    {
    public:
        BoundedExploreLayer(nav2_costmap_2d::LayeredCostmap* costmap);
        ~BoundedExploreLayer();

        /**
         * @brief ROS Service wrapper for updateBoundaryPolygon
         * @param req Service request
         * @param res Service response
         * @return True on service success, false otherwise
         * The request parameter contains the following fields:
         * - explore_boundary: A polygon representing the area to be explored or navigated within. This polygon is defined
         *   using a list of vertices specifying its boundary. The explore_boundary field is of type geometry_msgs::PolygonStamped,
         *   which is a standard ROS message type for representing a polygon in 2D space.
         */
        bool updateBoundaryPolygon(geometry_msgs::msg::PolygonStamped& explore_boundary);

        /**
         * @brief ROS Service wrapper for getNextFrontier
         * @param req Service request
         * @param res Service response
         * @return True on service success, false otherwise
         * The request parameter contains the following fields:
         * - start_pose: Start pose from which to search for the next frontier. This pose is of type geometry_msgs/PoseStamped.
         * - override_frontier_list: A boolean flag indicating whether to override the default frontier list for exploration.
         * - frontier_list_to_override: An array of Frontier messages representing the frontier list to override the default exploration.
         *
         * The response parameter contains the following fields:
         * - success: Flag indicating whether the service call was successful.
         * - next_frontier: PoseStamped representing the next frontier to explore.
         * - frontier_list: An array of Frontier messages representing the updated frontier list after exploration.
         * - frontier_costs: An array of floating-point values representing the costs associated with each frontier.
         */
        bool getNextFrontier(std::shared_ptr<GetNextFrontierRequest> requestData, std::shared_ptr<GetNextFrontierResponse> resultData);
    
        void visualizeFrontier(std::shared_ptr<GetNextFrontierRequest> requestData);

    protected:

        /**
         * @brief A function the processes the frontier list and selects the best frontier using the FIT-SLAM approach.
         * @param req The same request given in the getNextFrontier service.
         * @param res The same response given in the getNextFrontier service.
         * @param selected The frontier selected after processing the FIT-SLAM approach.
         * @param frontier_list The frontier list to run the approach on.
         */
        SelectionResult processOurApproach(
            std::vector<frontier_msgs::msg::Frontier> &frontier_list,
            geometry_msgs::msg::Point& start_point_w);

        /**
         * @brief A function that processes the frontier list and selects a frontier randomly.
         *
         * This function randomly selects a frontier from the provided list of frontiers.
         *
         * @param selected The frontier selected randomly.
         * @param frontier_list The list of frontiers to choose from.
         * @param every_frontier A vector of every frontier. (unclustered)
         * @param req The request provided in the getNextFrontier service.
         * @param res The response provided in the getNextFrontier service.
         *
         * */
        std::pair<frontier_msgs::msg::Frontier, bool> processRandomApproach(
            std::vector<frontier_msgs::msg::Frontier> &frontier_list);

        /**
         * @brief A function that processes the frontier list and selects a frontier using the Greedy approach.
         *
         * This function selects a frontier from the provided list of frontiers based on the Greedy approach.
         * The Greedy approach selects the frontier that is closest to the current robot position.
         *
         * @param selected The frontier selected using the Greedy approach.
         * @param frontier_list The list of frontiers to choose from.
         * @param every_frontier A vector representing every frontier's coordinates. (unclustered)
         * @param req The request provided in the getNextFrontier service.
         * @param res The response provided in the getNextFrontier service.
         */
        std::pair<frontier_msgs::msg::Frontier, bool> processGreedyApproach(
            std::vector<frontier_msgs::msg::Frontier> &frontier_list);

        /**
         * @brief Callback executed when a parameter change is detected
         * @param parameters The parameters that are sent for dynamic monitoring.
         */
        rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

    private:
        geometry_msgs::msg::Polygon polygon_;    ///< A Polygon representing the boundary polygon for exploration. This is used to set the vector of polygon points.
        std::string exploration_mode_;           ///< String representing the exploration mode. {Ours, Greedy, Random}
        std::vector<double> polygon_xy_min_max_; ///< Polygon points for the boundary.
        bool explore_clear_space_;               ///< Used to set to explore the clear space or not. Sets the default_value_ variable in the costmap.

        std::shared_ptr<frontier_exploration::FrontierSelectionNode> frontierSelect_; ///< A pointer to the instance of the Selection Node.

        // COSTMAP INTERNAL
        bool enabledLayer_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        std::string current_robot_namespace_;
        std::chrono::_V2::system_clock::time_point startTime_ = std::chrono::high_resolution_clock::now();
        rclcpp_lifecycle::LifecycleNode::SharedPtr node;

        // ROS Dynamic parameters handler
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

        // ROS Clients
        rclcpp::Client<slam_msgs::srv::GetMap>::SharedPtr client_get_map_data2_;

        // ROS Publishers
        rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;

        /**
         * (TODO: suchetan)
         * This is a temporary workaround to use spin_until_future_complete
         * with a lifecycle node that has already been added to the executor.
         *
         * This node has been created to use with the service clients.
         */
        rclcpp::Node::SharedPtr internal_node_;
        std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> internal_executor_;
        nav2_costmap_2d::LayeredCostmap* layered_costmap_;
    };

}
#endif
