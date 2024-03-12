#ifndef BOUNDED_EXPLORE_LAYER_HPP_
#define BOUNDED_EXPLORE_LAYER_HPP_

#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/parameter_events_filter.hpp>

#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/footprint.hpp>

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
#include <frontier_msgs/srv/update_boundary_polygon.hpp>
#include <frontier_msgs/srv/get_next_frontier.hpp>
#include <frontier_msgs/srv/get_frontier_costs.hpp>

#include <frontier_exploration/frontier_selection.hpp>

#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>

#include <slam_msgs/srv/get_map.hpp>

namespace frontier_exploration
{
    /**
     * @brief costmap_2d layer plugin that holds the state for a bounded frontier exploration task.
     * Manages the boundary polygon, superimposes the polygon on the overall exploration costmap,
     * and processes costmap to find next frontier to explore.
     */
    class BoundedExploreLayer : public nav2_costmap_2d::Layer, public nav2_costmap_2d::Costmap2D
    {
    public:
        BoundedExploreLayer();
        ~BoundedExploreLayer();

        /**
         * @brief Loads default values and initialize exploration costmap.
         */
        virtual void onInitialize();

        /**
         * @brief Calculate bounds of costmap window to update
         */
        virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double *polygon_min_x, double *polygon_min_y, double *polygon_max_x,
                                  double *polygon_max_y);

        /**
         * @brief Update requested costmap window
         */
        virtual void updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j);

        /**
         * @brief Match dimensions and origin of parent costmap
         */
        void matchSize() override;

        /**
         * @brief If clearing operations should be processed on this layer or not
         */
        virtual bool isClearable() { return false; }

        /**
         * @brief Reset exploration progress
         */
        virtual void reset();

        // ROS SERVICES
        rclcpp::Service<frontier_msgs::srv::UpdateBoundaryPolygon>::SharedPtr polygonService_;
        rclcpp::Service<frontier_msgs::srv::GetNextFrontier>::SharedPtr frontierService_;

    protected:
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
        void updateBoundaryPolygonService(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<frontier_msgs::srv::UpdateBoundaryPolygon::Request> req,
            std::shared_ptr<frontier_msgs::srv::UpdateBoundaryPolygon::Response> res);

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
        void getNextFrontierService(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<frontier_msgs::srv::GetNextFrontier::Request> req,
            std::shared_ptr<frontier_msgs::srv::GetNextFrontier::Response> res);

        /**
         * @brief A function the processes the frontier list and selects the best frontier using the FIT-SLAM approach.
         * @param req The same request given in the getNextFrontier service.
         * @param res The same response given in the getNextFrontier service.
         * @param selected The frontier selected after processing the FIT-SLAM approach.
         * @param frontier_list The frontier list to run the approach on.
         */
        void processOurApproach(
            frontier_msgs::msg::Frontier &selected,
            std::vector<frontier_msgs::msg::Frontier> &frontier_list,
            const std::shared_ptr<frontier_msgs::srv::GetNextFrontier::Request> req,
            std::shared_ptr<frontier_msgs::srv::GetNextFrontier::Response> res);

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
        void processRandomApproach(
            frontier_msgs::msg::Frontier &selected,
            std::vector<frontier_msgs::msg::Frontier> &frontier_list,
            const std::vector<std::vector<double>> &every_frontier,
            const std::shared_ptr<frontier_msgs::srv::GetNextFrontier::Request> req,
            std::shared_ptr<frontier_msgs::srv::GetNextFrontier::Response> res);

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
        void processGreedyApproach(
            frontier_msgs::msg::Frontier &selected,
            std::vector<frontier_msgs::msg::Frontier> &frontier_list,
            const std::vector<std::vector<double>> &every_frontier,
            const std::shared_ptr<frontier_msgs::srv::GetNextFrontier::Request> req,
            std::shared_ptr<frontier_msgs::srv::GetNextFrontier::Response> res);

        /**
         * @brief Update the map with exploration boundary data
         * @param master_grid Reference to master costmap
         */
        void mapUpdateKeepObstacles(nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j);

        /**
         * @brief Callback executed when a parameter change is detected
         * @param parameters The parameters that are sent for dynamic monitoring.
         */
        rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

    private:
        geometry_msgs::msg::Polygon polygon_;    ///< A Polygon representing the boundary polygon for exploration. This is used to set the vector of polygon points.
        int min_frontier_cluster_size_;          ///< Minimum size of a frontier cluster.
        std::string exploration_mode_;           ///< String representing the exploration mode. {Ours, Greedy, Random}
        std::vector<double> polygon_xy_min_max_; ///< Polygon points for the boundary.
        std::string frontier_travel_point_;      ///< Used to set the frontier travel point. {Closest, Centroid, Middle}
        bool explore_clear_space_;               ///< Used to set to explore the clear space or not. Sets the default_value_ variable in the costmap.

        std::shared_ptr<frontier_exploration::FrontierSelectionNode> frontierSelect_; ///< A pointer to the instance of the Selection Node.

        // COSTMAP INTERNAL
        bool enabledLayer_;
        bool configured_, marked_;
        bool resize_to_boundary_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

        std::string current_robot_namespace_;
        std::vector<std::string> robot_namespaces_;
        std::chrono::_V2::system_clock::time_point startTime_ = std::chrono::high_resolution_clock::now();
        rclcpp_lifecycle::LifecycleNode::SharedPtr node;

        // ROS Dynamic parameters handler
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

        // ROS Clients
        rclcpp::Client<slam_msgs::srv::GetMap>::SharedPtr client_get_map_data2_;
        rclcpp::Client<frontier_msgs::srv::GetFrontierCosts>::SharedPtr client_get_frontier_costs_;

        // ROS Publishers
        rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;

        /**
         * (TODO: suchetan)
         * This is a temporary workaround to use spin_until_future_complete
         * with a lifecycle node that has already been added to the executor.
         *
         * This node has been created to use with the service clients.
         */
        rclcpp::Node::SharedPtr client_node_;
    };

}
#endif
