#ifndef LETHAL_MARKER_HPP_
#define LETHAL_MARKER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <frontier_exploration/util/logger.hpp>

namespace frontier_exploration
{
    struct WorldLocation {
        float x;
        float y;
    };

    class LethalMarker : public nav2_costmap_2d::Layer
    {
    public:
        LethalMarker();

        virtual void onInitialize();

        virtual void updateBounds(
            double robot_x, double robot_y, double robot_yaw,
            double *min_x, double *min_y, double *max_x, double *max_y);

        virtual void updateCosts(
            nav2_costmap_2d::Costmap2D &master_grid,
            int min_i, int min_j, int max_i, int max_j);

        void matchSize() override;

        virtual bool isClearable() override {return false;}

        void reset() override
        {
            matchSize();
            current_ = false;
        }

        virtual void onFootprintChanged();

        void addNewMarkedArea(double center_wx, double center_wy, double radius);

    private:
        // --------------------MEMBER FUNCTIONS-----------------------------------------
        void populateCellsToMark(
            nav2_costmap_2d::Costmap2D &master_grid, uint8_t keepout_config);

        void markCells(unsigned char *master_array, const std::vector<unsigned int>& cells_to_mark);

        rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

        const std::map<int, std::vector<unsigned int>>& getIndexCache() {
            std::lock_guard<std::mutex> lock(cacheMutex_);
            return latest_cells_to_mark_index_;
        }

        std::map<int, std::vector<WorldLocation>> getWorldCache() {
            std::lock_guard<std::mutex> lock(cacheMutex_);
            return latest_cells_to_mark_world_;
        }
        //--------------------MEMBER VARIABLES-----------------------------------------
        // ROS Internal
        std::mutex parameter_mutex_;
        rclcpp_lifecycle::LifecycleNode::SharedPtr node;

        // Variables used internally by the keepout layer class.
        std::mutex cacheMutex_;                                          ///< Mutex used to implement thread-safe implementation of latest_cells_to_mark_index_ variable.
        std::map<int, std::vector<unsigned int>> latest_cells_to_mark_index_; ///< Variable holding the latest cells to mark by the layer.
        std::map<int, std::vector<WorldLocation>> latest_cells_to_mark_world_; ///< Variable holding the latest cells to mark by the layer.

        double last_min_x_, last_min_y_, last_max_x_, last_max_y_; ///< Variables holding the last bounds of the costmap layer.
        bool need_recalculation_;                                  ///< Indicates that the entire costmap should be reinflated next time around.
        int numAdditions_;
    };

} // namespace frontier_exploration

#endif