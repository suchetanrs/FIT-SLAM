#include "keepout_zones/keepout_layer.hpp"

namespace fit_slam2_nav2_plugins
{
    void cellEnforceBoundaries(int64_t& x, int64_t& y, nav2_costmap_2d::Costmap2D& costmap) {
        // this takes an x and y and makes sure it is within the costmap bounds.
        x = (x < 0) ? 0 : x;
        x = (x > costmap.getSizeInCellsX() - 1) ? costmap.getSizeInCellsX() - 1 : x;
        y = (y < 0) ? 0 : y;
        y = (y > costmap.getSizeInCellsY() - 1) ? costmap.getSizeInCellsY() - 1 : y;
    }

    void rayTraceGeneric(unsigned int map_width, unsigned int map_height,
    int64_t x0, int64_t y0, int64_t x1, int64_t y1, std::vector<int64_t>& raytraced_cells) {
        int64_t dx = std::abs(x1 - x0);
        int64_t dy = std::abs(y1 - y0);
        int64_t sx = (x0 < x1) ? 1 : -1;
        int64_t sy = (y0 < y1) ? 1 : -1;
        int64_t err = dx - dy;

        while (true) {
            int64_t index = y0 * map_width + x0;
            if (index >= 0 && index < static_cast<int64_t>(map_height * map_width)) {
                raytraced_cells.push_back(index);
            }

            if (x0 == x1 && y0 == y1) {
                break;
            }

            int64_t e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dx) {
                err += dx;
                y0 += sy;
            }
        }
    }

    std::vector<int64_t> rayTrace(nav2_costmap_2d::Costmap2D& costmap,
    unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1) {
        std::vector<int64_t> raytraced_cells;
        rayTraceGeneric(costmap.getSizeInCellsX(), costmap.getSizeInCellsY(), 
                        x0, y0, x1, y1, raytraced_cells);
        return raytraced_cells;
    }

    std::vector<unsigned int> getPointsInSemiCircle(unsigned int center_x, 
        unsigned int center_y, unsigned int radius_in_cells, unsigned int numPoints, double robot_yaw, nav2_costmap_2d::Costmap2D& costmap)
    {
        std::vector<unsigned int> all_cells;
        for (unsigned int i = 0; i < numPoints; ++i)
        {
            double angle = 2.0 * M_PI * i / numPoints;
            int64_t x = static_cast<int64_t>(center_x + radius_in_cells * std::cos(robot_yaw - M_PI_2 + angle));
            int64_t y = static_cast<int64_t>(center_y + radius_in_cells * std::sin(robot_yaw - M_PI_2 + angle));
            cellEnforceBoundaries(x, y, costmap);
            nav2_costmap_2d::MapLocation point;
            point.x = static_cast<unsigned int>(x);
            point.y = static_cast<unsigned int>(y);

            auto traced_points = rayTrace(costmap, center_x, center_y, point.x, point.y);
            for (auto traced_point : traced_points) {
                all_cells.push_back(static_cast<unsigned int>(traced_point));
            }
        }

        return all_cells;
    }

    std::vector<unsigned int> getPointsInIsoscelesTriangle(
        unsigned int apex_x,
        unsigned int apex_y,
        unsigned int triangle_height,   // distance from apex to base along the median
        unsigned int numPoints,         // number of rays to sample along the base
        double direction,               // direction from the apex toward the base center (in radians)
        double apex_angle,              // the angle at the apex (in radians)
        nav2_costmap_2d::Costmap2D& costmap)
    {
        std::vector<unsigned int> all_cells;
    
        // Compute the center of the base using the triangle height and the given direction.
        double base_center_x = apex_x + triangle_height * std::cos(direction);
        double base_center_y = apex_y + triangle_height * std::sin(direction);
    
        // For an isosceles triangle, half the base length can be computed using tan(apex_angle/2)
        double half_base = triangle_height * std::tan(apex_angle / 2.0);
    
        // Compute the endpoints of the base by moving perpendicular to the direction.
        // Adding/subtracting a perpendicular vector rotates the base by 90 degrees.
        double left_endpoint_x = base_center_x + half_base * std::cos(direction + M_PI_2);
        double left_endpoint_y = base_center_y + half_base * std::sin(direction + M_PI_2);
        double right_endpoint_x = base_center_x + half_base * std::cos(direction - M_PI_2);
        double right_endpoint_y = base_center_y + half_base * std::sin(direction - M_PI_2);
    
        // Sample numPoints evenly along the base (from left endpoint to right endpoint).
        // If numPoints is 1, we simply use the left endpoint (or base center) to avoid division by zero.
        for (unsigned int i = 0; i < numPoints; ++i)
        {
            double t = (numPoints > 1) ? static_cast<double>(i) / (numPoints - 1) : 0.0;
            double sample_x = left_endpoint_x + t * (right_endpoint_x - left_endpoint_x);
            double sample_y = left_endpoint_y + t * (right_endpoint_y - left_endpoint_y);
    
            // Enforce boundaries on the sampled point.
            int64_t cell_x = static_cast<int64_t>(sample_x);
            int64_t cell_y = static_cast<int64_t>(sample_y);
            cellEnforceBoundaries(cell_x, cell_y, costmap);
    
            // Create a MapLocation for the sampled point.
            nav2_costmap_2d::MapLocation sample_point;
            sample_point.x = static_cast<unsigned int>(cell_x);
            sample_point.y = static_cast<unsigned int>(cell_y);
    
            // Ray-trace from the apex to the sampled point.
            auto traced_points = rayTrace(costmap, apex_x, apex_y, sample_point.x, sample_point.y);
            for (auto traced_point : traced_points)
            {
                all_cells.push_back(static_cast<unsigned int>(traced_point));
            }
        }
    
        return all_cells;
    }

    using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    using nav2_costmap_2d::LETHAL_OBSTACLE;
    using nav2_costmap_2d::NO_INFORMATION;
    LethalMarker::LethalMarker()
        : need_recalculation_(false)
    {
        numAdditions_ = 0;
    }

    void LethalMarker::onInitialize()
    {
        std::cout << "LethalMarker::onInitialize";
        node = node_.lock();
        if (!node)
        {
            throw std::runtime_error{"Failed to lock node"};
            return;
        }

        ///-------------------------------ROS Params------------------------------------------------------------
        /// Declare Parameters (parameter_name, default value)

        declareParameter("enabled", rclcpp::ParameterValue(true));
        node->get_parameter(name_ + "." + "enabled", enabled_);

        dyn_params_handler_ = node->add_on_set_parameters_callback(
            std::bind(&LethalMarker::dynamicParametersCallback,
                      this, std::placeholders::_1));
        matchSize();

        layer_node_ = rclcpp::Node::make_shared("lethal_marker_internal_node");
        layer_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        layer_executor_->add_node(layer_node_);
        service_callback_group_ = layer_node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        edit_lethal_zone_server_ = layer_node_->create_service<fit_slam2_msgs::srv::MarkLethal>(
            name_ + "/mark_lethal_zone", std::bind(&LethalMarker::editLethalZoneService, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_default, service_callback_group_);
        std::thread t1(&LethalMarker::threadFunction, this, layer_executor_);
        t1.detach();

        std::cout << "LethalMarker onInitialize complete.";
    }

    void LethalMarker::editLethalZoneService(const std::shared_ptr<fit_slam2_msgs::srv::MarkLethal::Request> req,
                                          const std::shared_ptr<fit_slam2_msgs::srv::MarkLethal::Response> res)
    {
        addNewMarkedAreaFOV(req->lethal_point.x, req->lethal_point.y, req->yaw, 3.5);
        res->success = true;
    }

    void LethalMarker::onFootprintChanged()
    {
        std::cout << "LethalMarker::onFootprintChanged";
        need_recalculation_ = true;
    }

    void LethalMarker::matchSize()
    {
        std::cout << "LethalMarker::matchSize()";
        nav2_costmap_2d::Costmap2D *master = layered_costmap_->getCostmap();
        // make current false so that layer.hpp from nav2_costmap_2d treats this layer's values as old.
        // This is done in case recomputing the cache upon bounds change takes longer than expected.
        current_ = false;
        std::lock_guard<std::mutex> lock(cacheMutex_);
        current_ = true;    
        latest_cells_to_mark_index_.clear();
        numAdditions_ = 0;

        for (auto & spec : zone_specs_) {
            addNewMarkedAreaFOV(spec.wx, spec.wy, spec.yaw, spec.height);
        }
    }

    void LethalMarker::addNewMarkedAreaFOV(double center_wx, double center_wy, double angleYaw, double height)
    {
        zone_specs_.push_back({center_wx, center_wy, angleYaw, height});
        unsigned int robot_xm, robot_ym;
        if(!layered_costmap_->getCostmap()->worldToMap(center_wx, center_wy, robot_xm, robot_ym))
            return;
        auto height_in_cells = height / layered_costmap_->getCostmap()->getResolution();
        latest_cells_to_mark_index_[numAdditions_] = getPointsInIsoscelesTriangle(robot_xm, robot_ym, height_in_cells, 20, angleYaw, 45 * M_PI / 180, *layered_costmap_->getCostmap());
        numAdditions_++;
    }

    void LethalMarker::markCells(unsigned char *master_array, const std::vector<unsigned int>& cells_to_mark)
    {
        for (auto index : cells_to_mark)
        {
            master_array[index] = 253;
        }
    }

    rcl_interfaces::msg::SetParametersResult LethalMarker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        std::lock_guard<std::mutex> lock_reinit(parameter_mutex_);

        for (auto parameter : parameters)
        {
            const auto &type = parameter.get_type();
            const auto &name = parameter.get_name();

            if (type == rclcpp::ParameterType::PARAMETER_BOOL)
            {
                if (name == name_ + "." + "enabled" && enabled_ != parameter.as_bool())
                {
                    enabled_ = parameter.as_bool();
                    need_recalculation_ = true;
                    current_ = false;
                }
            }
        }

        std::cout << "Keepout Zone Maker parameter update triggered.";
        result.successful = true;
        return result;
    }

    void LethalMarker::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x,
                                    double *min_y, double *max_x, double *max_y)
    {
        if (need_recalculation_)
        {
            last_min_x_ = *min_x;
            last_min_y_ = *min_y;
            last_max_x_ = *max_x;
            last_max_y_ = *max_y;

            *min_x = std::numeric_limits<double>::lowest();
            *min_y = std::numeric_limits<double>::lowest();
            *max_x = std::numeric_limits<double>::max();
            *max_y = std::numeric_limits<double>::max();
            need_recalculation_ = false;
        }
        else
        {
            double tmp_min_x = last_min_x_;
            double tmp_min_y = last_min_y_;
            double tmp_max_x = last_max_x_;
            double tmp_max_y = last_max_y_;
            last_min_x_ = *min_x;
            last_min_y_ = *min_y;
            last_max_x_ = *max_x;
            last_max_y_ = *max_y;
            *min_x = std::min(tmp_min_x, *min_x);
            *min_y = std::min(tmp_min_y, *min_y);
            *max_x = std::max(tmp_max_x, *max_x);
            *max_y = std::max(tmp_max_y, *max_y);
        }
    }

    void LethalMarker::updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j,
                                   int max_i, int max_j)
    {
        current_ = true;
        if (!enabled_)
        {
            std::cout << "Costs returned because not enabled.";
            return;
        }

        unsigned char *master_array = master_grid.getCharMap();
        unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

        for(int add = 0; add < numAdditions_; add++)
        {
            if(getIndexCache().count(add) > 0)
                LethalMarker::markCells(master_array, getIndexCache().at(add));
            else
                std::cout << "Cells cache is empty for " << add << ", call to add points to map.";
        }
    }

} // namespace nav2_gradient_costmap_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fit_slam2_nav2_plugins::LethalMarker, nav2_costmap_2d::Layer)
