#include <traversability_gridmap/traversability_nav_layer.hpp>
#include <chrono>

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace traversability_gridmap
{
    TraversabilityNavLayer::TraversabilityNavLayer() {}

    TraversabilityNavLayer::~TraversabilityNavLayer()
    {
        dyn_params_handler_.reset();
    }

    void TraversabilityNavLayer::onInitialize()
    {
        auto node = node_.lock();
        if (!node)
        {
            throw std::runtime_error{"Failed to lock node traversability_nav_layer"};
        }
        
        
        declareParameter("square_half_side_length", rclcpp::ParameterValue(10.0));
        node->get_parameter(name_ + "." + "square_half_side_length", squareHalfSideLength_);

        declareParameter("traversability_lethal_threshold", rclcpp::ParameterValue(176));
        node->get_parameter(name_ + "." + "traversability_lethal_threshold", traversability_lethal_threshold_);

        costmap_sub_ = node->create_subscription<nav2_msgs::msg::Costmap>(
        "/traversability_costmap/costmap_raw",
        rclcpp::QoS(rclcpp::KeepLast(1)),
        std::bind(&TraversabilityNavLayer::costmapCallback, this, std::placeholders::_1));

        layered_costmap_->resizeMap((2 * squareHalfSideLength_) / resolution_, (2 * squareHalfSideLength_) / resolution_, layered_costmap_->getCostmap()->getResolution(), -squareHalfSideLength_, -squareHalfSideLength_);
        Costmap2D *master = layered_costmap_->getCostmap();
        resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
                    master->getOriginX(), master->getOriginY());

        dyn_params_handler_ = node->add_on_set_parameters_callback(
        std::bind(
            &TraversabilityNavLayer::dynamicParametersCallback,
            this, std::placeholders::_1));
    }

    rcl_interfaces::msg::SetParametersResult TraversabilityNavLayer::dynamicParametersCallback(
        std::vector<rclcpp::Parameter> parameters)
    {
        std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
        rcl_interfaces::msg::SetParametersResult result;

        for (auto parameter : parameters) {
            std::string param_name = parameter.get_name();
            rclcpp::ParameterType param_type = parameter.get_type();

            if (param_type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                if (param_name == "square_half_side_length") {
                    squareHalfSideLength_ = parameter.as_double();
                }
            }
            else if(param_type == rclcpp::ParameterType::PARAMETER_INTEGER) {
                if (param_name == "traversability_lethal_threshold") {
                    traversability_lethal_threshold_ = static_cast<unsigned char>(parameter.as_int());
                }
            }
        }
        
        result.successful = true;
        return result;
    }


    void TraversabilityNavLayer::costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
    {

        // RCLCPP_INFO(rclcpp::get_logger("traversability_nav_layer"), "Callback!");
        // Access costmap data from the message and update the layer's costs accordingly
        // Example: Update costs in a loop over the data array
        latest_costmap_ = *msg;
        Costmap2D * master = layered_costmap_->getCostmap();
        if (!layered_costmap_->isRolling() && (master->getSizeInCellsX() != latest_costmap_.metadata.size_x ||
            master->getSizeInCellsY() != latest_costmap_.metadata.size_y ||
            master->getResolution() != latest_costmap_.metadata.resolution ||
            master->getOriginX() != latest_costmap_.metadata.origin.position.x ||
            master->getOriginY() != latest_costmap_.metadata.origin.position.y ||
            !layered_costmap_->isSizeLocked()))
        {
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("your_logger_name"),
            // "Resizing map with parameters - "
            // << "SizeX: " << master->getSizeInCellsX()
            // << ", SizeY: " << master->getSizeInCellsY()
            // << ", Resolution: " << master->getResolution()
            // << ", OriginX: " << master->getOriginX()
            // << ", OriginY: " << master->getOriginY()
            // << "Size lock" << layered_costmap_->isSizeLocked());

        // RCLCPP_INFO_STREAM(
        //     rclcpp::get_logger("traversability_nav_layer"),
        //     "Callback1! - Resizing map: size_x = " << latest_costmap_.metadata.size_x <<
        //     ", size_y = " << latest_costmap_.metadata.size_y <<
        //     ", resolution = " << latest_costmap_.metadata.resolution <<
        //     ", origin_x = " << latest_costmap_.metadata.origin.position.x <<
        //     ", origin_y = " << latest_costmap_.metadata.origin.position.y);

            layered_costmap_->resizeMap(
            latest_costmap_.metadata.size_x, latest_costmap_.metadata.size_y, latest_costmap_.metadata.resolution,
            latest_costmap_.metadata.origin.position.x,
            latest_costmap_.metadata.origin.position.y,
            true);
        }
        std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
        // only update the size of the costmap stored locally in this layer
        resizeMap(
        latest_costmap_.metadata.size_x, latest_costmap_.metadata.size_y, latest_costmap_.metadata.resolution,
        latest_costmap_.metadata.origin.position.x, latest_costmap_.metadata.origin.position.y);

        unsigned int index = 0;

        // we have a new map, update full size of map

        // initialize the costmap with traversability data
        for (unsigned int i = 0; i < master->getSizeInCellsY(); ++i) {
            for (unsigned int j = 0; j < master->getSizeInCellsX(); ++j) {
                unsigned char value = latest_costmap_.data[index];
                // RCLCPP_INFO_STREAM(rclcpp::get_logger("traversability_nav_layer"), "Master index: " <<  index << "costmap index value" << static_cast<double>(value));

                costmap_[index] = interpretBinValue(value);
                ++index;
            }
        }
    }

    unsigned char TraversabilityNavLayer::interpretBinValue(unsigned char value) {
        if(value == NO_INFORMATION) {
            return NO_INFORMATION;
        }
        else if(value >= traversability_lethal_threshold_) {
            return LETHAL_OBSTACLE;
        }
        else {
            return FREE_SPACE;
        }
    }

    void TraversabilityNavLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x, double *max_y)
    {
        *min_x = -300.0;
        *min_y = -300.0;
        *max_x = 300.0;
        *max_y = 300.0;
        RCLCPP_DEBUG_STREAM(
            rclcpp::get_logger("traversability_nav_layer"),
            "updateBounds: robot_x = " << robot_x << ", robot_y = " << robot_y << ", robot_yaw = " << robot_yaw << ", min_x = " << *min_x << ", min_y = " << *min_y << ", max_x = " << *max_x << ", max_y = " << *max_y << "Resolution = " << resolution_;);
    }


    void TraversabilityNavLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
    {
        // RCLCPP_ERROR_STREAM(logger_, "Lethal threshold: " << static_cast<int>(traversability_lethal_threshold_));
        // RCLCPP_ERROR_STREAM(logger_, "square threshold: " << static_cast<int>(squareHalfSideLength_));
        RCLCPP_DEBUG_STREAM(
        rclcpp::get_logger("traversability_nav_layer"),
        "updateCosts: "
        << ", min_i = " << min_i << ", min_j = " << min_j << ", max_i = " << max_i << ", max_j = " << max_j << "Resolution = " << resolution_;);

        std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
        unsigned char * master = master_grid.getCharMap();
        unsigned int span = master_grid.getSizeInCellsX();

        for (int j = min_j; j < max_j; j++) {
            unsigned int it = span * j + min_i;
            for (int i = min_i; i < max_i; i++) {
                if (costmap_[it] != NO_INFORMATION) {
                    master[it] = costmap_[it];
                }
            it++;
            }
        }
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("traversability_nav_layer"), "The size in x and y after update is: " << master_grid.getSizeInCellsX() << ", " << master_grid.getSizeInCellsY());


    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(traversability_gridmap::TraversabilityNavLayer, nav2_costmap_2d::Layer)