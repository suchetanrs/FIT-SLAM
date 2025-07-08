#include "fisher_information_plugins/fisher_information/FisherInfoBTPlugin.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace roadmap_explorer
{
    class FisherInformationBT : public BT::SyncActionNode
    {
    public:
        FisherInformationBT(const std::string &name, const BT::NodeConfiguration &config,
                         std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
                         std::shared_ptr<nav2_util::LifecycleNode> node, std::shared_ptr<tf2_ros::Buffer> tf_buffer) : BT::SyncActionNode(name, config)
        {
            explore_costmap_ros_ = explore_costmap_ros;
            node_ = node;
            fisher_info_manager_ = std::make_shared<FisherInformationManager>(node_);
            tf_buffer_ = tf_buffer;
            // nav2_util::declare_parameter_if_not_declared(
            //     node, "fisherInformation.fisher_information_threshold", rclcpp::ParameterValue(2.0));
            // double fisher_information_threshold = node->get_parameter("fisherInformation.fisher_information_threshold").as_double();
            parameterInstance.setValue<double>("fisherInformation.fisher_information_threshold", 550.0);
            LOG_INFO("FisherInformationBT Constructor");
        }

        BT::NodeStatus tick() override
        {
            geometry_msgs::msg::PoseStamped robotP;
            geometry_msgs::msg::PoseStamped robotP3D;
            if (!config().blackboard->get<geometry_msgs::msg::PoseStamped>("latest_robot_pose", robotP))
            {
                // Handle the case when "latest_robot_pose" is not found
                LOG_FATAL("Failed to retrieve latest_robot_pose from blackboard.");
                throw std::runtime_error("Failed to retrieve latest_robot_pose from blackboard.");
            }
            geometry_msgs::msg::TransformStamped transform_stamped =
                tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
            robotP3D.pose.position.x = transform_stamped.transform.translation.x;
            robotP3D.pose.position.y = transform_stamped.transform.translation.y;
            robotP3D.pose.position.z = transform_stamped.transform.translation.z;
            robotP3D.pose.orientation = transform_stamped.transform.rotation;
            
            bool exhaustive_landmark_search_;
            getInput<bool>("exhaustive_landmark_search", exhaustive_landmark_search_);
            if (exhaustive_landmark_search_)
            {
                LOG_WARN("Exhaustive landmark search is enabled for this iteration");
            }
            float information;
            bool safety_value = fisher_info_manager_->isPoseSafe(robotP3D.pose, exhaustive_landmark_search_, information);
            if (safety_value)
            {
                return BT::NodeStatus::SUCCESS;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(700));
            config().blackboard->set<ExplorationErrorCode>(
                "error_code_id", ExplorationErrorCode::NO_ERROR);
            return BT::NodeStatus::FAILURE;
        }

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<bool>("exhaustive_landmark_search")
            };
        }

        std::shared_ptr<FisherInformationManager> fisher_info_manager_;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
        std::shared_ptr<nav2_util::LifecycleNode> node_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    };

    class MarkLethalFOV : public BT::SyncActionNode
    {
        public:
        MarkLethalFOV(const std::string &name, const BT::NodeConfiguration &config,
                     std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
                     std::shared_ptr<nav2_util::LifecycleNode> node) : BT::SyncActionNode(name, config)
        {
            explore_costmap_ros_ = explore_costmap_ros;
            node_ = node;
            blacklisted_poses_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseArray>("posearray_blacklisted_region", 10);
            blacklisted_poses_publisher_->on_activate();
        }

        void publishBlacklistPoses()
        {
            poseBlacklists_.header.stamp = node_->now();
            poseBlacklists_.header.frame_id = "map";
            blacklisted_poses_publisher_->publish(poseBlacklists_);
        }

        void blacklistFrontier(geometry_msgs::msg::PoseStamped pose, FrontierPtr& frontier)
        {
            auto robotYaw = quatToEuler(pose.pose.orientation)[2];
            pose.pose.position = frontier->getGoalPoint();
            pose.pose.position.x += (1.7 * cos(robotYaw));
            pose.pose.position.y += (1.7 * sin(robotYaw));
            robotYaw += M_PI;
            auto quat = eulerToQuat(0, 0, robotYaw);
            pose.pose.orientation = quat;
            poseBlacklists_.poses.push_back(pose.pose);
        }

        void updateLethalZone(double x, double y, double robotYaw, std::string service_name)
        {
            auto lethal_zone_srv_client_ = node_->create_client<fit_slam2_msgs::srv::MarkLethal>(service_name);
            LOG_DEBUG("Waiting indefinitely for service to become available");
            if (!lethal_zone_srv_client_->wait_for_service())
            {
                LOG_WARN("mark lethal service not available after waiting");
            }
            else
            {
                LOG_DEBUG("Got service");
            }

            if (!lethal_zone_srv_client_->service_is_ready())
            {
                LOG_ERROR("mark lethal service not ready");
                return;
            }
            // create service request
            auto lethal_zone_srv_request = std::make_shared<fit_slam2_msgs::srv::MarkLethal::Request>();
            lethal_zone_srv_request->radius = 1.7;
            lethal_zone_srv_request->lethal_point.x = x;
            lethal_zone_srv_request->lethal_point.y = y;
            lethal_zone_srv_request->yaw = robotYaw;
            auto lethal_zone_srv_result = lethal_zone_srv_client_->async_send_request(lethal_zone_srv_request);
            if (lethal_zone_srv_result.wait_for(std::chrono::seconds(20)) == std::future_status::ready)
            {
                auto lethal_zone_srv_response = lethal_zone_srv_result.get();
                if (!lethal_zone_srv_response->success)
                {
                    LOG_ERROR("lethal zone response failure.");
                }
                else
                {
                    LOG_INFO("Marked lethal zone successfully");
                }
            }
            else
            {
                LOG_ERROR("Failed to call the service");
            }
        }

        BT::NodeStatus tick() override
        {
            geometry_msgs::msg::PoseStamped robotP;
            if (!config().blackboard->get<geometry_msgs::msg::PoseStamped>("latest_robot_pose", robotP))
            {
                // Handle the case when "latest_robot_pose" is not found
                LOG_FATAL("Failed to retrieve latest_robot_pose from blackboard.");
                throw std::runtime_error("Failed to retrieve latest_robot_pose from blackboard.");
            }

            auto robotYaw = quatToEuler(robotP.pose.orientation)[2];
            float blacklist_x = robotP.pose.position.x + (2.5 * cos(robotYaw));
            float blacklist_y = robotP.pose.position.y + (2.5 * sin(robotYaw));

            float blacklist_x_fov = robotP.pose.position.x + (0.8 * cos(robotYaw));
            float blacklist_y_fov = robotP.pose.position.y + (0.8 * sin(robotYaw));

            FrontierPtr blacklistedFrontier = std::make_shared<Frontier>();
            blacklistedFrontier->setGoalPoint(blacklist_x, blacklist_y);
            blacklistFrontier(robotP, blacklistedFrontier);
            publishBlacklistPoses();
            updateLethalZone(blacklist_x_fov, blacklist_y_fov, robotYaw, "lethal_marker_global_costmap/mark_lethal_zone");
            updateLethalZone(blacklist_x_fov, blacklist_y_fov, robotYaw, "lethal_marker_exploration/mark_lethal_zone");
            auto plugins = explore_costmap_ros_->getLayeredCostmap()->getPlugins();
            // for (auto plugin : *plugins)
            // {
            //     if (plugin->getName() == "lethal_marker")
            //     {
            //         auto lethal_plugin = std::dynamic_pointer_cast<fit_slam2_nav2_plugins::LethalMarker>(plugin);
            //         // lethal_plugin->addNewMarkedArea(blacklist_x, blacklist_y, 1.7);
            //         lethal_plugin->addNewMarkedAreaFOV(blacklist_x_fov, blacklist_y_fov, robotYaw, 3.5);
            //     }
            // }
            return BT::NodeStatus::SUCCESS;
        }

        private:
        geometry_msgs::msg::PoseArray poseBlacklists_;
        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>::SharedPtr blacklisted_poses_publisher_;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
        std::shared_ptr<nav2_util::LifecycleNode> node_;
    };

    FisherInfoBTPlugin::FisherInfoBTPlugin()
    {
    }

    FisherInfoBTPlugin::~FisherInfoBTPlugin()
    {
    }

    void FisherInfoBTPlugin::registerNodes(BT::BehaviorTreeFactory &factory, std::shared_ptr<nav2_util::LifecycleNode> node, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros, std::shared_ptr<tf2_ros::Buffer> tf_buffer)
    {
        BT::NodeBuilder builder_log_iteration =
            [explore_costmap_ros, node, tf_buffer](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<FisherInformationBT>(name, config, explore_costmap_ros, node, tf_buffer);
        };
        factory.registerBuilder<FisherInformationBT>("EvaluateFisherInformation", builder_log_iteration);

        BT::NodeBuilder builder_mark_lethal =
            [explore_costmap_ros, node, tf_buffer](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<MarkLethalFOV>(name, config, explore_costmap_ros, node);
        };
        factory.registerBuilder<MarkLethalFOV>("MarkLethalFOV", builder_mark_lethal);
    }
}

PLUGINLIB_EXPORT_CLASS(
    roadmap_explorer::FisherInfoBTPlugin,
    roadmap_explorer::BTPlugin)