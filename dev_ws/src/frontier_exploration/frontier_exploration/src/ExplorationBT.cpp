#include <frontier_exploration/ExplorationBT.hpp>
#include <frontier_exploration/util/geometry_tools.hpp>

#if defined(FRONTIER_POINT_MEDIAN) + defined(FRONTIER_POINT_INITIAL) > 1
#error "Only one of FRONTIER_POINT_MEDIAN, or FRONTIER_POINT_INITIAL can be defined at a time."
#elif !defined(FRONTIER_POINT_MEDIAN) && !defined(FRONTIER_POINT_INITIAL)
#error "One of FRONTIER_POINT_MEDIAN, or FRONTIER_POINT_INITIAL must be defined."
#endif

namespace frontier_exploration
{
    class InitializationSequence : public BT::StatefulActionNode
    {
    public:
        InitializationSequence(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<InitCommandVelNode> initialization_controller) : BT::StatefulActionNode(name, config)
        {
            initialization_controller_ = initialization_controller;
            LOG_DEBUG("InitializationSequence Constructor");
        }

        BT::NodeStatus onStart() override
        {
            LOG_HIGHLIGHT("MODULE InitializationSequence");
            if (initialization_controller_->send_velocity_commands())
            {
                return BT::NodeStatus::FAILURE;
            }
            else
            {
                return BT::NodeStatus::SUCCESS;
            }
        }

        BT::NodeStatus onRunning() override
        {
        }

        void onHalted()
        {
            return;
        }
        std::shared_ptr<InitCommandVelNode> initialization_controller_;
    };

    class WaitForCurrent : public BT::StatefulActionNode
    {
    public:
        WaitForCurrent(const std::string &name, const BT::NodeConfig &config,
                       std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
                       rclcpp::Node::SharedPtr ros_node_ptr) : BT::StatefulActionNode(name, config)
        {
            explore_costmap_ros_ = explore_costmap_ros;
            ros_node_ptr_ = ros_node_ptr;
            LOG_DEBUG("WaitForCurrentBT Constructor");
        }

        BT::NodeStatus onStart() override
        {
            LOG_HIGHLIGHT("MODULE WaitForCurrent");
            return BT::NodeStatus::RUNNING;
        }

        BT::NodeStatus onRunning() override
        {
            if (!explore_costmap_ros_->isCurrent())
            {
                LOG_DEBUG("Waiting for costmap to be current");
                return BT::NodeStatus::RUNNING;
            }
            LOG_DEBUG("CostMap is current.");
            return BT::NodeStatus::SUCCESS;
        }

        void onHalted()
        {
            return;
        }

        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
        rclcpp::Node::SharedPtr ros_node_ptr_;
    };

    class UpdateBoundaryPolygonBT : public BT::StatefulActionNode
    {
    public:
        UpdateBoundaryPolygonBT(const std::string &name, const BT::NodeConfig &config,
                                std::shared_ptr<CostAssigner> bel_ptr, std::vector<std::string> boundary_config,
                                rclcpp::Node::SharedPtr ros_node_ptr) : BT::StatefulActionNode(name, config)
        {
            bel_ptr_ = bel_ptr;
            config_ = boundary_config;
            ros_node_ptr_ = ros_node_ptr;
            LOG_DEBUG("UpdateBoundaryPolygonBT Constructor");
        }

        BT::NodeStatus onStart() override
        {
            LOG_HIGHLIGHT("MODULE UpdateBoundaryPolygonBT");
            geometry_msgs::msg::PolygonStamped explore_boundary_;
            geometry_msgs::msg::PointStamped explore_center_;
            explore_boundary_.header.frame_id = "map";
            explore_boundary_.header.stamp = rclcpp::Clock().now();
            for (int i = 0; i < config_.size(); i += 2)
            {
                geometry_msgs::msg::Point32 point;
                point.x = std::stof(config_[i]);
                point.y = std::stof(config_[i + 1]);
                explore_boundary_.polygon.points.push_back(point);
            }

            explore_center_.header.frame_id = "map";
            explore_center_.point.x = 5.5;
            explore_center_.point.y = 5.5;

            auto updateBoundaryResult = bel_ptr_->updateBoundaryPolygon(explore_boundary_);
            LOG_DEBUG("Adding update boundary polygon for spin.");
            if (updateBoundaryResult == true)
            {
                LOG_INFO("Region boundary set");
            }
            else
            {
                LOG_ERROR("Failed to receive response for updateBoundaryPolygon");
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus onRunning()
        {
            LOG_DEBUG("UpdateBoundaryPolygonBT On running called ");
            return BT::NodeStatus::SUCCESS;
        }

        void onHalted()
        {
            return;
        }

        static BT::PortsList providedPorts()
        {
            return {};
        }

        std::shared_ptr<CostAssigner> bel_ptr_;
        rclcpp::Node::SharedPtr ros_node_ptr_;
        std::vector<std::string> config_;
    };

    class SearchForFrontiersBT : public BT::StatefulActionNode
    {
    public:
        SearchForFrontiersBT(const std::string &name, const BT::NodeConfig &config,
                             std::shared_ptr<FrontierSearch> frontierSearchPtr, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
                             rclcpp::Node::SharedPtr ros_node_ptr) : BT::StatefulActionNode(name, config)
        {
            explore_costmap_ros_ = explore_costmap_ros;
            frontierSearchPtr_ = frontierSearchPtr;
            ros_node_ptr_ = ros_node_ptr;
            LOG_DEBUG("SearchForFrontiersBT Constructor");
        }

        BT::NodeStatus onStart() override
        {
            LOG_HIGHLIGHT("MODULE SearchForFrontiersBT");
            eventLoggerInstance.startEvent("SearchForF rontiers", 0);
            frontierSearchPtr_->reset();
            explore_costmap_ros_->getCostmap()->getMutex()->lock();
            LOG_DEBUG("SearchForFrontiersBT OnStart called ");
            geometry_msgs::msg::PoseStamped robotP;
            explore_costmap_ros_->getRobotPose(robotP);
            config().blackboard->set<geometry_msgs::msg::PoseStamped>("latest_robot_pose", robotP);
            auto frontier_list = frontierSearchPtr_->searchFrom(robotP.pose.position);
            auto every_frontier = frontierSearchPtr_->getAllFrontiers();
            if (frontier_list.size() == 0)
            {
                double increment_value = 0.1;
                getInput("increment_search_distance_by", increment_value);
                frontierSearchPtr_->incrementSearchDistance(increment_value);
                LOG_WARN("No frontiers found in search. Incrementing search radius and returning BT Failure.")
                eventLoggerInstance.endEvent("SearchForFrontiers", 0);
                explore_costmap_ros_->getCostmap()->getMutex()->unlock();
                return BT::NodeStatus::FAILURE;
            }
            LOG_INFO("Recieved " << frontier_list.size() << " frontiers");
            setOutput("frontier_list", frontier_list);
            setOutput("every_frontier", every_frontier);
            RosVisualizer::getInstance().visualizeFrontier(frontier_list, every_frontier, explore_costmap_ros_->getLayeredCostmap()->getGlobalFrameID());
            frontierSearchPtr_->resetSearchDistance();
            eventLoggerInstance.endEvent("SearchForFrontiers", 0);
            explore_costmap_ros_->getCostmap()->getMutex()->unlock();
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus onRunning()
        {
            LOG_INFO("SearchForFrontiersBT On running called ");
            return BT::NodeStatus::SUCCESS;
        }

        void onHalted()
        {
            return;
        }

        static BT::PortsList providedPorts()
        {
            return {
                BT::OutputPort<std::vector<Frontier>>("frontier_list"),
                BT::OutputPort<std::vector<std::vector<double>>>("every_frontier"),
                BT::InputPort<double>("increment_search_distance_by"),
            };
        }

        std::shared_ptr<FrontierSearch> frontierSearchPtr_;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
        rclcpp::Node::SharedPtr ros_node_ptr_;
    };

    class UpdateRoadmapBT : public BT::StatefulActionNode
    {
    public:
        UpdateRoadmapBT(const std::string &name, const BT::NodeConfig &config,
                        rclcpp::Node::SharedPtr ros_node_ptr, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros) : BT::StatefulActionNode(name, config)
        {
            explore_costmap_ros_ = explore_costmap_ros;
            LOG_INFO("UpdateRoadmapBT Constructor");
        }

        BT::NodeStatus onStart() override
        {
            eventLoggerInstance.startEvent("UpdateRoadmapBT", 0);
            LOG_HIGHLIGHT("MODULE UpdateRoadmapBT");
            std::vector<Frontier> frontier_list;
            getInput<std::vector<Frontier>>("frontier_list", frontier_list);
            geometry_msgs::msg::PoseStamped robotP;
            config().blackboard->get<geometry_msgs::msg::PoseStamped>("latest_robot_pose", robotP);
            FrontierRoadMap::getInstance().addNodes(frontier_list, true);
            bool addPose;
            getInput("add_robot_pose_to_roadmap", addPose);
            if (addPose)
            {
                LOG_HIGHLIGHT("Adding robot pose as frontier node.");
                FrontierRoadMap::getInstance().addRobotPoseAsNode(robotP.pose, false);
            }
            eventLoggerInstance.startEvent("roadmapReconstruction", 1);
            FrontierRoadMap::getInstance().constructNewEdges(frontier_list);
            FrontierRoadMap::getInstance().constructNewEdgeRobotPose(robotP.pose);
            // FrontierRoadMap::getInstance().reConstructGraph();
            eventLoggerInstance.endEvent("roadmapReconstruction", 1);

            eventLoggerInstance.startEvent("publishRoadmap", 2);
            FrontierRoadMap::getInstance().publishRoadMap();
            eventLoggerInstance.endEvent("publishRoadmap", 2);
            // TODO: make sure to add a thing such that the entire roadmap within a certain distance (max frontier search distance) is reconstructed periodically
            eventLoggerInstance.endEvent("UpdateRoadmapBT", 0);
            // FrontierRoadMap::getInstance().countTotalItemsInSpatialMap();
            // TODO: remove below line
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus onRunning()
        {
            LOG_INFO("UpdateRoadmapBT On running called ");
            return BT::NodeStatus::SUCCESS;
        }

        void onHalted()
        {
            return;
        }

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::vector<Frontier>>("frontier_list"),
                BT::InputPort<bool>("add_robot_pose_to_roadmap")};
        }

        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
    };

    class CleanupRoadMapBT : public BT::StatefulActionNode
    {
    public:
        CleanupRoadMapBT(const std::string &name, const BT::NodeConfig &config,
                         rclcpp::Node::SharedPtr ros_node_ptr, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
                         std::shared_ptr<FullPathOptimizer> full_path_optimizer) : BT::StatefulActionNode(name, config)
        {
            full_path_optimizer_ = full_path_optimizer;
            explore_costmap_ros_ = explore_costmap_ros;
            LOG_INFO("CleanupRoadMapBT Constructor");
        }

        BT::NodeStatus onStart() override
        {
            LOG_HIGHLIGHT("MODULE CleanupRoadMapBT");
            LOG_INFO("Time since last clearance: " << eventLoggerInstance.getTimeSinceStart("clearRoadmap", 0));
            double time_between_cleanup;
            getInput("time_between_cleanup", time_between_cleanup);
            if (eventLoggerInstance.getTimeSinceStart("clearRoadmap", 0) < time_between_cleanup)
            {
                return BT::NodeStatus::FAILURE;
            }
            eventLoggerInstance.startEvent("clearRoadmap", 0);
            eventLoggerInstance.startEvent("CleanupRoadMapBT", 0);
            eventLoggerInstance.startEvent("roadmapReconstructionFull", 1);
            FrontierRoadMap::getInstance().reConstructGraph();
            eventLoggerInstance.endEvent("roadmapReconstructionFull", 1);
            full_path_optimizer_->clearPlanCache();
            // TODO: make sure to add a thing such that the entire roadmap within a certain distance (max frontier search distance) is reconstructed periodically
            eventLoggerInstance.endEvent("CleanupRoadMapBT", 0);
            // FrontierRoadMap::getInstance().countTotalItemsInSpatialMap();
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus onRunning()
        {
            LOG_INFO("CleanupRoadMapBT On running called ");
            return BT::NodeStatus::SUCCESS;
        }

        void onHalted()
        {
            return;
        }

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::vector<Frontier>>("frontier_list"),
                BT::InputPort<double>("time_between_cleanup")};
        }

        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
        std::shared_ptr<FullPathOptimizer> full_path_optimizer_;
    };

    class VerifyFrontierSearchBT : public BT::StatefulActionNode
    {
    public:
        VerifyFrontierSearchBT(const std::string &name, const BT::NodeConfig &config,
                               std::shared_ptr<FrontierSearch> frontierSearchPtr, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
                               rclcpp::Node::SharedPtr ros_node_ptr) : BT::StatefulActionNode(name, config)
        {
            explore_costmap_ros_ = explore_costmap_ros;
            ros_node_ptr_ = ros_node_ptr;
            LOG_INFO("VerifyFrontierSearchBT Constructor");
        }

        BT::NodeStatus onStart() override
        {
            LOG_INFO("VerifyFrontierSearchBT OnStart called ");
            std::vector<Frontier> frontier_list;
            getInput<std::vector<Frontier>>("frontier_list", frontier_list);
            if (verifyFrontierList(frontier_list, explore_costmap_ros_->getCostmap()))
                return BT::NodeStatus::SUCCESS;
            return BT::NodeStatus::FAILURE;
        }

        BT::NodeStatus onRunning()
        {
            LOG_INFO("VerifyFrontierSearchBT On running called ");
            return BT::NodeStatus::SUCCESS;
        }

        void onHalted()
        {
            return;
        }

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::vector<Frontier>>("frontier_list")};
        }

        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
        rclcpp::Node::SharedPtr ros_node_ptr_;
    };

    class ProcessFrontierCostsBT : public BT::StatefulActionNode
    {
    public:
        ProcessFrontierCostsBT(const std::string &name, const BT::NodeConfig &config,
                               std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
                               std::shared_ptr<CostAssigner> bel_ptr,
                               RobotActiveGoals &robot_goals,
                               std::shared_ptr<TaskAllocator> taskAllocator,
                               rclcpp::Node::SharedPtr ros_node_ptr)
            : BT::StatefulActionNode(name, config),
              robot_goals_(robot_goals)
        {
            explore_costmap_ros_ = explore_costmap_ros;
            bel_ptr_ = bel_ptr;
            taskAllocator_ = taskAllocator;
            ros_node_ptr_ = ros_node_ptr;
            LOG_INFO("ProcessFrontierCostsBT Constructor");
        }

        BT::NodeStatus onStart() override
        {
            eventLoggerInstance.startEvent("ProcessFrontierCosts", 0);
            LOG_HIGHLIGHT("MODULE ProcessFrontierCostsBT");
            taskAllocator_->reset();
            auto frontierCostsRequestPtr = std::make_shared<GetFrontierCostsRequest>();
            auto frontierCostsResultPtr = std::make_shared<GetFrontierCostsResponse>();
            std::string robot_name;

            if (!getInput<std::vector<Frontier>>("frontier_list", frontierCostsRequestPtr->frontier_list))
            {
                BT::RuntimeError("No correct input recieved for frontier list");
            }
            if (!getInput<std::vector<std::vector<double>>>("every_frontier", frontierCostsRequestPtr->every_frontier))
            {
                BT::RuntimeError("No correct input recieved for every_frontier");
            }
            if (!getInput<std::string>("robot_name", robot_name))
            {
                BT::RuntimeError("No correct input recieved for every_frontier");
            }

            config().blackboard->get<geometry_msgs::msg::PoseStamped>("latest_robot_pose", frontierCostsRequestPtr->start_pose);
            config().blackboard->get<std::vector<Frontier>>("frontier_blacklist", frontierCostsRequestPtr->prohibited_frontiers);
            LOG_INFO("Request to get frontier costs sent");
            bool frontierCostsSuccess = bel_ptr_->getFrontierCosts(frontierCostsRequestPtr, frontierCostsResultPtr);
            if (frontierCostsSuccess == false)
            {
                LOG_INFO("Failed to receive response for getNextFrontier called from within the robot.");
                eventLoggerInstance.endEvent("ProcessFrontierCosts", 0);
                return BT::NodeStatus::FAILURE;
            }
            setOutput("frontier_costs_result", frontierCostsResultPtr->frontier_list);
            taskAllocator_->addRobotTasks(frontierCostsResultPtr->frontier_costs, frontierCostsResultPtr->frontier_distances, robot_name);

            bel_ptr_->logMapData(frontierCostsRequestPtr);
            eventLoggerInstance.endEvent("ProcessFrontierCosts", 0);
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus onRunning()
        {
            return BT::NodeStatus::SUCCESS;
        }

        void onHalted()
        {
            return;
        }

        static BT::PortsList providedPorts()
        {
            return {BT::InputPort<std::vector<Frontier>>("frontier_list"),
                    BT::InputPort<std::string>("robot_name"),
                    BT::InputPort<std::vector<std::vector<double>>>("every_frontier"),
                    BT::OutputPort<std::vector<Frontier>>("frontier_costs_result")};
        }

        std::shared_ptr<FrontierSearch> frontierSearchPtr_;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
        std::shared_ptr<CostAssigner> bel_ptr_;
        RobotActiveGoals &robot_goals_;
        std::shared_ptr<TaskAllocator> taskAllocator_;
        rclcpp::Node::SharedPtr ros_node_ptr_;
    };

    class OptimizeFullPath : public BT::StatefulActionNode
    {
    public:
        OptimizeFullPath(const std::string &name, const BT::NodeConfig &config,
                         std::shared_ptr<FullPathOptimizer> full_path_optimizer,
                         std::shared_ptr<CostAssigner> bel_ptr,
                         std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
                         std::shared_ptr<FrontierSearch> frontierSearchPtr,
                         std::shared_ptr<Nav2Interface> nav2_interface,
                         std::shared_ptr<RecoveryController> recovery_controller,
                         rclcpp::Node::SharedPtr node) : BT::StatefulActionNode(name, config)
        {
            explore_costmap_ros_ = explore_costmap_ros;
            full_path_optimizer_ = full_path_optimizer;
            frontierSearchPtr_ = frontierSearchPtr;
            nav2_interface_ = nav2_interface;
            recovery_controller_ = recovery_controller;
            currentFIRetries = 0;
            node_ = node;
            LOG_INFO("OptimizeFullPath Constructor");
        }

        void updateLethalZone(geometry_msgs::msg::Point point)
        {
            auto lethal_zone_srv_client_ = node_->create_client<frontier_msgs::srv::MarkLethal>("lethal_marker_global_costmap/mark_lethal_zone");
            RCLCPP_WARN(node_->get_logger(), "Waiting indefinitely for service to become available");
            if (!lethal_zone_srv_client_->wait_for_service())
            {
                RCLCPP_ERROR_STREAM(node_->get_logger(), "mark lethal service not available after waiting");
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(), "Got service");
            }

            if (!lethal_zone_srv_client_->service_is_ready())
            {
                RCLCPP_ERROR_STREAM(node_->get_logger(), "mark lethal service not ready");
                return;
            }
            // create service request
            auto lethal_zone_srv_request = std::make_shared<frontier_msgs::srv::MarkLethal::Request>();
            lethal_zone_srv_request->radius = 1.7;
            lethal_zone_srv_request->lethal_point = point;
            auto lethal_zone_srv_result = lethal_zone_srv_client_->async_send_request(lethal_zone_srv_request);
            if (lethal_zone_srv_result.wait_for(std::chrono::seconds(20)) == std::future_status::ready)
            {
                auto lethal_zone_srv_response = lethal_zone_srv_result.get();
                if (!lethal_zone_srv_response->success)
                {
                    RCLCPP_WARN(node_->get_logger(), "lethal zone response failure.");
                }
                else
                {
                    RCLCPP_WARN(node_->get_logger(), "SRV SUCCESS!");
                }
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "Failed to call the service");
            }
        }

        BT::NodeStatus onStart() override
        {
            eventLoggerInstance.startEvent("OptimizeFullPath", 0);
            LOG_HIGHLIGHT("MODULE OptimizeFullPath");
            double numberRetriesFI;
            getInput("number_retries_fi", numberRetriesFI);
            std::vector<Frontier> globalFrontierList;
            getInput<std::vector<Frontier>>("frontier_costs_result", globalFrontierList);
            bool use_fi;
            getInput<bool>("use_fisher_information", use_fi);
            geometry_msgs::msg::PoseStamped robotP;
            config().blackboard->get<geometry_msgs::msg::PoseStamped>("latest_robot_pose", robotP);
            Frontier allocatedFrontier;
            if(currentFIRetries == numberRetriesFI - 1 && use_fi)
                full_path_optimizer_->setExhaustiveSearch(true);
            auto return_state_with_fi = full_path_optimizer_->getNextGoal(globalFrontierList, allocatedFrontier, 3, robotP, use_fi);
            if (return_state_with_fi == PathSafetyStatus::SAFE)
            {
                currentFIRetries = 0;
                frontierSearchPtr_->resetSearchDistance();
                LOG_WARN("The next goal that will be sent to Nav2 is:" << allocatedFrontier);
                eventLoggerInstance.endEvent("OptimizeFullPath", 0);
                setOutput<Frontier>("allocated_frontier", allocatedFrontier);
            }
            else if (return_state_with_fi == PathSafetyStatus::UNDETERMINED)
            {
                nav2_interface_->cancelAllGoals();
                double increment_value = 0.1;
                getInput("increment_search_distance_by", increment_value);
                frontierSearchPtr_->incrementSearchDistance(increment_value);
                recovery_controller_->computeVelocityCommand(false);
                full_path_optimizer_->setExhaustiveSearch(false);
                return BT::NodeStatus::FAILURE;
            }
            else if (return_state_with_fi == PathSafetyStatus::UNSAFE)
            {
                nav2_interface_->cancelAllGoals();
                ++currentFIRetries;
                LOG_WARN("Current FI retry number: " << currentFIRetries << " max retries: " << numberRetriesFI);
                // geometry_msgs::msg::Pose relative_pose;
                // getRelativePoseGivenTwoPoints(robotP.pose.position, allocatedFrontier.getGoalPoint(), relative_pose);
                // if(!recovery_controller_->alignWithPose(relative_pose))
                //     return BT::NodeStatus::FAILURE;
                rclcpp::sleep_for(std::chrono::seconds(3));
                if (currentFIRetries >= numberRetriesFI)
                {
                    auto robotYaw = quatToEuler(robotP.pose.orientation)[2];
                    float blacklist_x = robotP.pose.position.x + (2.5 * cos(robotYaw));
                    float blacklist_y = robotP.pose.position.y + (2.5 * sin(robotYaw));
                    Frontier blacklistedFrontier;
                    blacklistedFrontier.setGoalPoint(blacklist_x, blacklist_y);
                    full_path_optimizer_->blacklistFrontier(blacklistedFrontier);
                    full_path_optimizer_->publishBlacklistCircles();
                    FrontierRoadMap::getInstance().addFrontierToBlacklist(blacklistedFrontier);

                    recovery_controller_->computeVelocityCommand(true);
                    updateLethalZone(blacklistedFrontier.getGoalPoint());
                    auto plugins = explore_costmap_ros_->getLayeredCostmap()->getPlugins();
                    for (auto plugin : *plugins)
                    {
                        if (plugin->getName() == "lethal_marker")
                        {
                            auto lethal_plugin = std::dynamic_pointer_cast<frontier_exploration::LethalMarker>(plugin);
                            lethal_plugin->addNewMarkedArea(blacklist_x, blacklist_y, 1.7);
                        }
                    }
                }
                full_path_optimizer_->setExhaustiveSearch(false);
                return BT::NodeStatus::FAILURE;
            }
            frontierSearchPtr_->resetSearchDistance();
            LOG_WARN("The next goal that will be sent to Nav2 is:" << allocatedFrontier);
            eventLoggerInstance.endEvent("OptimizeFullPath", 0);
            setOutput<Frontier>("allocated_frontier", allocatedFrontier);
            full_path_optimizer_->setExhaustiveSearch(false);
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus onRunning()
        {
            return BT::NodeStatus::SUCCESS;
        }

        void onHalted()
        {
            return;
        }

        static BT::PortsList providedPorts()
        {
            return {BT::InputPort<std::vector<Frontier>>("frontier_costs_result"),
                    BT::InputPort<bool>("use_fisher_information"),
                    BT::OutputPort<Frontier>("allocated_frontier"),
                    BT::InputPort<double>("increment_search_distance_by"),
                    BT::InputPort<double>("number_retries_fi")};
        }

        std::shared_ptr<FrontierSearch> frontierSearchPtr_;
        std::shared_ptr<FullPathOptimizer> full_path_optimizer_;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
        std::shared_ptr<Nav2Interface> nav2_interface_;
        std::shared_ptr<RecoveryController> recovery_controller_;
        rclcpp::Node::SharedPtr node_;
        double currentFIRetries;
    };

    class HysterisisControl : public BT::StatefulActionNode
    {
    public:
        HysterisisControl(const std::string &name, const BT::NodeConfig &config,
                          std::shared_ptr<std::vector<Frontier>> active_hysterisis_candidates) : BT::StatefulActionNode(name, config)
        {
            hysterisis_candidates_ = active_hysterisis_candidates;
            LOG_INFO("HysterisisControl Constructor");
            minDistance = std::numeric_limits<double>::max();
            eventLoggerInstance.startEvent("startHysterisis", 1);
        }

        BT::NodeStatus onStart() override
        {
            eventLoggerInstance.startEvent("HysterisisControl", 0);
            if (eventLoggerInstance.getTimeSinceStart("startHysterisis", 1) > 60.0)
            {
                eventLoggerInstance.endEvent("startHysterisis", 1);
                eventLoggerInstance.startEvent("startHysterisis", 1);
                hysterisis_candidates_->clear();
                minDistance = std::numeric_limits<double>::max();
            }
            LOG_HIGHLIGHT("MODULE HysterisisControl");
            LOG_HIGHLIGHT("MODULE HysterisisControl Elements in picture: " << hysterisis_candidates_->size());
            bool enabled_;
            config().blackboard->get<bool>("hysterisis_enabled", enabled_);
            if (enabled_)
            {
                Frontier allocatedFrontier;
                getInput<Frontier>("allocated_frontier", allocatedFrontier);
                hysterisis_candidates_->push_back(allocatedFrontier);

                // // Use a map to count the occurrences of each frontier
                // std::unordered_map<Frontier, int, FrontierHash> frequencyMap;
                // for (const auto &frontier : *hysterisis_candidates_)
                // {
                //     if (frequencyMap.count(frontier) > 0)
                //     {
                //         if (frontier.isAchievable() == false)
                //         {
                //             frequencyMap[frontier] = -3;
                //             continue;
                //         }
                //         if (frequencyMap[frontier] <= 3)
                //             frequencyMap[frontier]++;
                //     }
                //     else
                //         frequencyMap[frontier] = 1;
                // }

                // // Find the frontier with the highest count
                // Frontier mostFrequentFrontier = allocatedFrontier;
                // int maxCount = 0;
                // for (const auto &[frontier, count] : frequencyMap)
                // {
                //     LOG_INFO("Frontier in hysterisis: " << frontier << " with count: " << count);
                //     if (count > maxCount)
                //     {
                //         mostFrequentFrontier = frontier;
                //         maxCount = count;
                //     }
                // }

                geometry_msgs::msg::PoseStamped robotP;
                config().blackboard->get<geometry_msgs::msg::PoseStamped>("latest_robot_pose", robotP);
                if (distanceBetweenPoints(allocatedFrontier.getGoalPoint(), robotP.pose.position) < minDistance - 2.0)
                {
                    LOG_WARN("Found a closer one: " << minDistance);
                    minDistance = distanceBetweenPoints(allocatedFrontier.getGoalPoint(), robotP.pose.position);
                    mostFrequentFrontier = allocatedFrontier;
                }

                // Set the most frequent frontier as the output
                setOutput<Frontier>("allocated_frontier_after_hysterisis", mostFrequentFrontier);
            }
            else
            {
                hysterisis_candidates_->clear();
                minDistance = std::numeric_limits<double>::max();
                Frontier allocatedFrontier;
                getInput<Frontier>("allocated_frontier", allocatedFrontier);
                hysterisis_candidates_->push_back(allocatedFrontier);
                setOutput<Frontier>("allocated_frontier_after_hysterisis", allocatedFrontier);
            }
            eventLoggerInstance.endEvent("HysterisisControl", 0);
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus onRunning()
        {
            return BT::NodeStatus::SUCCESS;
        }

        void onHalted()
        {
            return;
        }

        static BT::PortsList providedPorts()
        {
            return {BT::InputPort<Frontier>("allocated_frontier"),
                    BT::OutputPort<Frontier>("allocated_frontier_after_hysterisis")};
        }

        std::shared_ptr<std::vector<Frontier>> hysterisis_candidates_;
        double minDistance;
        Frontier mostFrequentFrontier;
    };

    class ExecuteRecoveryMove : public BT::StatefulActionNode
    {
    public:
        ExecuteRecoveryMove(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<RecoveryController> recovery_controller) : BT::StatefulActionNode(name, config)
        {
            recovery_controller_bt_ = recovery_controller;
            LOG_INFO("ExecuteRecoveryMove Constructor");
        }

        BT::NodeStatus onStart() override
        {
            eventLoggerInstance.startEvent("ExecuteRecoveryMove", 0);
            LOG_HIGHLIGHT("MODULE ExecuteRecoveryMove");
            if (!recovery_controller_bt_)
                throw std::runtime_error("recovery ptr is null.");
            bool backwardOnly;
            getInput<bool>("backward_only", backwardOnly);
            recovery_controller_bt_->computeVelocityCommand(backwardOnly);
            eventLoggerInstance.endEvent("ExecuteRecoveryMove", 0);
            return BT::NodeStatus::FAILURE;
        }

        BT::NodeStatus onRunning()
        {
            return BT::NodeStatus::SUCCESS;
        }

        void onHalted()
        {
            return;
        }

        static BT::PortsList providedPorts()
        {
            return {BT::InputPort<bool>("backward_only")};
        }

        std::shared_ptr<RecoveryController> recovery_controller_bt_;
    };

    class GetAllocatedGoalBT : public BT::StatefulActionNode
    {
    public:
        GetAllocatedGoalBT(const std::string &name, const BT::NodeConfig &config,
                           std::shared_ptr<TaskAllocator> taskAllocator,
                           rclcpp::Node::SharedPtr ros_node_ptr,
                           std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros) : BT::StatefulActionNode(name, config)
        {
            taskAllocator_ = taskAllocator;
            ros_node_ptr_ = ros_node_ptr;
            explore_costmap_ros_ = explore_costmap_ros;
            LOG_INFO("!!!!! NODE NAME! " << ros_node_ptr_->get_namespace());
            LOG_INFO("GetAllocatedGoalBT Constructor");
        }

        BT::NodeStatus onStart() override
        {
            LOG_INFO("GetAllocatedGoalBT OnStart called ");
            taskAllocator_->solveAllocationHungarian();
            auto allocatedIndex = taskAllocator_->getAllocatedTasks()[ros_node_ptr_->get_namespace()];
            std::vector<Frontier> globalFrontierList;
            getInput<std::vector<Frontier>>("frontier_costs_result", globalFrontierList);
            auto allocatedFrontier = globalFrontierList[allocatedIndex];
            LOG_INFO("Allocated frontier x:" + std::to_string(allocatedFrontier.getGoalPoint().x));
            LOG_INFO("Allocated frontier y:" + std::to_string(allocatedFrontier.getGoalPoint().y));
            LOG_INFO("Allocated frontier z:" + std::to_string(allocatedFrontier.getGoalPoint().z));
            LOG_INFO("Allocated frontier oz:" + std::to_string(allocatedFrontier.getGoalOrientation().z));
            LOG_INFO("Allocated frontier ow:" + std::to_string(allocatedFrontier.getGoalOrientation().w));
            LOG_INFO("Allocated frontier ox:" + std::to_string(allocatedFrontier.getGoalOrientation().x));
            LOG_INFO("Allocated frontier oy:" + std::to_string(allocatedFrontier.getGoalOrientation().y));
            LOG_INFO("Allocated frontier uid:" + std::to_string(allocatedFrontier.getUID()));
            setOutput<Frontier>("allocated_frontier", allocatedFrontier);
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus onRunning()
        {
            return BT::NodeStatus::SUCCESS;
        }

        void onHalted()
        {
            return;
        }

        static BT::PortsList providedPorts()
        {
            return {BT::InputPort<std::vector<Frontier>>("frontier_costs_result"),
                    BT::OutputPort<Frontier>("allocated_frontier")};
        }

        std::shared_ptr<TaskAllocator> taskAllocator_;
        rclcpp::Node::SharedPtr ros_node_ptr_;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
    };

    class SendNav2Goal : public BT::StatefulActionNode
    {
    public:
        SendNav2Goal(const std::string &name, const BT::NodeConfig &config,
                     std::shared_ptr<Nav2Interface> nav2_interface,
                     rclcpp::Node::SharedPtr ros_node_ptr) : BT::StatefulActionNode(name, config)
        {
            nav2_interface_ = nav2_interface;
            ros_node_ptr_ = ros_node_ptr;
            latestAllocationFailures_ = 0;
            LOG_INFO("SendNav2Goal Constructor");
        }

        BT::NodeStatus onStart() override
        {
            LOG_INFO("SendNav2Goal onStart");
            Frontier allocatedFrontier;
            latestAllocation_ = allocatedFrontier;
            getInput("allocated_frontier", allocatedFrontier);
            LOG_INFO("Allocated frontier x:" + std::to_string(allocatedFrontier.getGoalPoint().x));
            LOG_INFO("Allocated frontier y:" + std::to_string(allocatedFrontier.getGoalPoint().y));
            LOG_INFO("Allocated frontier z:" + std::to_string(allocatedFrontier.getGoalPoint().z));
            LOG_INFO("Allocated frontier oz:" + std::to_string(allocatedFrontier.getGoalOrientation().z));
            LOG_INFO("Allocated frontier ow:" + std::to_string(allocatedFrontier.getGoalOrientation().w));
            LOG_INFO("Allocated frontier ox:" + std::to_string(allocatedFrontier.getGoalOrientation().x));
            LOG_INFO("Allocated frontier oy:" + std::to_string(allocatedFrontier.getGoalOrientation().y));
            LOG_INFO("Allocated frontier uid:" + std::to_string(allocatedFrontier.getUID()));
            geometry_msgs::msg::PoseStamped goalPose;
            goalPose.header.frame_id = "map";
            goalPose.pose.position = allocatedFrontier.getGoalPoint();
            goalPose.pose.orientation = allocatedFrontier.getGoalOrientation();
            nav2_interface_->sendGoal(goalPose);
            return BT::NodeStatus::RUNNING;
        }

        BT::NodeStatus onRunning()
        {
            LOG_INFO("SendNav2Goal onRunning");
            if (nav2_interface_->goalStatus() == 0)
                return BT::NodeStatus::RUNNING;
            else if (nav2_interface_->goalStatus() == 1)
            {
                latestAllocationFailures_ = 0;
                return BT::NodeStatus::SUCCESS;
            }
            else if (nav2_interface_->goalStatus() == -1)
            {
                latestAllocationFailures_++;
                if (latestAllocationFailures_ > 4)
                {
                    latestAllocationFailures_ = 0;
                    latestAllocation_.setBlacklisted(true);
                }
                return BT::NodeStatus::FAILURE;
            }
            else
                throw BT::RuntimeError("Unknown goal status");
            return BT::NodeStatus::SUCCESS;
        }

        void onHalted()
        {
            return;
        }

        static BT::PortsList providedPorts()
        {
            return {BT::InputPort<Frontier>("allocated_frontier")};
        }

        std::shared_ptr<Nav2Interface> nav2_interface_;
        std::shared_ptr<TaskAllocator> taskAllocator_;
        rclcpp::Node::SharedPtr ros_node_ptr_;
        Frontier latestAllocation_;
        int latestAllocationFailures_;
    };

    class CheckIfGoalMapped : public BT::StatefulActionNode
    {
    public:
        CheckIfGoalMapped(const std::string &name, const BT::NodeConfig &config,
                          std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
                          rclcpp::Node::SharedPtr ros_node_ptr) : BT::StatefulActionNode(name, config)
        {
            explore_costmap_ros_ = explore_costmap_ros;
            ros_node_ptr_ = ros_node_ptr;
            LOG_INFO("CheckIfGoalMapped Constructor");
        }

        BT::NodeStatus onStart() override
        {
            LOG_INFO("CheckIfGoalMapped onStart");
            Frontier allocatedFrontier;
            getInput("allocated_frontier", allocatedFrontier);
            // LOG_INFO("Allocated frontier x:" + std::to_string(allocatedFrontier.getGoalPoint().x));
            // LOG_INFO("Allocated frontier y:" + std::to_string(allocatedFrontier.getGoalPoint().y));
            // LOG_INFO("Allocated frontier z:" + std::to_string(allocatedFrontier.getGoalPoint().z));
            // LOG_INFO("Allocated frontier oz:" + std::to_string(allocatedFrontier.getGoalOrientation().z));
            // LOG_INFO("Allocated frontier ow:" + std::to_string(allocatedFrontier.getGoalOrientation().w));
            // LOG_INFO("Allocated frontier ox:" + std::to_string(allocatedFrontier.getGoalOrientation().x));
            // LOG_INFO("Allocated frontier oy:" + std::to_string(allocatedFrontier.getGoalOrientation().y));
            // LOG_INFO("Allocated frontier uid:" + std::to_string(allocatedFrontier.getUID()));
            geometry_msgs::msg::PoseStamped goalPose;
            goalPose.header.frame_id = "map";
            goalPose.pose.position = allocatedFrontier.getGoalPoint();
            if (surroundingCellsMapped(goalPose.pose.position, *explore_costmap_ros_->getCostmap()))
            {
                config().blackboard->set<bool>("hysterisis_enabled", false);
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::FAILURE;
        }

        BT::NodeStatus onRunning()
        {
            LOG_INFO("CheckIfGoalMapped onRunning");
            Frontier allocatedFrontier;
            getInput("allocated_frontier", allocatedFrontier);
            geometry_msgs::msg::PoseStamped goalPose;
            goalPose.header.frame_id = "map";
            goalPose.pose.position = allocatedFrontier.getGoalPoint();
            if (surroundingCellsMapped(goalPose.pose.position, *explore_costmap_ros_->getCostmap()))
            {
                config().blackboard->set<bool>("hysterisis_enabled", false);
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::RUNNING;
        }

        void onHalted()
        {
            return;
        }

        static BT::PortsList providedPorts()
        {
            return {BT::InputPort<Frontier>("allocated_frontier")};
        }

        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
        rclcpp::Node::SharedPtr ros_node_ptr_;
    };

    class ReplanTimeoutCompleteBT : public BT::StatefulActionNode
    {
    public:
        ReplanTimeoutCompleteBT(const std::string &name, const BT::NodeConfig &config,
                                std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
                                rclcpp::Node::SharedPtr ros_node_ptr) : BT::StatefulActionNode(name, config)
        {
            explore_costmap_ros_ = explore_costmap_ros;
            ros_node_ptr_ = ros_node_ptr;
            LOG_INFO("ReplanTimeoutCompleteBT Constructor");
        }

        BT::NodeStatus onStart() override
        {
            double timeoutValue;
            getInput("timeout_value", timeoutValue);
            LOG_INFO("ReplanTimeoutCompleteBT onStart");
            if (eventLoggerInstance.getTimeSinceStart("replanTimeout", 0) > timeoutValue)
            {
                LOG_WARN("Replanning timed out. Restarting the frontier computation");
                eventLoggerInstance.startEvent("replanTimeout", 0);
                config().blackboard->set<bool>("hysterisis_enabled", true);
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::FAILURE;
        }

        BT::NodeStatus onRunning()
        {
            return BT::NodeStatus::SUCCESS;
        }

        void onHalted()
        {
            return;
        }

        static BT::PortsList providedPorts()
        {
            return {BT::InputPort<Frontier>("allocated_frontier"), BT::InputPort<double>("timeout_value")};
        }

        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
        rclcpp::Node::SharedPtr ros_node_ptr_;
    };

    class RecoveryMoveBack : public BT::StatefulActionNode
    {
    public:
        RecoveryMoveBack(const std::string &name, const BT::NodeConfig &config,
                         std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
                         rclcpp::Node::SharedPtr ros_node_ptr) : BT::StatefulActionNode(name, config)
        {
            explore_costmap_ros_ = explore_costmap_ros;
            ros_node_ptr_ = ros_node_ptr;
            cmd_vel_publisher_ = ros_node_ptr->create_publisher<geometry_msgs::msg::Twist>("/robot_0/cmd_vel_nav", 10);
            LOG_INFO("RecoveryMoveBack Constructor");
        }

        BT::NodeStatus onStart() override
        {
            LOG_INFO("RecoveryMoveBack onStart");
            startTime = std::chrono::high_resolution_clock::now();
            return BT::NodeStatus::RUNNING;
        }

        BT::NodeStatus onRunning()
        {
            LOG_INFO("RecoveryMoveBack onRunning");
            auto endTime = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> duration = endTime - startTime;
            double maxDuration;
            getInput("move_back_duration", maxDuration);
            if (duration.count() > maxDuration)
            {
                geometry_msgs::msg::Twist twist_msg;
                cmd_vel_publisher_->publish(twist_msg);
                cmd_vel_publisher_->publish(twist_msg);
                cmd_vel_publisher_->publish(twist_msg);
                return BT::NodeStatus::SUCCESS;
            }
            geometry_msgs::msg::Twist twist_msg;
            twist_msg.linear.x = -0.5; // -0.5 m/s for reverse motion
            cmd_vel_publisher_->publish(twist_msg);
            return BT::NodeStatus::RUNNING;
        }

        void onHalted()
        {
            return;
        }

        static BT::PortsList providedPorts()
        {
            return {BT::InputPort<double>("move_back_duration")};
        }

        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
        rclcpp::Node::SharedPtr ros_node_ptr_;
        std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    };
}

namespace frontier_exploration
{
    FrontierExplorationServer::FrontierExplorationServer(rclcpp::Node::SharedPtr node)
    {
        LOG_TRACE("First BT constructor");
        bt_node_ = node;
        blackboard = BT::Blackboard::create();
        blackboard->set<bool>("hysterisis_enabled", false);
        active_hysterisis_candidates_ = std::make_shared<std::vector<Frontier>>();

        bt_node_->declare_parameter("retry_count", 30);
        bt_node_->declare_parameter("nav2_goal_timeout_sec", 35);
        bt_node_->declare_parameter("use_custom_sim", false);
        robot_namespaces_ = {};
        bt_node_->declare_parameter("robot_namespaces", rclcpp::ParameterValue(robot_namespaces_));
        config_ = {"100.0", "100.0", "100.0", "-100.0", "-100.0", "-100.0", "-100.0", "100.0"};
        bt_node_->declare_parameter("config", rclcpp::ParameterValue(config_));
        bt_node_->declare_parameter("min_frontier_cluster_size", rclcpp::ParameterValue(1));
        bt_node_->declare_parameter("max_frontier_cluster_size", rclcpp::ParameterValue(20));
        bt_node_->declare_parameter("max_frontier_distance", rclcpp::ParameterValue(0.5));
        bt_node_->declare_parameter("process_other_robots", rclcpp::ParameterValue(false));
        bt_node_->declare_parameter("bt_xml_path", "/root/dev_ws/src/frontier_exploration/frontier_exploration/xml/exploration.xml");

        bt_node_->get_parameter("retry_count", retry_);
        bt_node_->get_parameter("nav2_goal_timeout_sec", nav2WaitTime_);
        bt_node_->get_parameter("use_custom_sim", use_custom_sim_);
        bt_node_->get_parameter("robot_namespaces", robot_namespaces_);
        bt_node_->get_parameter("config", config_);
        bt_node_->get_parameter("min_frontier_cluster_size", min_frontier_cluster_size_);
        bt_node_->get_parameter("max_frontier_cluster_size", max_frontier_cluster_size_);
        bt_node_->get_parameter("max_frontier_distance", max_frontier_distance_);
        bt_node_->get_parameter("process_other_robots", process_other_robots_);
        bt_node_->get_parameter("bt_xml_path", bt_xml_path_);
        LOG_TRACE("Declared BT params");
        //--------------------------------------------NAV2 CLIENT RELATED-------------------------------
        nav2_interface_ = std::make_shared<Nav2Interface>(bt_node_);
        LOG_TRACE("Created Nav2 interface instance");

        //--------------------------------------------EXPLORE SERVER RELATED----------------------------

        explore_costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("explore_costmap", std::string{bt_node_->get_namespace()}, "explore_costmap");
        explore_costmap_ros_->configure();
        // Launch a thread to run the costmap node
        explore_costmap_thread_ = std::make_unique<nav2_util::NodeThread>(explore_costmap_ros_);
        explore_costmap_ros_->activate();
        LOG_TRACE("Created costmap instance");

        recovery_controller_ = std::make_shared<RecoveryController>(explore_costmap_ros_, bt_node_);
        initialization_controller_ = std::make_shared<InitCommandVelNode>();

        //------------------------------------------BOUNDED EXPLORE LAYER RELATED------------------------
        RosVisualizer::createInstance(bt_node_, explore_costmap_ros_->getCostmap());
        FrontierRoadMap::createInstance(explore_costmap_ros_);
        LOG_TRACE("Created ros visualizer instance");
        bel_ptr_ = std::make_shared<CostAssigner>(explore_costmap_ros_);
        task_allocator_ptr_ = std::make_shared<TaskAllocator>();

        multirobot_service_callback_group_ = bt_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        service_send_current_goal_ = bt_node_->create_service<frontier_msgs::srv::SendCurrentGoal>(
            "multirobot_send_current_goal", std::bind(&FrontierExplorationServer::handle_multirobot_current_goal_request, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
            rmw_qos_profile_default, multirobot_service_callback_group_);

        frontierSearchPtr_ = std::make_shared<FrontierSearch>(*(explore_costmap_ros_->getLayeredCostmap()->getCostmap()), min_frontier_cluster_size_, max_frontier_cluster_size_, max_frontier_distance_);
        full_path_optimizer_ = std::make_shared<FullPathOptimizer>(bt_node_, explore_costmap_ros_);
        //---------------------------------------------ROS RELATED------------------------------------------
        tf_listener_ = std::make_shared<tf2_ros::Buffer>(bt_node_->get_clock());
        LOG_INFO("FrontierExplorationServer::FrontierExplorationServer()");
    }

    FrontierExplorationServer::~FrontierExplorationServer()
    {
        LOG_INFO("FrontierExplorationServer::~FrontierExplorationServer()");
        explore_costmap_ros_->deactivate();
        explore_costmap_ros_->cleanup();
        explore_costmap_thread_.reset();
    }

    void FrontierExplorationServer::makeBTNodes()
    {
        eventLoggerInstance.startEvent("clearRoadmap", 0);
        eventLoggerInstance.startEvent("replanTimeout", 0);

        BT::NodeBuilder builder_initialization =
            [&](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<InitializationSequence>(name, config, initialization_controller_);
        };
        factory.registerBuilder<WaitForCurrent>("InitializationSequence", builder_initialization);

        BT::NodeBuilder builder_wait_for_current =
            [&](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<WaitForCurrent>(name, config, explore_costmap_ros_, bt_node_);
        };
        factory.registerBuilder<WaitForCurrent>("WaitForCurrent", builder_wait_for_current);

        BT::NodeBuilder builder_update_boundary_polygon =
            [&](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<UpdateBoundaryPolygonBT>(name, config, bel_ptr_, config_, bt_node_);
        };
        factory.registerBuilder<UpdateBoundaryPolygonBT>("UpdateBoundaryPolygon", builder_update_boundary_polygon);

        BT::NodeBuilder builder_frontier_search =
            [&](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<SearchForFrontiersBT>(name, config, frontierSearchPtr_, explore_costmap_ros_, bt_node_);
        };
        factory.registerBuilder<SearchForFrontiersBT>("SearchForFrontiers", builder_frontier_search);

        BT::NodeBuilder builder_frontier_search_verification =
            [&](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<VerifyFrontierSearchBT>(name, config, frontierSearchPtr_, explore_costmap_ros_, bt_node_);
        };
        factory.registerBuilder<VerifyFrontierSearchBT>("VerifyFrontierSearch", builder_frontier_search);

        BT::NodeBuilder builder_update_roadmap_data =
            [&](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<UpdateRoadmapBT>(name, config, bt_node_, explore_costmap_ros_);
        };
        factory.registerBuilder<UpdateRoadmapBT>("UpdateFrontierRoadmap", builder_update_roadmap_data);

        BT::NodeBuilder builder_cleanup_roadmap_data =
            [&](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<CleanupRoadMapBT>(name, config, bt_node_, explore_costmap_ros_, full_path_optimizer_);
        };
        factory.registerBuilder<CleanupRoadMapBT>("CleanupRoadmap", builder_cleanup_roadmap_data);

        BT::NodeBuilder builder_frontier_costs =
            [&](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<ProcessFrontierCostsBT>(name, config, explore_costmap_ros_, bel_ptr_, robot_active_goals_, task_allocator_ptr_, bt_node_);
        };
        factory.registerBuilder<ProcessFrontierCostsBT>("ProcessFrontierCosts", builder_frontier_costs);

        BT::NodeBuilder builder_full_path_optimizer =
            [&](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<OptimizeFullPath>(name, config, full_path_optimizer_, bel_ptr_, explore_costmap_ros_, frontierSearchPtr_, nav2_interface_, recovery_controller_, bt_node_);
        };
        factory.registerBuilder<OptimizeFullPath>("OptimizeFullPath", builder_full_path_optimizer);

        BT::NodeBuilder builder_hystersis_control =
            [&](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<HysterisisControl>(name, config, active_hysterisis_candidates_);
        };
        factory.registerBuilder<HysterisisControl>("HysterisisControl", builder_hystersis_control);

        BT::NodeBuilder builder_task_allocator =
            [&](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<GetAllocatedGoalBT>(name, config, task_allocator_ptr_, bt_node_, explore_costmap_ros_);
        };
        factory.registerBuilder<GetAllocatedGoalBT>("GetAllocatedGoal", builder_task_allocator);

        BT::NodeBuilder builder_send_goal =
            [&](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<SendNav2Goal>(name, config, nav2_interface_, bt_node_);
        };
        factory.registerBuilder<SendNav2Goal>("SendNav2Goal", builder_send_goal);

        BT::NodeBuilder builder_goal_mapped =
            [&](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<CheckIfGoalMapped>(name, config, explore_costmap_ros_, bt_node_);
        };
        factory.registerBuilder<CheckIfGoalMapped>("CheckIfGoalMapped", builder_goal_mapped);

        BT::NodeBuilder builder_replan_timeout =
            [&](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<ReplanTimeoutCompleteBT>(name, config, explore_costmap_ros_, bt_node_);
        };
        factory.registerBuilder<ReplanTimeoutCompleteBT>("ReplanTimeoutComplete", builder_replan_timeout);

        BT::NodeBuilder builder_recovery_back =
            [&](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<RecoveryMoveBack>(name, config, explore_costmap_ros_, bt_node_);
        };
        factory.registerBuilder<RecoveryMoveBack>("RecoveryMoveBack", builder_recovery_back);

        BT::NodeBuilder execute_recovery_move =
            [&](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<ExecuteRecoveryMove>(name, config, recovery_controller_);
        };
        factory.registerBuilder<ExecuteRecoveryMove>("ExecuteRecoveryMove", execute_recovery_move);

        behaviour_tree = factory.createTreeFromFile(bt_xml_path_, blackboard);
        while (rclcpp::ok())
        {
            behaviour_tree.tickOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            LOG_DEBUG("TICKED ONCE");
        }
    }

    void FrontierExplorationServer::handle_multirobot_current_goal_request(
        std::shared_ptr<rmw_request_id_t> request_header,
        std::shared_ptr<frontier_msgs::srv::SendCurrentGoal::Request> request,
        std::shared_ptr<frontier_msgs::srv::SendCurrentGoal::Response> response)
    {
        std::lock_guard<std::mutex> lock(robot_active_goals_.mutex);
        LOG_ERROR("Goal updated for: " + request->sending_robot_name + " New goal? " + std::to_string(request->goal_state));
        if (request->goal_state == 1)
            robot_active_goals_.goals[request->sending_robot_name] = std::make_shared<geometry_msgs::msg::PoseStamped>(request->current_goal);
        else if (request->goal_state == 0)
            robot_active_goals_.goals[request->sending_robot_name] = nullptr;

        response->success = true;
        return;
    }

};