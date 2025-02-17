#include <frontier_exploration/ExplorationBT.hpp>

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
            LOG_FLOW("MODULE InitializationSequence");
            eventLoggerInstance.endEvent("NewIteration", -1);
            LOG_HIGHLIGHT("*******NEW SEQUENCE*******");
            eventLoggerInstance.startEvent("NewIteration");
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
            return BT::NodeStatus::RUNNING;
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
            LOG_FLOW("MODULE WaitForCurrent");
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
            LOG_FLOW("MODULE UpdateBoundaryPolygonBT");
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
            LOG_FLOW("MODULE SearchForFrontiersBT");
            eventLoggerInstance.startEvent("SearchForFrontiers");
            frontierSearchPtr_->reset();
            explore_costmap_ros_->getCostmap()->getMutex()->lock();
            LOG_DEBUG("SearchForFrontiersBT OnStart called ");
            geometry_msgs::msg::PoseStamped robotP;
            explore_costmap_ros_->getRobotPose(robotP);
            config().blackboard->set<geometry_msgs::msg::PoseStamped>("latest_robot_pose", robotP);
            LOG_INFO("Using robotP: " << robotP.pose.position.x << ", " << robotP.pose.position.y);
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
            eventLoggerInstance.startEvent("UpdateRoadmapBT");
            LOG_FLOW("MODULE UpdateRoadmapBT");
            std::vector<Frontier> frontier_list;
            getInput<std::vector<Frontier>>("frontier_list", frontier_list);
            geometry_msgs::msg::PoseStamped robotP;
            if (!config().blackboard->get<geometry_msgs::msg::PoseStamped>("latest_robot_pose", robotP))
            {
                // Handle the case when "latest_robot_pose" is not found
                LOG_FATAL("Failed to retrieve latest_robot_pose from blackboard.");
                throw std::runtime_error("Failed to retrieve latest_robot_pose from blackboard.");
            }
            FrontierRoadMap::getInstance().addNodes(frontier_list, true);
            bool addPose;
            getInput("add_robot_pose_to_roadmap", addPose);
            if (addPose)
            {
                LOG_FLOW("Adding robot pose as frontier node.");
                FrontierRoadMap::getInstance().addRobotPoseAsNode(robotP.pose, true);
            }
            eventLoggerInstance.startEvent("roadmapReconstruction");
            FrontierRoadMap::getInstance().constructNewEdges(frontier_list);
            FrontierRoadMap::getInstance().constructNewEdgeRobotPose(robotP.pose);
            // FrontierRoadMap::getInstance().reConstructGraph();
            eventLoggerInstance.endEvent("roadmapReconstruction", 1);

            eventLoggerInstance.startEvent("publishRoadmap");
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
            LOG_FLOW("MODULE CleanupRoadMapBT");
            // LOG_INFO("Time since last clearance: " << eventLoggerInstance.getTimeSinceStart("clearRoadmap"));
            double time_between_cleanup;
            getInput("time_between_cleanup", time_between_cleanup);
            if (eventLoggerInstance.getTimeSinceStart("clearRoadmap") < time_between_cleanup)
            {
                return BT::NodeStatus::FAILURE;
            }
            eventLoggerInstance.startEvent("clearRoadmap");
            eventLoggerInstance.startEvent("CleanupRoadMapBT");
            eventLoggerInstance.startEvent("roadmapReconstructionFull");
            bool correct_loop_closure_;
            getInput("correct_loop_closure", correct_loop_closure_);
            LOG_WARN("Reconstructing roadmap and clearing plan cache!");
            FrontierRoadMap::getInstance().reConstructGraph(true, correct_loop_closure_);
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
                BT::InputPort<bool>("correct_loop_closure"),
                BT::InputPort<double>("time_between_cleanup")};
        }

        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
        std::shared_ptr<FullPathOptimizer> full_path_optimizer_;
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
            eventLoggerInstance.startEvent("ProcessFrontierCosts");
            LOG_FLOW("MODULE ProcessFrontierCostsBT");
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
            if (static_cast<std::string>(ros_node_ptr_->get_namespace()) == "/")
            {
                robot_name = static_cast<std::string>(ros_node_ptr_->get_namespace());
                // robot_name.pop();
                robot_name = robot_name.substr(0, robot_name.size() - 1);
            }
            if (!config().blackboard->get<geometry_msgs::msg::PoseStamped>("latest_robot_pose", frontierCostsRequestPtr->start_pose))
            {
                // Handle the case when "latest_robot_pose" is not found
                LOG_FATAL("Failed to retrieve latest_robot_pose from blackboard.");
                throw std::runtime_error("Failed to retrieve latest_robot_pose from blackboard.");
            }
            // if (!config().blackboard->get<std::vector<Frontier>>("frontier_blacklist", frontierCostsRequestPtr->prohibited_frontiers))
            // {
            //     LOG_FATAL("Failed to retrieve frontier_blacklist from blackboard.");
            //     throw std::runtime_error("Failed to retrieve frontier_blacklist from blackboard.");
            // }
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
            fi_drop_count = 0;
            node_ = node;
            LOG_INFO("OptimizeFullPath Constructor");
        }

        void updateLethalZone(geometry_msgs::msg::Point point)
        {
            auto lethal_zone_srv_client_ = node_->create_client<frontier_msgs::srv::MarkLethal>("lethal_marker_global_costmap/mark_lethal_zone");
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
            auto lethal_zone_srv_request = std::make_shared<frontier_msgs::srv::MarkLethal::Request>();
            lethal_zone_srv_request->radius = 1.7;
            lethal_zone_srv_request->lethal_point = point;
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

        BT::NodeStatus onStart() override
        {
            eventLoggerInstance.startEvent("OptimizeFullPath");
            LOG_FLOW("MODULE OptimizeFullPath");
            double numberRetriesFI;
            getInput("number_retries_fi", numberRetriesFI);
            std::vector<Frontier> globalFrontierList;
            getInput<std::vector<Frontier>>("frontier_costs_result", globalFrontierList);
            bool use_fi;
            getInput<bool>("use_fisher_information", use_fi);
            geometry_msgs::msg::PoseStamped robotP;
            if (!config().blackboard->get<geometry_msgs::msg::PoseStamped>("latest_robot_pose", robotP))
            {
                // Handle the case when "latest_robot_pose" is not found
                LOG_FATAL("Failed to retrieve latest_robot_pose from blackboard.");
                throw std::runtime_error("Failed to retrieve latest_robot_pose from blackboard.");
            }
            Frontier allocatedFrontier;
            if (currentFIRetries == numberRetriesFI - 1 && use_fi)
            {
                LOG_INFO("Setting exhaustive search");
                full_path_optimizer_->setExhaustiveSearch(true);
            }
            auto return_state_with_fi = full_path_optimizer_->getNextGoal(globalFrontierList, allocatedFrontier, 3, robotP, use_fi);
            bool forceBlacklist = false;
            if (fi_drop_count == 2)
            {
                LOG_WARN("Pattern found. Forcing blacklist");
                forceBlacklist = true;
                return_state_with_fi = PathSafetyStatus::UNSAFE;
            }
            if (return_state_with_fi == PathSafetyStatus::SAFE)
            {
                frontierSearchPtr_->resetSearchDistance();
                setOutput<Frontier>("allocated_frontier", allocatedFrontier);
                if (full_path_optimizer_->getExhaustiveSearch())
                {
                    LOG_INFO("Found to be safe with exhaustive search. Incrementing fi_drop_count");
                    currentFIRetries = 0;
                    ++fi_drop_count;
                }
                else
                {
                    fi_drop_count = 0;
                    currentFIRetries = 0;
                }
            }
            else if (return_state_with_fi == PathSafetyStatus::UNDETERMINED)
            {
                nav2_interface_->cancelAllGoals();
                double increment_value = 0.1;
                getInput("increment_search_distance_by", increment_value);
                frontierSearchPtr_->incrementSearchDistance(increment_value);
                recovery_controller_->computeVelocityCommand(false);
                full_path_optimizer_->setExhaustiveSearch(false);
                eventLoggerInstance.endEvent("OptimizeFullPath", 0);
                return BT::NodeStatus::FAILURE;
            }
            else if (return_state_with_fi == PathSafetyStatus::UNSAFE)
            {
                nav2_interface_->cancelAllGoals();
                ++currentFIRetries;
                LOG_WARN("Current FI retry number: " << currentFIRetries << " max retries: " << numberRetriesFI);
                LOG_WARN("FI drop count: " << fi_drop_count);
                // geometry_msgs::msg::Pose relative_pose;
                // getRelativePoseGivenTwoPoints(robotP.pose.position, allocatedFrontier.getGoalPoint(), relative_pose);
                // if(!recovery_controller_->alignWithPose(relative_pose))
                //     return BT::NodeStatus::FAILURE;
                if (currentFIRetries >= numberRetriesFI || forceBlacklist)
                {
                    LOG_ERROR("Marking lethal....");
                    currentFIRetries = 0;
                    fi_drop_count = 0;
                    auto robotYaw = quatToEuler(robotP.pose.orientation)[2];
                    float blacklist_x = robotP.pose.position.x + (2.5 * cos(robotYaw));
                    float blacklist_y = robotP.pose.position.y + (2.5 * sin(robotYaw));
                    Frontier blacklistedFrontier;
                    blacklistedFrontier.setGoalPoint(blacklist_x, blacklist_y);
                    // full_path_optimizer_->blacklistFrontier(blacklistedFrontier);
                    full_path_optimizer_->blacklistFrontier(robotP, blacklistedFrontier);
                    // full_path_optimizer_->publishBlacklistCircles();
                    full_path_optimizer_->publishBlacklistPoses();
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
                rclcpp::sleep_for(std::chrono::seconds(3));
                full_path_optimizer_->setExhaustiveSearch(false);
                eventLoggerInstance.endEvent("OptimizeFullPath", 0);
                return BT::NodeStatus::FAILURE;
            }
            frontierSearchPtr_->resetSearchDistance();
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
        int fi_drop_count;
    };

    class HysterisisControl : public BT::StatefulActionNode
    {
    public:
        HysterisisControl(const std::string &name, const BT::NodeConfig &config,
                          std::shared_ptr<FullPathOptimizer> full_path_optimizer) : BT::StatefulActionNode(name, config)
        {
            LOG_INFO("HysterisisControl Constructor");
            minDistance = std::numeric_limits<double>::max();
            eventLoggerInstance.startEvent("startHysterisis");
            full_path_optimizer_ = full_path_optimizer;
        }

        BT::NodeStatus onStart() override
        {
            eventLoggerInstance.startEvent("HysterisisControl");
            LOG_FLOW("MODULE HysterisisControl");
            geometry_msgs::msg::PoseStamped robotP;
            if (!config().blackboard->get<geometry_msgs::msg::PoseStamped>("latest_robot_pose", robotP))
            {
                // Handle the case when "latest_robot_pose" is not found
                LOG_FATAL("Failed to retrieve latest_robot_pose from blackboard.");
                throw std::runtime_error("Failed to retrieve latest_robot_pose from blackboard.");
            }
            CurrentGoalStatus status;
            if (!config().blackboard->get<CurrentGoalStatus>("current_goal_status", status))
            {
                LOG_FATAL("Failed to retrieve hysteresis_enabled from blackboard.");
                throw std::runtime_error("Failed to retrieve hysteresis_enabled from blackboard.");
            }
            /**
             * The status is set to RUNNING when when the robot is still replanning (typically at 1Hz) towards a goal.
             * The status is set to SUCCESS when the robot has reached the goal / the goal is mapped.
             */
            if (status == CurrentGoalStatus::RUNNING)
            {
                Frontier allocatedFrontier;
                getInput<Frontier>("allocated_frontier", allocatedFrontier);
                LOG_INFO("Hysterisis prior: " << allocatedFrontier);

                LOG_DEBUG("Value of min Distance " << minDistance);
                LOG_DEBUG("Reference of min Distance " << &minDistance);
                if (parameterInstance.getValue<bool>("goalHysteresis/use_euclidean_distance") == true)
                {
                    LOG_INFO("Using Euclidean Distance for hysteresis");
                    if (distanceBetweenPoints(allocatedFrontier.getGoalPoint(), robotP.pose.position) < minDistance - 3.0)
                    {
                        minDistance = distanceBetweenPoints(allocatedFrontier.getGoalPoint(), robotP.pose.position);
                        mostFrequentFrontier = allocatedFrontier;
                        LOG_DEBUG("Found a closer one: " << minDistance);
                    }
                }
                else if (parameterInstance.getValue<bool>("goalHysteresis/use_roadmap_planner_distance") == true)
                {
                    Frontier robotPoseFrontier;
                    robotPoseFrontier.setGoalPoint(robotP.pose.position.x, robotP.pose.position.y);
                    robotPoseFrontier.setUID(generateUID(robotPoseFrontier));
                    robotPoseFrontier.setPathLength(0.0);
                    robotPoseFrontier.setPathLengthInM(0.0);
                    LOG_INFO("Using Roadmap Planner Distance for hysteresis");
                    auto lengthToGoal = full_path_optimizer_->calculateLengthRobotToGoal(robotPoseFrontier, allocatedFrontier, robotP);
                    if (lengthToGoal < minDistance - 3.0)
                    {
                        minDistance = lengthToGoal;
                        mostFrequentFrontier = allocatedFrontier;
                        LOG_DEBUG("Found a closer one: " << minDistance);
                    }
                }

                // Set the most frequent frontier as the output
                setOutput<Frontier>("allocated_frontier_after_hysterisis", mostFrequentFrontier);
            }
            else
            {
                Frontier allocatedFrontier;
                getInput<Frontier>("allocated_frontier", allocatedFrontier);
                LOG_INFO("Hysterisis prior: " << allocatedFrontier);
                setOutput<Frontier>("allocated_frontier_after_hysterisis", allocatedFrontier);
                if (parameterInstance.getValue<bool>("goalHysteresis/use_euclidean_distance") == true)
                    minDistance = distanceBetweenPoints(allocatedFrontier.getGoalPoint(), robotP.pose.position);
                else if (parameterInstance.getValue<bool>("goalHysteresis/use_roadmap_planner_distance") == true)
                {
                    Frontier robotPoseFrontier;
                    robotPoseFrontier.setGoalPoint(robotP.pose.position.x, robotP.pose.position.y);
                    robotPoseFrontier.setUID(generateUID(robotPoseFrontier));
                    robotPoseFrontier.setPathLength(0.0);
                    robotPoseFrontier.setPathLengthInM(0.0);
                    minDistance = full_path_optimizer_->calculateLengthRobotToGoal(robotPoseFrontier, allocatedFrontier, robotP);
                }
                LOG_INFO("Current length to goal: " << minDistance);
                mostFrequentFrontier = allocatedFrontier;
            }
            LOG_INFO("Hysterisis post: " << mostFrequentFrontier);
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

        double minDistance;
        Frontier mostFrequentFrontier;
        std::shared_ptr<FullPathOptimizer> full_path_optimizer_;
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
            eventLoggerInstance.startEvent("ExecuteRecoveryMove");
            LOG_FLOW("MODULE ExecuteRecoveryMove");
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
            LOG_FLOW("SendNav2Goal onStart");
            Frontier allocatedFrontier;
            latestAllocation_ = allocatedFrontier;
            getInput("allocated_frontier", allocatedFrontier);
            // LOG_INFO("Allocated frontier oz:" + std::to_string(allocatedFrontier.getGoalOrientation().z));
            // LOG_INFO("Allocated frontier ow:" + std::to_string(allocatedFrontier.getGoalOrientation().w));
            // LOG_INFO("Allocated frontier ox:" + std::to_string(allocatedFrontier.getGoalOrientation().x));
            // LOG_INFO("Allocated frontier oy:" + std::to_string(allocatedFrontier.getGoalOrientation().y));
            // LOG_INFO("Allocated frontier uid:" + std::to_string(allocatedFrontier.getUID()));
            geometry_msgs::msg::PoseStamped goalPose;
            goalPose.header.frame_id = "map";
            goalPose.pose.position = allocatedFrontier.getGoalPoint();
            goalPose.pose.orientation = allocatedFrontier.getGoalOrientation();
            nav2_interface_->sendGoal(goalPose);
            eventLoggerInstance.startEvent("GoalSentToNav2");
            time_for_planning_ = eventLoggerInstance.getTimeSinceStart("TimeForPlanning");
            return BT::NodeStatus::RUNNING;
        }

        BT::NodeStatus onRunning()
        {
            double timeoutValue;
            getInput("timeout_value", timeoutValue);
            LOG_DEBUG("SendNav2Goal onRunning");
            if (nav2_interface_->goalStatus() == 0 && eventLoggerInstance.getTimeSinceStart("GoalSentToNav2") + time_for_planning_ < timeoutValue)
                return BT::NodeStatus::RUNNING;
            else if (nav2_interface_->goalStatus() == 0 && eventLoggerInstance.getTimeSinceStart("GoalSentToNav2") + time_for_planning_ >= timeoutValue)
            {
                eventLoggerInstance.startEvent("TimeForPlanning");
                return BT::NodeStatus::SUCCESS;
            }
            else if (nav2_interface_->goalStatus() == 1)
            {
                eventLoggerInstance.startEvent("TimeForPlanning");
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
                eventLoggerInstance.startEvent("TimeForPlanning");
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
            return {BT::InputPort<Frontier>("allocated_frontier"),
                    BT::InputPort<double>("timeout_value")};
        }

        std::shared_ptr<Nav2Interface> nav2_interface_;
        std::shared_ptr<TaskAllocator> taskAllocator_;
        rclcpp::Node::SharedPtr ros_node_ptr_;
        Frontier latestAllocation_;
        int latestAllocationFailures_;
        double time_for_planning_;
    };

    class CheckIfGoalMapped : public BT::StatefulActionNode
    {
    public:
        CheckIfGoalMapped(const std::string &name, const BT::NodeConfig &config,
                          std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
                          rclcpp::Node::SharedPtr ros_node_ptr,
                          std::shared_ptr<FullPathOptimizer> full_path_optimizer) : BT::StatefulActionNode(name, config)
        {
            explore_costmap_ros_ = explore_costmap_ros;
            ros_node_ptr_ = ros_node_ptr;
            full_path_optimizer_ = full_path_optimizer;
            LOG_INFO("CheckIfGoalMapped Constructor");
        }

        BT::NodeStatus onStart() override
        {
            LOG_FLOW("CheckIfGoalMapped onStart");
            Frontier allocatedFrontier;
            getInput("allocated_frontier", allocatedFrontier);
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
                LOG_WARN("Goal is mapped. Restarting....");
                config().blackboard->set<CurrentGoalStatus>("current_goal_status", CurrentGoalStatus::COMPLETE);
                return BT::NodeStatus::SUCCESS;
            }
            geometry_msgs::msg::PoseStamped robotP;
            if (!config().blackboard->get<geometry_msgs::msg::PoseStamped>("latest_robot_pose", robotP))
            {
                // Handle the case when "latest_robot_pose" is not found
                LOG_FATAL("Failed to retrieve latest_robot_pose from blackboard.");
                throw std::runtime_error("Failed to retrieve latest_robot_pose from blackboard.");
            }
            if (!full_path_optimizer_->refineAndPublishPath(robotP, allocatedFrontier))
            {
                LOG_ERROR("Failed to refine and publish path between robotP: " << robotP.pose.position.x << ", " << robotP.pose.position.y << " and " << allocatedFrontier);
                config().blackboard->set<CurrentGoalStatus>("current_goal_status", CurrentGoalStatus::COMPLETE);
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::FAILURE;
        }

        BT::NodeStatus onRunning()
        {
            throw BT::RuntimeError("ReplanTimeoutCompleteBT should not be running");
            return BT::NodeStatus::FAILURE;
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
        std::shared_ptr<FullPathOptimizer> full_path_optimizer_;
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
            LOG_FLOW("ReplanTimeoutCompleteBT onStart");
            if (eventLoggerInstance.getTimeSinceStart("triggeredReplan") > timeoutValue)
            {
                LOG_WARN("Replan duration exceeded. Check cpu performance!!");
                eventLoggerInstance.startEvent("triggeredReplan");
                return BT::NodeStatus::FAILURE;
            }
            if (eventLoggerInstance.getTimeSinceStart("replanTimeout") > timeoutValue)
            {
                LOG_WARN("Replanning timed out. Restarting the frontier computation");
                eventLoggerInstance.startEvent("replanTimeout");
                config().blackboard->set<CurrentGoalStatus>("current_goal_status", CurrentGoalStatus::RUNNING);
                eventLoggerInstance.startEvent("triggeredReplan");
                return BT::NodeStatus::SUCCESS;
            }
            eventLoggerInstance.startEvent("triggeredReplan");
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
            cmd_vel_publisher_ = ros_node_ptr->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_nav", 10);
            LOG_INFO("RecoveryMoveBack Constructor");
        }

        BT::NodeStatus onStart() override
        {
            LOG_FLOW("RecoveryMoveBack onStart");
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
        exploration_active_ = true;
        blackboard = BT::Blackboard::create();
        blackboard->set<CurrentGoalStatus>("current_goal_status", CurrentGoalStatus::RUNNING);

        robot_namespaces_ = {};
        bt_node_->declare_parameter("robot_namespaces", rclcpp::ParameterValue(robot_namespaces_));
        config_ = {"100.0", "100.0", "100.0", "-100.0", "-100.0", "-100.0", "-100.0", "100.0"};
        bt_node_->declare_parameter("config", rclcpp::ParameterValue(config_));
        bt_node_->declare_parameter("process_other_robots", rclcpp::ParameterValue(false));
        bt_node_->declare_parameter("bt_xml_path", "/root/dev_ws/src/frontier_exploration/frontier_exploration/xml/exploration.xml");

        bt_node_->get_parameter("robot_namespaces", robot_namespaces_);
        bt_node_->get_parameter("config", config_);
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

        frontierSearchPtr_ = std::make_shared<FrontierSearch>(*(explore_costmap_ros_->getLayeredCostmap()->getCostmap()));
        full_path_optimizer_ = std::make_shared<FullPathOptimizer>(bt_node_, explore_costmap_ros_);
        //---------------------------------------------ROS RELATED------------------------------------------
        exploration_rviz_sub_ = node->create_subscription<std_msgs::msg::Int32>("/exploration_state", 10, std::bind(&FrontierExplorationServer::rvizControl, this, std::placeholders::_1));
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
        while (!explore_costmap_ros_->isCurrent())
        {
            LOG_WARN("Waiting for explore costmap to be current.");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        eventLoggerInstance.startEvent("clearRoadmap");
        eventLoggerInstance.startEvent("replanTimeout");

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
            return std::make_unique<HysterisisControl>(name, config, full_path_optimizer_);
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
            return std::make_unique<CheckIfGoalMapped>(name, config, explore_costmap_ros_, bt_node_, full_path_optimizer_);
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
        int bt_sleep_duration = parameterInstance.getValue<int>("explorationBT/bt_sleep_ms");
        while (rclcpp::ok())
        {
            if (exploration_active_)
            {
                behaviour_tree.tickOnce();
                std::this_thread::sleep_for(std::chrono::milliseconds(bt_sleep_duration));
                LOG_DEBUG("TICKED ONCE");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void FrontierExplorationServer::rvizControl(std_msgs::msg::Int32 rvizControlValue)
    {
        if (rvizControlValue.data == 0)
        {
            LOG_WARN("Pausing exploration");
            exploration_active_ = false;
            nav2_interface_->cancelAllGoals();
        }
        else if (rvizControlValue.data == 1)
        {
            LOG_WARN("Playing exploration");
            exploration_active_ = true;
        }
    }

    void FrontierExplorationServer::handle_multirobot_current_goal_request(
        std::shared_ptr<rmw_request_id_t> request_header,
        std::shared_ptr<frontier_msgs::srv::SendCurrentGoal::Request> request,
        std::shared_ptr<frontier_msgs::srv::SendCurrentGoal::Response> response)
    {
        std::lock_guard<std::mutex> lock(robot_active_goals_.mutex);
        if (request->goal_state == 1)
            robot_active_goals_.goals[request->sending_robot_name] = std::make_shared<geometry_msgs::msg::PoseStamped>(request->current_goal);
        else if (request->goal_state == 0)
            robot_active_goals_.goals[request->sending_robot_name] = nullptr;

        response->success = true;
        return;
    }

};