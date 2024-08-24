#include <frontier_exploration/ExplorationBT.hpp>
#include <frontier_exploration/geometry_tools.hpp>

#if defined(FRONTIER_POINT_MEDIAN) + defined(FRONTIER_POINT_INITIAL) > 1
#error "Only one of FRONTIER_POINT_MEDIAN, or FRONTIER_POINT_INITIAL can be defined at a time."
#elif !defined(FRONTIER_POINT_MEDIAN) && !defined(FRONTIER_POINT_INITIAL)
#error "One of FRONTIER_POINT_MEDIAN, or FRONTIER_POINT_INITIAL must be defined."
#endif

namespace frontier_exploration
{
    class WaitForCurrent : public BT::StatefulActionNode
    {
    public:
        WaitForCurrent(const std::string &name, const BT::NodeConfig &config,
                       std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
                       rclcpp::Node::SharedPtr ros_node_ptr) : BT::StatefulActionNode(name, config)
        {
            explore_costmap_ros_ = explore_costmap_ros;
            ros_node_ptr_ = ros_node_ptr;
            std::cout << "WaitForCurrent Constructor" << std::endl;
        }

        BT::NodeStatus onStart() override
        {
            return BT::NodeStatus::RUNNING;
        }

        BT::NodeStatus onRunning() override
        {
            if (!explore_costmap_ros_->isCurrent())
            {
                std::cout << "Waiting for costmap to be current" << std::endl;
                return BT::NodeStatus::RUNNING;
            }
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
                                std::shared_ptr<BoundedExploreLayer> bel_ptr, std::vector<std::string> boundary_config,
                                rclcpp::Node::SharedPtr ros_node_ptr) : BT::StatefulActionNode(name, config)
        {
            bel_ptr_ = bel_ptr;
            config_ = boundary_config;
            ros_node_ptr_ = ros_node_ptr;
            std::cout << COLOR_STR("UpdateBoundaryPolygonBT Constructor", ros_node_ptr_->get_namespace()) << std::endl;
        }

        BT::NodeStatus onStart() override
        {
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
            // for (const auto &point : explore_boundary_.polygon.points)
            // {
            //     std::cout << "Sending Polygon from config x:" << point.x << " y: " << point.y << " z: " << point.z << std::endl;
            // }

            explore_center_.header.frame_id = "map";
            explore_center_.point.x = 5.5;
            explore_center_.point.y = 5.5;

            auto updateBoundaryResult = bel_ptr_->updateBoundaryPolygon(explore_boundary_);
            std::cout << COLOR_STR("Adding update boundary polygon for spin.", ros_node_ptr_->get_namespace()) << std::endl;
            if (updateBoundaryResult == true)
            {
                std::cout << COLOR_STR("Region boundary set", ros_node_ptr_->get_namespace()) << std::endl;
            }
            else
            {
                std::cout << COLOR_STR("Failed to receive response for updateBoundaryPolygon called from within.", ros_node_ptr_->get_namespace()) << std::endl;
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus onRunning()
        {
            std::cout << COLOR_STR("UpdateBoundaryPolygonBT On running called ", ros_node_ptr_->get_namespace()) << std::endl;
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

        std::shared_ptr<BoundedExploreLayer> bel_ptr_;
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
            std::cout << COLOR_STR("SearchForFrontiersBT Constructor", ros_node_ptr_->get_namespace()) << std::endl;
        }

        BT::NodeStatus onStart() override
        {
            frontierSearchPtr_->reset();
            explore_costmap_ros_->getCostmap()->getMutex()->lock();
            std::cout << COLOR_STR("SearchForFrontiersBT OnStart called ", ros_node_ptr_->get_namespace()) << std::endl;
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
                explore_costmap_ros_->getCostmap()->getMutex()->unlock();
                return BT::NodeStatus::FAILURE;
            }
            setOutput("frontier_list", frontier_list);
            setOutput("every_frontier", every_frontier);
            frontierSearchPtr_->resetSearchDistance();
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus onRunning()
        {
            std::cout << COLOR_STR("SearchForFrontiersBT On running called ", ros_node_ptr_->get_namespace()) << std::endl;
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

    class VerifyFrontierSearchBT : public BT::StatefulActionNode
    {
    public:
        VerifyFrontierSearchBT(const std::string &name, const BT::NodeConfig &config,
                             std::shared_ptr<FrontierSearch> frontierSearchPtr, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
                             rclcpp::Node::SharedPtr ros_node_ptr) : BT::StatefulActionNode(name, config)
        {
            explore_costmap_ros_ = explore_costmap_ros;
            ros_node_ptr_ = ros_node_ptr;
            std::cout << COLOR_STR("VerifyFrontierSearchBT Constructor", ros_node_ptr_->get_namespace()) << std::endl;
        }

        BT::NodeStatus onStart() override
        {
            std::cout << COLOR_STR("VerifyFrontierSearchBT OnStart called ", ros_node_ptr_->get_namespace()) << std::endl;
            std::vector<Frontier> frontier_list;
            getInput<std::vector<Frontier>>("frontier_list", frontier_list);
            if(verifyFrontierList(frontier_list, explore_costmap_ros_->getCostmap()))
                return BT::NodeStatus::SUCCESS;
            return BT::NodeStatus::FAILURE;
        }

        BT::NodeStatus onRunning()
        {
            std::cout << COLOR_STR("VerifyFrontierSearchBT On running called ", ros_node_ptr_->get_namespace()) << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

        void onHalted()
        {
            return;
        }

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::vector<Frontier>>("frontier_list")
            };
        }

        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
        rclcpp::Node::SharedPtr ros_node_ptr_;
    };

    class ProcessFrontierCostsBT : public BT::StatefulActionNode
    {
    public:
        ProcessFrontierCostsBT(const std::string &name, const BT::NodeConfig &config,
                           std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
                           std::shared_ptr<BoundedExploreLayer> bel_ptr,
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
            std::cout << COLOR_STR("ProcessFrontierCostsBT Constructor", ros_node_ptr_->get_namespace()) << std::endl;
        }

        BT::NodeStatus onStart() override
        {
            taskAllocator_->reset();
            std::cout << COLOR_STR("ProcessFrontierCostsBT OnStart called ", ros_node_ptr_->get_namespace()) << std::endl;
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
            bool frontierCostsSuccess = bel_ptr_->getFrontierCosts(frontierCostsRequestPtr, frontierCostsResultPtr);
            std::cout << COLOR_STR("Sent GNF request", ros_node_ptr_->get_namespace()) << std::endl;
            if (frontierCostsSuccess == false)
            {
                std::cout << COLOR_STR("Failed to receive response for getNextFrontier called from within the robot.", ros_node_ptr_->get_namespace()) << std::endl;
                explore_costmap_ros_->getCostmap()->getMutex()->unlock();
                return BT::NodeStatus::FAILURE;
            }
            setOutput("frontier_costs_result", frontierCostsResultPtr->frontier_list);
            taskAllocator_->addRobotTasks(frontierCostsResultPtr->frontier_costs, frontierCostsResultPtr->frontier_distances, robot_name);

            bel_ptr_->logMapData(frontierCostsRequestPtr);
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
        std::shared_ptr<BoundedExploreLayer> bel_ptr_;
        RobotActiveGoals &robot_goals_;
        std::shared_ptr<TaskAllocator> taskAllocator_;
        rclcpp::Node::SharedPtr ros_node_ptr_;
    };

    class OptimizeFullPath : public BT::StatefulActionNode
    {
    public:
        OptimizeFullPath(const std::string &name, const BT::NodeConfig &config,
                          std::shared_ptr<FullPathOptimizer> full_path_optimizer,
                          std::shared_ptr<BoundedExploreLayer> bel_ptr) : BT::StatefulActionNode(name, config)
        {
            full_path_optimizer_ = full_path_optimizer;
            // std::cout << COLOR_STR("OptimizeFullPath Constructor", ros_node_ptr_->get_namespace()) << std::endl;
        }

        BT::NodeStatus onStart() override
        {
            // std::cout << COLOR_STR("OptimizeFullPath OnStart called ", ros_node_ptr_->get_namespace()) << std::endl;
            std::vector<Frontier> globalFrontierList;
            getInput<std::vector<Frontier>>("frontier_costs_result", globalFrontierList);
            // full_path_optimizer_->getTopThreeFrontiers(globalFrontierList);
            geometry_msgs::msg::PoseStamped robotP;
            config().blackboard->get<geometry_msgs::msg::PoseStamped>("latest_robot_pose", robotP);
            full_path_optimizer_->publishLocalSearchArea(globalFrontierList, 3, robotP);
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

        std::shared_ptr<FullPathOptimizer> full_path_optimizer_;
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
            std::cout << "!!!!! NODE NAME! " << ros_node_ptr_->get_namespace() << std::endl;
            std::cout << COLOR_STR("GetAllocatedGoalBT Constructor", ros_node_ptr_->get_namespace()) << std::endl;
        }

        BT::NodeStatus onStart() override
        {
            std::cout << COLOR_STR("GetAllocatedGoalBT OnStart called ", ros_node_ptr_->get_namespace()) << std::endl;
            taskAllocator_->solveAllocationHungarian();
            auto allocatedIndex = taskAllocator_->getAllocatedTasks()[ros_node_ptr_->get_namespace()];
            std::vector<Frontier> globalFrontierList;
            getInput<std::vector<Frontier>>("frontier_costs_result", globalFrontierList);
            auto allocatedFrontier = globalFrontierList[allocatedIndex];
            std::cout << COLOR_STR("Allocated frontier x:" + std::to_string(allocatedFrontier.getGoalPoint().x), ros_node_ptr_->get_namespace()) << std::endl;
            std::cout << COLOR_STR("Allocated frontier y:" + std::to_string(allocatedFrontier.getGoalPoint().y), ros_node_ptr_->get_namespace()) << std::endl;
            std::cout << COLOR_STR("Allocated frontier z:" + std::to_string(allocatedFrontier.getGoalPoint().z), ros_node_ptr_->get_namespace()) << std::endl;
            std::cout << COLOR_STR("Allocated frontier oz:" + std::to_string(allocatedFrontier.getGoalOrientation().z), ros_node_ptr_->get_namespace()) << std::endl;
            std::cout << COLOR_STR("Allocated frontier ow:" + std::to_string(allocatedFrontier.getGoalOrientation().w), ros_node_ptr_->get_namespace()) << std::endl;
            std::cout << COLOR_STR("Allocated frontier ox:" + std::to_string(allocatedFrontier.getGoalOrientation().x), ros_node_ptr_->get_namespace()) << std::endl;
            std::cout << COLOR_STR("Allocated frontier oy:" + std::to_string(allocatedFrontier.getGoalOrientation().y), ros_node_ptr_->get_namespace()) << std::endl;
            std::cout << COLOR_STR("Allocated frontier uid:" + std::to_string(allocatedFrontier.getUID()), ros_node_ptr_->get_namespace()) << std::endl;
            setOutput<Frontier>("allocated_frontier", allocatedFrontier);
            explore_costmap_ros_->getCostmap()->getMutex()->unlock();
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
            std::cout << COLOR_STR("SendNav2Goal Constructor", ros_node_ptr_->get_namespace()) << std::endl;
        }

        BT::NodeStatus onStart() override
        {
            std::cout << "SendNav2Goal onStart" << std::endl;
            Frontier allocatedFrontier;
            getInput("allocated_frontier", allocatedFrontier);
            std::cout << COLOR_STR("Allocated frontier x:" + std::to_string(allocatedFrontier.getGoalPoint().x), ros_node_ptr_->get_namespace()) << std::endl;
            std::cout << COLOR_STR("Allocated frontier y:" + std::to_string(allocatedFrontier.getGoalPoint().y), ros_node_ptr_->get_namespace()) << std::endl;
            std::cout << COLOR_STR("Allocated frontier z:" + std::to_string(allocatedFrontier.getGoalPoint().z), ros_node_ptr_->get_namespace()) << std::endl;
            std::cout << COLOR_STR("Allocated frontier oz:" + std::to_string(allocatedFrontier.getGoalOrientation().z), ros_node_ptr_->get_namespace()) << std::endl;
            std::cout << COLOR_STR("Allocated frontier ow:" + std::to_string(allocatedFrontier.getGoalOrientation().w), ros_node_ptr_->get_namespace()) << std::endl;
            std::cout << COLOR_STR("Allocated frontier ox:" + std::to_string(allocatedFrontier.getGoalOrientation().x), ros_node_ptr_->get_namespace()) << std::endl;
            std::cout << COLOR_STR("Allocated frontier oy:" + std::to_string(allocatedFrontier.getGoalOrientation().y), ros_node_ptr_->get_namespace()) << std::endl;
            std::cout << COLOR_STR("Allocated frontier uid:" + std::to_string(allocatedFrontier.getUID()), ros_node_ptr_->get_namespace()) << std::endl;
            geometry_msgs::msg::PoseStamped goalPose;
            goalPose.header.frame_id = "map";
            goalPose.pose.position = allocatedFrontier.getGoalPoint();
            goalPose.pose.orientation = allocatedFrontier.getGoalOrientation();
            nav2_interface_->sendGoal(goalPose);
            return BT::NodeStatus::RUNNING;
        }

        BT::NodeStatus onRunning()
        {
            std::cout << "SendNav2Goal onRunning" << std::endl;
            if (nav2_interface_->goalStatus() == 0)
                return BT::NodeStatus::RUNNING;
            else if (nav2_interface_->goalStatus() == 1)
                return BT::NodeStatus::SUCCESS;
            else if (nav2_interface_->goalStatus() == -1)
                return BT::NodeStatus::FAILURE;
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
            std::cout << COLOR_STR("CheckIfGoalMapped Constructor", ros_node_ptr_->get_namespace()) << std::endl;
        }

        BT::NodeStatus onStart() override
        {
            std::cout << "CheckIfGoalMapped onStart" << std::endl;
            Frontier allocatedFrontier;
            getInput("allocated_frontier", allocatedFrontier);
            // std::cout << COLOR_STR("Allocated frontier x:" + std::to_string(allocatedFrontier.getGoalPoint().x), ros_node_ptr_->get_namespace()) << std::endl;
            // std::cout << COLOR_STR("Allocated frontier y:" + std::to_string(allocatedFrontier.getGoalPoint().y), ros_node_ptr_->get_namespace()) << std::endl;
            // std::cout << COLOR_STR("Allocated frontier z:" + std::to_string(allocatedFrontier.getGoalPoint().z), ros_node_ptr_->get_namespace()) << std::endl;
            // std::cout << COLOR_STR("Allocated frontier oz:" + std::to_string(allocatedFrontier.getGoalOrientation().z), ros_node_ptr_->get_namespace()) << std::endl;
            // std::cout << COLOR_STR("Allocated frontier ow:" + std::to_string(allocatedFrontier.getGoalOrientation().w), ros_node_ptr_->get_namespace()) << std::endl;
            // std::cout << COLOR_STR("Allocated frontier ox:" + std::to_string(allocatedFrontier.getGoalOrientation().x), ros_node_ptr_->get_namespace()) << std::endl;
            // std::cout << COLOR_STR("Allocated frontier oy:" + std::to_string(allocatedFrontier.getGoalOrientation().y), ros_node_ptr_->get_namespace()) << std::endl;
            // std::cout << COLOR_STR("Allocated frontier uid:" + std::to_string(allocatedFrontier.getUID()), ros_node_ptr_->get_namespace()) << std::endl;
            geometry_msgs::msg::PoseStamped goalPose;
            goalPose.header.frame_id = "map";
            goalPose.pose.position = allocatedFrontier.getGoalPoint();
            if (surroundingCellsMapped(goalPose.pose.position, *explore_costmap_ros_->getCostmap()))
                return BT::NodeStatus::SUCCESS;
            return BT::NodeStatus::FAILURE;
        }

        BT::NodeStatus onRunning()
        {
            std::cout << "CheckIfGoalMapped onRunning" << std::endl;
            Frontier allocatedFrontier;
            getInput("allocated_frontier", allocatedFrontier);
            geometry_msgs::msg::PoseStamped goalPose;
            goalPose.header.frame_id = "map";
            goalPose.pose.position = allocatedFrontier.getGoalPoint();
            if (surroundingCellsMapped(goalPose.pose.position, *explore_costmap_ros_->getCostmap()))
                return BT::NodeStatus::SUCCESS;
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


    class RecoveryMoveBack : public BT::StatefulActionNode
    {
    public:
        RecoveryMoveBack(const std::string &name, const BT::NodeConfig &config,
                          std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
                          rclcpp::Node::SharedPtr ros_node_ptr) : BT::StatefulActionNode(name, config)
        {
            explore_costmap_ros_ = explore_costmap_ros;
            ros_node_ptr_ = ros_node_ptr;
            cmd_vel_publisher_ = ros_node_ptr->create_publisher<geometry_msgs::msg::Twist>("/smb_velocity_controller/cmd_vel", 10);
            std::cout << COLOR_STR("RecoveryMoveBack Constructor", ros_node_ptr_->get_namespace()) << std::endl;
        }

        BT::NodeStatus onStart() override
        {
            std::cout << "RecoveryMoveBack onStart" << std::endl;
            startTime = std::chrono::high_resolution_clock::now();
            return BT::NodeStatus::RUNNING;
        }

        BT::NodeStatus onRunning()
        {
            std::cout << "RecoveryMoveBack onRunning" << std::endl;
            auto endTime = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> duration = endTime - startTime;
            double maxDuration;
            getInput("move_back_duration", maxDuration);
            if(duration.count() > maxDuration)
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
        bt_node_ = node;
        blackboard = BT::Blackboard::create();

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

        bt_node_->get_parameter("retry_count", retry_);
        bt_node_->get_parameter("nav2_goal_timeout_sec", nav2WaitTime_);
        bt_node_->get_parameter("use_custom_sim", use_custom_sim_);
        bt_node_->get_parameter("robot_namespaces", robot_namespaces_);
        bt_node_->get_parameter("config", config_);
        bt_node_->get_parameter("min_frontier_cluster_size", min_frontier_cluster_size_);
        bt_node_->get_parameter("max_frontier_cluster_size", max_frontier_cluster_size_);
        bt_node_->get_parameter("max_frontier_distance", max_frontier_distance_);
        bt_node_->get_parameter("process_other_robots", process_other_robots_);
        //--------------------------------------------NAV2 CLIENT RELATED-------------------------------
        nav2_interface_ = std::make_shared<Nav2Interface>(bt_node_);

        //--------------------------------------------EXPLORE SERVER RELATED----------------------------

        explore_costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("explore_costmap", std::string{bt_node_->get_namespace()}, "explore_costmap");
        explore_costmap_ros_->configure();
        // Launch a thread to run the costmap node
        explore_costmap_thread_ = std::make_unique<nav2_util::NodeThread>(explore_costmap_ros_);
        explore_costmap_ros_->activate();

        //------------------------------------------BOUNDED EXPLORE LAYER RELATED------------------------
        bel_ptr_ = std::make_shared<BoundedExploreLayer>(explore_costmap_ros_);
        task_allocator_ptr_ = std::make_shared<TaskAllocator>();

        multirobot_service_callback_group_ = bt_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        service_send_current_goal_ = bt_node_->create_service<frontier_msgs::srv::SendCurrentGoal>(
            "multirobot_send_current_goal", std::bind(&FrontierExplorationServer::handle_multirobot_current_goal_request, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
            rmw_qos_profile_default, multirobot_service_callback_group_);

        frontierSearchPtr_ = std::make_shared<FrontierSearch>(*(explore_costmap_ros_->getLayeredCostmap()->getCostmap()), min_frontier_cluster_size_, max_frontier_cluster_size_, max_frontier_distance_);
        full_path_optimizer_ = std::make_shared<FullPathOptimizer>(bt_node_, explore_costmap_ros_, bel_ptr_->getCostManagerPtr()->getCostCalcPtr()->getRoadmapPtr());
        //---------------------------------------------ROS RELATED------------------------------------------
        tf_listener_ = std::make_shared<tf2_ros::Buffer>(bt_node_->get_clock());
        RCLCPP_INFO_STREAM(bt_node_->get_logger(), COLOR_STR("FrontierExplorationServer::FrontierExplorationServer()", bt_node_->get_namespace()));
    }

    FrontierExplorationServer::~FrontierExplorationServer()
    {
        RCLCPP_INFO_STREAM(bt_node_->get_logger(), COLOR_STR("FrontierExplorationServer::~FrontierExplorationServer()", bt_node_->get_namespace()));
        explore_costmap_ros_->deactivate();
        explore_costmap_ros_->cleanup();
        explore_costmap_thread_.reset();
    }

    void FrontierExplorationServer::makeBTNodes()
    {
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

        BT::NodeBuilder builder_frontier_costs =
            [&](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<ProcessFrontierCostsBT>(name, config, explore_costmap_ros_, bel_ptr_, robot_active_goals_, task_allocator_ptr_, bt_node_);
        };
        factory.registerBuilder<ProcessFrontierCostsBT>("ProcessFrontierCosts", builder_frontier_costs);
        
        BT::NodeBuilder builder_full_path_optimizer =
            [&](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<OptimizeFullPath>(name, config, full_path_optimizer_, bel_ptr_);
        };
        factory.registerBuilder<OptimizeFullPath>("OptimizeFullPath", builder_full_path_optimizer);

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

        BT::NodeBuilder builder_recovery_back =
            [&](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<RecoveryMoveBack>(name, config, explore_costmap_ros_, bt_node_);
        };
        factory.registerBuilder<RecoveryMoveBack>("RecoveryMoveBack", builder_recovery_back);

        behaviour_tree = factory.createTreeFromFile("/root/dev_ws/src/frontier_exploration/frontier_exploration/xml/exploration.xml", blackboard);
        while (rclcpp::ok())
        {
            behaviour_tree.tickOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            RCLCPP_ERROR(bt_node_->get_logger(), "TICKED ONCE");
        }
    }

    void FrontierExplorationServer::handle_multirobot_current_goal_request(
        std::shared_ptr<rmw_request_id_t> request_header,
        std::shared_ptr<frontier_msgs::srv::SendCurrentGoal::Request> request,
        std::shared_ptr<frontier_msgs::srv::SendCurrentGoal::Response> response)
    {
        std::lock_guard<std::mutex> lock(robot_active_goals_.mutex);
        RCLCPP_ERROR_STREAM(bt_node_->get_logger(), COLOR_STR("Goal updated for: " + request->sending_robot_name + " New goal? " + std::to_string(request->goal_state), bt_node_->get_namespace()));
        if (request->goal_state == 1)
            robot_active_goals_.goals[request->sending_robot_name] = std::make_shared<geometry_msgs::msg::PoseStamped>(request->current_goal);
        else if (request->goal_state == 0)
            robot_active_goals_.goals[request->sending_robot_name] = nullptr;

        response->success = true;
        return;
    }

};