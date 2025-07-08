#include <frontier_exploration/CostAssigner.hpp>

namespace frontier_exploration
{
    using nav2_costmap_2d::FREE_SPACE;
    using nav2_costmap_2d::LETHAL_OBSTACLE;
    using nav2_costmap_2d::NO_INFORMATION;
    using rcl_interfaces::msg::ParameterType;

    CostAssigner::CostAssigner(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros) 
    {
        layered_costmap_ = explore_costmap_ros->getLayeredCostmap();
        exploration_mode_ = "ours";

        LOG_INFO("CostAssigner::onInitialize");

        // client_get_map_data2_ = internal_node_->create_client<slam_msgs::srv::GetMap>("orb_slam3_get_map_data");
        // rosVisualizerInstance = std::make_shared<RosVisualizer>(internal_node_, layered_costmap_->getCostmap());
        frontierCostsManager_ = std::make_shared<frontier_exploration::FrontierCostsManager>(explore_costmap_ros);
    }

    CostAssigner::~CostAssigner()
    {
        LOG_INFO("CostAssigner::~CostAssigner()");
        delete layered_costmap_;
        rclcpp::shutdown();
    }

    bool CostAssigner::processOurApproach(std::vector<FrontierPtr> &frontier_list, geometry_msgs::msg::Pose& start_pose_w)
    {
        LOG_DEBUG("CostAssigner::processOurApproach");
        eventLoggerInstance.startEvent("processOurApproach");

        // Getting map data
        // Create a service request
        auto request_map_data = std::make_shared<slam_msgs::srv::GetMap::Request>();
        auto response_map_data = std::make_shared<slam_msgs::srv::GetMap::Response>();
        std::vector<std::string> chosenMethods = {"RoadmapPlannerDistance", "ArrivalInformation"};
        // std::vector<std::string> chosenMethods = {"A*PlannerDistance", "ArrivalInformation"};
        // std::vector<std::string> chosenMethods = {"EuclideanDistance", "ArrivalInformation"};
        // std::vector<std::string> chosenMethods = {"RandomCosts"};
        // std::vector<std::string> chosenMethods = {};
        LOG_INFO("Methods chosen are: " << chosenMethods);
        std::vector<std::vector<std::string>> costTypes;
        for (auto frontier: frontier_list)
        {
            costTypes.push_back(chosenMethods);
        }
        // Select the frontier
        bool costsResult;
        costsResult = frontierCostsManager_->assignCosts(frontier_list, polygon_xy_min_max_, start_pose_w, response_map_data, costTypes);
        if (costsResult == false)
        {
            LOG_CRITICAL("The selection result for our method is false!");
            return costsResult;
        }

        eventLoggerInstance.endEvent("processOurApproach", 1);
        return costsResult;
    }

    void CostAssigner::logMapData(std::shared_ptr<GetFrontierCostsRequest> requestData)
    {
        PROFILE_FUNCTION;
        // for(auto frontier : requestData->frontier_list)
        // {
        //     std::cout << "FrontierPtr cost : " << frontier->getWeightedCost() << std::endl;
        // }
        // RosVisualizer::getInstance()exportMapCoverage(polygon_xy_min_max_, counter_, exploration_mode_);
        RosVisualizer::getInstance().visualizeFrontierMarker(requestData->frontier_list, requestData->every_frontier, layered_costmap_->getGlobalFrameID());
    }

    bool CostAssigner::getFrontierCosts(std::shared_ptr<GetFrontierCostsRequest> requestData, std::shared_ptr<GetFrontierCostsResponse> resultData)
    {
        frontierCostsManager_->setFrontierBlacklist(requestData->prohibited_frontiers);

        if (exploration_mode_ == "ours")
        {
            bool costsResult = CostAssigner::processOurApproach(requestData->frontier_list, requestData->start_pose.pose);
            eventLoggerInstance.startEvent("outside_processOurApproach");
            if (costsResult == false)
            {
                resultData->success = false;
                return resultData->success;
            }
            resultData->success = true;
            std::vector<FrontierPtr> frontiers_list;
            std::vector<double> frontier_costs;
            std::vector<double> frontier_distances;
            std::vector<double> frontier_arrival_information;
            std::vector<double> frontier_path_information;
            for (auto& frontier : requestData->frontier_list)
            {
                frontiers_list.push_back(frontier);
                frontier_costs.push_back(frontier->getWeightedCost());
                frontier_distances.push_back(frontier->getPathLengthInM());
                frontier_arrival_information.push_back(frontier->getArrivalInformation());
                // LOG_INFO("Cost is: " << resultData->frontier_costs.size();
            }
            LOG_DEBUG("Making list");
            resultData->frontier_list = frontiers_list;
            resultData->frontier_costs = frontier_costs;
            resultData->frontier_distances = frontier_distances;
            resultData->frontier_arrival_information = frontier_arrival_information;
            if(resultData->frontier_list != requestData->frontier_list)
            {
                throw std::runtime_error("Lists are not SAME!");
            }
        }
        else
        {
            LOG_FATAL("Invalid mode of exploration");
            rclcpp::shutdown();
        }
        LOG_DEBUG("Res frontier costs size: " << resultData->frontier_costs.size());
        LOG_DEBUG("Res frontier list size: " << resultData->frontier_list.size());
        eventLoggerInstance.endEvent("outside_processOurApproach", 1);
        return resultData->success;
    }

    bool CostAssigner::updateBoundaryPolygon(geometry_msgs::msg::PolygonStamped& explore_boundary)
    {
        // Transform all points of boundary polygon into costmap frame
        geometry_msgs::msg::PointStamped in;
        in.header = explore_boundary.header;
        for (const auto &point32 : explore_boundary.polygon.points)
        {
            LOG_TRACE("Sending Polygon from config x:" << point32.x << " y: " << point32.y << " z: " << point32.z);
            in.point = nav2_costmap_2d::toPoint(point32);
            polygon_.points.push_back(nav2_costmap_2d::toPoint32(in.point));
        }

        // if empty boundary provided, set to whole map
        if (polygon_.points.empty())
        {
            geometry_msgs::msg::Point32 temp;
            temp.x = layered_costmap_->getCostmap()->getOriginX();
            temp.y = layered_costmap_->getCostmap()->getOriginY();
            polygon_.points.push_back(temp);
            temp.y = layered_costmap_->getCostmap()->getSizeInMetersY();
            polygon_.points.push_back(temp);
            temp.x = layered_costmap_->getCostmap()->getSizeInMetersX();
            polygon_.points.push_back(temp);
            temp.y = layered_costmap_->getCostmap()->getOriginY();
            polygon_.points.push_back(temp);
        }

        // Find map size and origin by finding min/max points of polygon
        double min_x_polygon = std::numeric_limits<double>::infinity();
        double min_y_polygon = std::numeric_limits<double>::infinity();
        double max_x_polygon = -std::numeric_limits<double>::infinity(); // observe the minus here
        double max_y_polygon = -std::numeric_limits<double>::infinity(); // observe the minus here

        for (const auto &point : polygon_.points)
        {
            min_x_polygon = std::min(min_x_polygon, (double)point.x);
            min_y_polygon = std::min(min_y_polygon, (double)point.y);
            max_x_polygon = std::max(max_x_polygon, (double)point.x);
            max_y_polygon = std::max(max_y_polygon, (double)point.y);
        }

        polygon_xy_min_max_.push_back(min_x_polygon);
        polygon_xy_min_max_.push_back(min_y_polygon);
        polygon_xy_min_max_.push_back(max_x_polygon);
        polygon_xy_min_max_.push_back(max_y_polygon);
        return true;
    }
}