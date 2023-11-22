#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_util/node_thread.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>

#include <action_msgs/msg/goal_status_array.hpp>

#include <frontier_msgs/action/explore_task.hpp>
#include <frontier_msgs/srv/get_next_frontier.hpp>
#include <frontier_msgs/srv/update_boundary_polygon.hpp>

#include <frontier_exploration/geometry_tools.hpp>

#include <nav2_msgs/action/navigate_to_pose.hpp>


namespace frontier_exploration{

/**
 * @brief Server for frontier exploration action, runs the state machine associated with a
 * structured frontier exploration task and manages robot movement through move_base.
 */
class FrontierExplorationServer : public rclcpp::Node {
public:

    using GoalHandleExplore = rclcpp_action::ServerGoalHandle<frontier_msgs::action::ExploreTask>;
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
	using GoalHandleNav2 = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    /**
     * @brief Constructor for the server, sets up this node's ActionServer for exploration and ActionClient to move_base for robot movement.
     * @param name Name for SimpleActionServer
     */
    FrontierExplorationServer() : Node("explore_server")
    {
        this->declare_parameter("frequency", 5.0);
        this->declare_parameter("goal_aliasing", 0.1);
        this->declare_parameter("retry_count",30);
        this->declare_parameter("nav2_goal_timeout_sec", 35);

        this->get_parameter("frequency", frequency_);
        this->get_parameter("goal_aliasing", goal_aliasing_);
        this->get_parameter("retry_count", retry_);
        this->get_parameter("nav2_goal_timeout_sec", nav2WaitTime_);


        RCLCPP_ERROR_STREAM(rclcpp::get_logger("explore_server"),"frequency: " << frequency_);
        nav2Client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose"); //was previously move_base, true
        action_server_ = rclcpp_action::create_server<frontier_msgs::action::ExploreTask>(
            this,
            "explore_action",
            std::bind(&FrontierExplorationServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&FrontierExplorationServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&FrontierExplorationServer::handle_accepted, this, std::placeholders::_1));
        explore_costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("explore_costmap",std::string{get_namespace()},"explore_costmap");
        explore_costmap_ros_->configure();
        // Launch a thread to run the costmap node
        explore_costmap_thread_ = std::make_unique<nav2_util::NodeThread>(explore_costmap_ros_);
        explore_costmap_ros_->activate();

        tf_listener_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        nav2Publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
        nav2Subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>("goal_reached_path_follower",10, std::bind(&FrontierExplorationServer::feedbackNav2cb, this, std::placeholders::_1));
        dyn_params_handler_ = this->add_on_set_parameters_callback(
        std::bind(
            &FrontierExplorationServer::dynamicParametersCallback,
            this, std::placeholders::_1));
    }

    ~FrontierExplorationServer()
    {
        explore_costmap_ros_->deactivate();
        explore_costmap_ros_->cleanup();
        explore_costmap_thread_.reset();
    }

    rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
        std::vector<rclcpp::Parameter> parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;

        for (auto parameter : parameters) {
            const auto & param_type = parameter.get_type();
            const auto & param_name = parameter.get_name();

            if (param_type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                if (param_name == "frequency") {
                    frequency_ = parameter.as_double();
                }
                else if(param_name == "goal_aliasing") {
                    goal_aliasing_ = parameter.as_double();
                }
            }
            else if(param_type == rclcpp::ParameterType::PARAMETER_INTEGER) {
                if (param_name == "retry_count") {
                    retry_ = parameter.as_int();
                }
                else if(param_name == "nav2_goal_timeout_sec") {
                    nav2WaitTime_ = parameter.as_int();
                }
            }

        }

        result.successful = true;
        return result;
    }

private:

    std::shared_ptr<tf2_ros::Buffer> tf_listener_;
    rclcpp_action::Server<frontier_msgs::action::ExploreTask>::SharedPtr action_server_;

    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
    std::unique_ptr<nav2_util::NodeThread> explore_costmap_thread_;
    double frequency_, goal_aliasing_;
    bool success_, moving_;
    int retry_;
    int nav2WaitTime_;

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav2Client_;
    std::mutex nav2Clientlock_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav2Publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr nav2Subscriber_;
    NavigateToPose::Goal old_goal; //previously was old_goal
    std::shared_ptr<const frontier_msgs::action::ExploreTask::Goal> frontier_goal;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const frontier_msgs::action::ExploreTask::Goal> goal)
    {
        (void)uuid;
        frontier_goal = goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleExplore> goal_handle)
    {
        nav2Client_->async_cancel_goals_before(rclcpp::Clock().now());
        RCLCPP_WARN(rclcpp::get_logger("explore_server"),"Current exploration task cancelled");

        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleExplore> goal_handle)
    {
        std::thread{std::bind(&FrontierExplorationServer::executeCb, this, std::placeholders::_1, std::placeholders::_2),goal_handle, frontier_goal}.detach();
    }


    geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw)
    {
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        return tf2::toMsg(q);
    }

    /**
     * @brief Execute callback for actionserver, run after accepting a new goal
     * @param goal ActionGoal containing boundary of area to explore, and a valid centerpoint for the area.
     */

    void executeCb(const std::shared_ptr<GoalHandleExplore> goal_handle, std::shared_ptr<const frontier_msgs::action::ExploreTask::Goal> goal)
    {
        success_ = false;
        moving_ = false;
        auto feedback_ = std::make_shared<frontier_msgs::action::ExploreTask::Feedback>();
        auto res = std::make_shared<frontier_msgs::action::ExploreTask::Result>();

        // Don't compute a plan until costmap is valid (after clear costmap)
        rclcpp::Rate r(100);
        while (!explore_costmap_ros_->isCurrent()) {
            r.sleep();
        }
        rclcpp::Client<frontier_msgs::srv::UpdateBoundaryPolygon>::SharedPtr updateBoundaryPolygon = this->create_client<frontier_msgs::srv::UpdateBoundaryPolygon>("explore_costmap/update_boundary_polygon");
        rclcpp::Client<frontier_msgs::srv::GetNextFrontier>::SharedPtr getNextFrontier = this->create_client<frontier_msgs::srv::GetNextFrontier>("explore_costmap/get_next_frontier");

        //wait for move_base and costmap services
        if(!nav2Client_->wait_for_action_server(std::chrono::seconds(10)) || !updateBoundaryPolygon->wait_for_service(std::chrono::seconds(10)) || !getNextFrontier->wait_for_service(std::chrono::seconds(10))){
            goal_handle->abort(res);
            RCLCPP_ERROR(rclcpp::get_logger("explore_server"),"Action server or service is not available.");
            return;
        }

        //set region boundary on costmap
        if(rclcpp::ok() && goal_handle->is_active()){
            auto srv_req = std::make_shared<frontier_msgs::srv::UpdateBoundaryPolygon::Request>();
            srv_req->explore_boundary = goal->explore_boundary;
            auto resultBoundaryPolygon = updateBoundaryPolygon->async_send_request(srv_req);
            // RCLCPP_INFO(rclcpp::get_logger("explore_server"),"TempUpdate Boundary Polygon result recieved.");
            std::future_status fstatus = resultBoundaryPolygon.wait_for(std::chrono::seconds(10));
            if(fstatus == std::future_status::ready){
                RCLCPP_INFO(rclcpp::get_logger("explore_server"),"Region boundary set");
            }else{
                RCLCPP_ERROR(rclcpp::get_logger("explore_server"),"Failed to set region boundary");
                goal_handle->abort(res);
                return;
            }
        }

        //loop until all frontiers are explored
        // rclcpp::Rate rate(frequency_);
        while(rclcpp::ok() && goal_handle->is_active()){
            RCLCPP_INFO_STREAM(rclcpp::get_logger("explore_server"), "THE CHECKPOINT IS 0!!!!");
            auto srv_req = std::make_shared<frontier_msgs::srv::GetNextFrontier::Request>();
            auto srv_res = std::make_shared<frontier_msgs::srv::GetNextFrontier::Response>();

            //placeholder for next goal to be sent to move base
            geometry_msgs::msg::PoseStamped goal_pose;

            //get current robot pose in frame of exploration boundary
            geometry_msgs::msg::PoseStamped robot_pose;
            explore_costmap_ros_->getRobotPose(robot_pose);
            srv_req->start_pose = robot_pose;

            //evaluate if robot is within exploration boundary using robot_pose in boundary frame
            geometry_msgs::msg::PoseStamped eval_pose = srv_req->start_pose;
            if(eval_pose.header.frame_id != goal->explore_boundary.header.frame_id){
                RCLCPP_ERROR(rclcpp::get_logger("explore_server"), "Frames not same");
                tf_listener_->transform(srv_req->start_pose, eval_pose,goal->explore_boundary.header.frame_id);
            }

            auto resultNextFrontier = getNextFrontier->async_send_request(srv_req);
            std::future_status fstatus = resultNextFrontier.wait_for(std::chrono::seconds(10));
            srv_res = resultNextFrontier.get();
            RCLCPP_INFO_STREAM(rclcpp::get_logger("explore_server"), "The result of the getNext frontier service is: " << srv_res->next_frontier.pose.position.x << "," << srv_res->next_frontier.pose.position.y);
            //check if robot is not within exploration boundary and needs to return to center of search area
            if(goal->explore_boundary.polygon.points.size() > 0 && !pointInPolygon(eval_pose.pose.position,goal->explore_boundary.polygon)){
                RCLCPP_INFO_STREAM(rclcpp::get_logger("explore_server"), "THE CHECKPOINT IS 1!!!!");
                //check if robot has explored at least one frontier, and promote debug message to warning
                if(success_){
                    RCLCPP_ERROR(rclcpp::get_logger("explore_server"),"Robot left exploration boundary, returning to center");
                }else{
                    RCLCPP_ERROR(rclcpp::get_logger("explore_server"),"Robot not initially in exploration boundary, traveling to center");
                }
                //get current robot position in frame of exploration center
                geometry_msgs::msg::PointStamped eval_point;
                eval_point.header = eval_pose.header;
                eval_point.point = eval_pose.pose.position;
                if(eval_point.header.frame_id != goal->explore_center.header.frame_id){
                    geometry_msgs::msg::PointStamped temp = eval_point;
                    RCLCPP_ERROR(rclcpp::get_logger("explore_server"), "Frames not same");
                    tf_listener_->transform(temp, eval_point, goal->explore_center.header.frame_id);
                }

                //set goal pose to exploration center
                goal_pose.header = goal->explore_center.header;
                goal_pose.pose.position = goal->explore_center.point;
                goal_pose.pose.orientation = FrontierExplorationServer::createQuaternionMsgFromYaw(yawOfVector(eval_point.point, goal->explore_center.point));
            }
            else if(srv_res->success == true){
                RCLCPP_INFO_STREAM(rclcpp::get_logger("explore_server"), "THE CHECKPOINT IS 2!!!!");
                success_ = true;
                goal_pose = srv_res->next_frontier;
            }
            else if(srv_res->success == false){ //if no frontier found, check if search is successful
                RCLCPP_INFO_STREAM(rclcpp::get_logger("explore_server"), "THE CHECKPOINT IS 3!!!!");
                // RCLCPP_INFO(rclcpp::get_logger("explore_server"),"Couldn't find a frontier");

                //search is succesful
                if(retry_ == 0 && success_){
                    RCLCPP_WARN(rclcpp::get_logger("explore_server"),"Finished exploring room");
                    goal_handle->succeed(res);
                    std::unique_lock<std::mutex> lock(nav2Clientlock_);
                    nav2Client_->async_cancel_goals_before(rclcpp::Clock().now());
                    return;

                }else if(retry_ == 0 || !rclcpp::ok()){ //search is not successful

                    RCLCPP_ERROR(rclcpp::get_logger("explore_server"),"Failed exploration");
                    goal_handle->abort(res);
                    return;
                }

                RCLCPP_INFO(rclcpp::get_logger("explore_server"),"Retrying...");
                retry_--;
                //try to find frontier again, without moving robot
                if(retry_ <= 7){
                    performBackupAction();
                    rclcpp::Rate(2).sleep();
                }
                continue;
            }
            //if above conditional does not escape this loop step, search has a valid goal_pose

            //check if new goal is close to old goal, hence no need to resend
            RCLCPP_INFO_STREAM(rclcpp::get_logger("explore_server"),"Moving is: " << moving_);
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("explore_server"),"The old_goal x is: " << old_goal.pose.pose.position.x);
            RCLCPP_INFO_STREAM(rclcpp::get_logger("explore_server"),"The goal_pose x is: " << goal_pose.pose.position.x);
            RCLCPP_INFO_STREAM(rclcpp::get_logger("explore_server"),"Nearby? is: " << pointsNearby(old_goal.pose.pose.position,goal_pose.pose.position,goal_aliasing_*0.5));
            // if((!moving_ || !pointsNearby(old_goal.pose.pose.position,goal_pose.pose.position,goal_aliasing_*0.5)) && srv_res->success == true){
            if((!moving_ || !pointsNearby(old_goal.pose.pose.position,goal_pose.pose.position,goal_aliasing_*0.5)) && srv_res->success == true){
                // RCLCPP_INFO_STREAM(rclcpp::get_logger("explore_server"), "THE CHECKPOINT IS 4!!!!" << goal_pose.pose.position.x << "," << goal_pose.pose.position.y);

                // This is to check if old pose is near new pose.
                old_goal.pose = goal_pose;
                // auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
                // send_goal_options.feedback_callback = std::bind(&FrontierExplorationServer::feedbackMovingCb, this, std::placeholders::_1);
                // send_goal_options.result_callback = std::bind(&FrontierExplorationServer::doneMovingCb, this, std::placeholders::_1);
                // auto goal_handle_future = nav2Client_->async_send_goal(old_goal, send_goal_options);
                nav2Publisher_->publish(goal_pose);
                moving_ = true;
                int waitCount_ = 0;
                bool minuteWait = false;
                while(moving_ && minuteWait == false){
                    rclcpp::Rate(1).sleep();
                    waitCount_++;
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("explore_server"),"Wait count goal is: " << waitCount_);
                    if( waitCount_ > nav2WaitTime_){
                        minuteWait = true;
                    }
                }
                // Moving is made false here to treat this as equal to nav2 feedback saying goal is reached.
                moving_ = false;
                // lock.unlock();
            }
        }

        //goal should never be active at this point
        // ROS_ASSERT(!goal_handle->is_active());
        if(!goal_handle->is_active()){
            RCLCPP_ERROR(rclcpp::get_logger("explore_server"),"Goal Active when it should not be.");
        }
    }

    void performBackupAction(){
        RCLCPP_INFO(rclcpp::get_logger("explore_server"),"Frontier Exploration backup");
        geometry_msgs::msg::PoseStamped robot_pose;
        explore_costmap_ros_->getRobotPose(robot_pose);
        tf2::Quaternion quaternion;
        tf2::fromMsg(robot_pose.pose.orientation, quaternion);
        double current_yaw = tf2::getYaw(quaternion);
        double desired_rad = M_PI / 2;
        double new_yaw = current_yaw + desired_rad;
        quaternion.setRPY(0, 0, new_yaw);
        geometry_msgs::msg::PoseStamped new_pose;
        new_pose.header.frame_id = robot_pose.header.frame_id;
        new_pose.pose.position = robot_pose.pose.position;
        new_pose.pose.orientation = tf2::toMsg(quaternion);
        RCLCPP_INFO(rclcpp::get_logger("explore_server"),"Rotating in place to get new frontiers");
        nav2Publisher_->publish(new_pose);
        moving_ = true;
        int waitCount_ = 0;
        bool minuteWait = false;
        while(moving_ && minuteWait == false){
            rclcpp::WallRate(1.0).sleep();
            RCLCPP_INFO(rclcpp::get_logger("explore_server"),"Waiting to finish moving while rotating");
            waitCount_++;
            RCLCPP_INFO_STREAM(rclcpp::get_logger("explore_server"),"Wait count rotation is: " << waitCount_);
            // TODO: Assign different wait times for goal and rotate.
            if( waitCount_ > (nav2WaitTime_ / 3)){
                minuteWait = true;
            }
        }
        // Moving is made false here to treat this as equal to nav2 feedback saying goal is reached.
        moving_ = false;
    }


    /**
     * @brief Preempt callback for the server, cancels the current running goal and all associated movement actions.
     */

    /**
     * @brief Feedback callback for the move_base client, republishes as feedback for the exploration server
     * @param feedback Feedback from the move_base client
     */
    void feedbackMovingCb(GoalHandleNav2::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback){

        auto feedback_ = std::make_shared<NavigateToPose::Feedback>();
        feedback_->current_pose = feedback->current_pose;
        // action_server_->publish_feedback(feedback_);
    }

    /**
     * @brief Done callback for the move_base client, checks for errors and aborts exploration task if necessary
     * @param state State from the move_base client
     * @param result Result from the move_base client
     */
    void doneMovingCb(const GoalHandleNav2::WrappedResult & result){

        if (result.code == rclcpp_action::ResultCode::ABORTED){
            RCLCPP_ERROR(rclcpp::get_logger("explore_server"),"Failed to move");
            // action_server_.setAborted();
        }else if(result.code == rclcpp_action::ResultCode::SUCCEEDED){
            moving_ = false;
        }
    }

    void feedbackNav2cb(const geometry_msgs::msg::PointStamped::SharedPtr status){
        if(status->point.x == 10.0){
            moving_ = false;
            // RCLCPP_ERROR(rclcpp::get_logger("explore_server"),"Moving completed");
        }
        else if(status->point.x == 0.0){
            // RCLCPP_INFO(rclcpp::get_logger("explore_server"),"Currently moving");
        }
    }

};

}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<frontier_exploration::FrontierExplorationServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}