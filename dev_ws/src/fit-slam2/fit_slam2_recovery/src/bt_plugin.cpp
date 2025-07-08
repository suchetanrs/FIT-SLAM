#include "fit_slam2_recovery/bt_plugin.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace roadmap_explorer
{
    class RecoveryControllerBT : public BT::StatefulActionNode
    {
    public:
        RecoveryControllerBT(
            const std::string & name, const BT::NodeConfiguration & config,
            std::shared_ptr<nav2_util::LifecycleNode> node, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros, std::shared_ptr<tf2_ros::Buffer> tf_buffer)
        : BT::StatefulActionNode(name, config)
        {
            recovery_controller_ = std::make_shared<RecoveryController>(explore_costmap_ros, node, tf_buffer);
            LOG_INFO("RecoveryControllerBT Constructor");
        }

        BT::NodeStatus onStart() override
        {
            LOG_FLOW("RecoveryControllerBT onStart");
            bool backward_recovery_only;
            getInput<bool>("backward_recovery_only", backward_recovery_only);
            recovery_controller_->computeVelocityCommand(backward_recovery_only);
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus onRunning()
        {
            return BT::NodeStatus::SUCCESS;
        }

        void onHalted()
        {
            LOG_WARN("RecoveryControllerBT onHalted");
            return;
        }

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<bool>("backward_recovery_only")
            };
        }

        std::shared_ptr<RecoveryController> recovery_controller_;
    };

    RecoveryControllerBTPlugin::RecoveryControllerBTPlugin()
    {
    }

    RecoveryControllerBTPlugin::~RecoveryControllerBTPlugin()
    {
    }

    void RecoveryControllerBTPlugin::registerNodes(BT::BehaviorTreeFactory &factory, std::shared_ptr<nav2_util::LifecycleNode> node, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros, std::shared_ptr<tf2_ros::Buffer> tf_buffer)
    {
        BT::NodeBuilder builder_recovery_controller =
        [explore_costmap_ros, node, tf_buffer](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<RecoveryControllerBT>(name, config, node, explore_costmap_ros, tf_buffer);
        };
        factory.registerBuilder<RecoveryControllerBT>("ExecuteRecovery", builder_recovery_controller);
    }
}

PLUGINLIB_EXPORT_CLASS(
    roadmap_explorer::RecoveryControllerBTPlugin,
    roadmap_explorer::BTPlugin)