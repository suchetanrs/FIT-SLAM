// core_plugins.hpp
#pragma once

#include <roadmap_explorer/bt_plugins/interface_pluginlib.hpp>
#include "roadmap_explorer/util/Logger.hpp"
#include "roadmap_explorer/util/EventLogger.hpp"

#include "fit_slam2_recovery/recovery_controller.hpp"

namespace roadmap_explorer
{
    class RecoveryControllerBTPlugin : public BTPlugin
    {
        public:
        RecoveryControllerBTPlugin();

        ~RecoveryControllerBTPlugin();

        void registerNodes(BT::BehaviorTreeFactory & factory, std::shared_ptr<nav2_util::LifecycleNode> node, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros, std::shared_ptr<tf2_ros::Buffer> tf_buffer) override;
    };
};