// core_plugins.hpp
#pragma once

#include <roadmap_explorer/bt_plugins/interface_pluginlib.hpp>
#include "roadmap_explorer/util/Logger.hpp"
#include "roadmap_explorer/util/EventLogger.hpp"
#include <fisher_information_plugins/fisher_information/FisherInfoManager.hpp>
#include "fit_slam2_msgs/srv/mark_lethal.hpp"


namespace roadmap_explorer
{
    class FisherInfoBTPlugin : public BTPlugin
    {
        public:
        FisherInfoBTPlugin();

        ~FisherInfoBTPlugin();

        void registerNodes(BT::BehaviorTreeFactory & factory, std::shared_ptr<nav2_util::LifecycleNode> node, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros, std::shared_ptr<tf2_ros::Buffer> tf_buffer) override;
    };
};