#include "behaviortree_cpp/bt_factory.h"

using namespace BT;
// clang-format on

class SaySomething : public BT::StatefulActionNode
{
public:
    SaySomething(const std::string &name, const BT::NodeConfig &config, int num) : BT::StatefulActionNode(name, config)
    {
        std::cout << "SaySomething Constructor" << std::endl;
    }

    BT::NodeStatus onStart() override
    {
        std::string msg;
        getInput("message", msg);
        std::cout << msg << std::endl;
        // config().blackboard->get<int>("message", strrr);
        // std::cout << strrr.num << std::endl;
        return BT::NodeStatus::RUNNING;
    }
    
    BT::NodeStatus onRunning()
    {
        std::cout << "On running called " << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted()
    {
        return;
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>("message")};
    }
};

class ThinkWhatToSay : public BT::SyncActionNode
{
public:
    ThinkWhatToSay(const std::string &name, const BT::NodeConfig &config) : BT::SyncActionNode(name, config)
    {
        std::cout << "ThinkWhatToSay Constructor" << std::endl;
    }

    BT::NodeStatus tick() override
    {
        setOutput("text", "The answer is 42");
        // NumText structt;
        // config().blackboard->set<NumText>("message", structt);
        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts()
    {
        return {BT::OutputPort<std::string>("text")};
    }
};

int main()
{

    BehaviorTreeFactory factory;
    BT::Blackboard::Ptr blackboard = BT::Blackboard::create();
    int num = 69;

    BT::NodeBuilder builder_cpds_srv =
        [&](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<SaySomething>(name, config, num);
    };

    factory.registerBuilder<SaySomething>("SaySomething", builder_cpds_srv);
    factory.registerNodeType<ThinkWhatToSay>("ThinkWhatToSay");

    auto tree = factory.createTreeFromFile("/root/dev_ws/src/behaviour_test/hello_hi_tree.xml", blackboard);

    tree.tickWhileRunning();
    return 0;
}