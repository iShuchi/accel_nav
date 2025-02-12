#include <behaviortree_cpp_v3/behavior_tree.h>
#include <iostream>
#include <tree_node.h>

class LidarSensor : public BT::SyncActionNode
{
public:
    LidarSensor(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {BT::OutputPort<float>("lidar_range")};
    }

    BT::NodeStatus tick() override
    {
        float lidar_range = 1.2f;              
        setOutput("lidar_range", lidar_range); 

        return BT::NodeStatus::SUCCESS;
    }
};

class ObstacleDetection : public BT::SyncActionNode
{
public:
    ObstacleDetection(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<float>("lidar_range")};
    }

    BT::NodeStatus tick() override
    {
        float lidar_range = 0.0f;
        if (!getInput("lidar_range", lidar_range))
        {
            return BT::NodeStatus::FAILURE;
        }

        if (lidar_range < 1.0f)
        {
            std::cout << "Obstacle detected!" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        std::cout << "No obstacle detected." << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

int main()
{
    auto blackboard = BT::Blackboard::create();
    BT::Tree tree;
    tree.setBlackboard(blackboard);

    BehaviorTreeFactory factory;
    factory.registerNodeType<LidarSensor>("LidarSensor");
    factory.registerNodeType<ObstacleDetection>("ObstacleDetection");

    BT::TreeNode *root_node = factory.createTreeFromFile("tree.xml");

    tree.tickRoot();
}
