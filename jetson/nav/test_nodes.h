#ifndef SIMPLE_TEST_NODES_H
#define SIMPLE_TEST_NODES_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <unordered_map>

namespace TestNodes
{
void fillDrinkTable();

// Example of custom SyncActionNode (synchronous action)
// with an input port.
class CheckSpace : public BT::SyncActionNode
{
  public:
    CheckSpace(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
    {
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("drink_port") };
    }
};



// Example of custom SyncActionNode (synchronous action)
// with an input port.
class DispenseDrink : public BT::SyncActionNode
{
  public:
    DispenseDrink(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
    {
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("drink_port") };
    }
};


class Prompt : public BT::SyncActionNode
{
  public:
    Prompt(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
    {
    }

    // This Action simply write a value in the port "text"
    BT::NodeStatus tick() override;

    // A node having ports MUST implement this STATIC method
    static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<std::string>("drink_port") };
    }
};

BT::NodeStatus CheckOverallDrinkAvail();
BT::NodeStatus PrintOptions();
BT::NodeStatus CheckMachineStatus();
} // end namespace

#endif   // SIMPLE_BT_NODES_H
