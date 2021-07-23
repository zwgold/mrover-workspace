#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "stateMachine.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "test_nodes.h"
using namespace rover_msgs;
using namespace std;
using namespace BT;

static const char* xml_text = R"(

 <root main_tree_to_execute = "MainTree" >

     <BehaviorTree ID="MainTree">
        <ReactiveSequence>
            <CheckOverallDrinkAvail/>
            <Sequence name="not_root_sequence">
                <Sequence name="root_sequence">
                    <Prompt   drink_port="{drink}"/>
                    <Fallback name="fallback">
                        <CheckSpace drink_port="{drink}"/>
                        <ForceFailure>
                            <PrintOptions name="print_options"/>
                        </ForceFailure>
                        <ForceFailure>
                            <Prompt drink_port="{drink}"/>
                        </ForceFailure>
                        <CheckSpace drink_port="{drink}"/>
                    </Fallback>
                    <DispenseDrink   drink_port="{drink}"/>
                </Sequence>
            </Sequence>
        </ReactiveSequence>
        
     </BehaviorTree>

 </root>
 )";

// This class handles all incoming LCM messages for the autonomous
// navigation of the rover.
class LcmHandlers
{
public:
    // Constructs an LcmHandler with the given state machine to work
    // with.
    LcmHandlers( StateMachine* stateMachine )
        : mStateMachine( stateMachine )
    {}

    // Sends the auton state lcm message to the state machine.
    void autonState(
        const lcm::ReceiveBuffer* recieveBuffer,
        const string& channel,
        const AutonState* autonState
        )
    {
        mStateMachine->updateRoverStatus( *autonState );
    }

    // Sends the course lcm message to the state machine.
    void course(
        const lcm::ReceiveBuffer* recieveBuffer,
        const string& channel,
        const Course* course
        )
    {
        mStateMachine->updateRoverStatus( *course );
    }

    // Sends the obstacle lcm message to the state machine.
    void obstacle(
        const lcm::ReceiveBuffer* receiveBuffer,
        const string& channel,
        const Obstacle* obstacle
        )
    {
        mStateMachine->updateRoverStatus( *obstacle );
    }

    // Sends the odometry lcm message to the state machine.
    void odometry(
        const lcm::ReceiveBuffer* recieveBuffer,
        const string& channel,
        const Odometry* odometry
        )
    {
        mStateMachine->updateRoverStatus( *odometry );
    }

    // Sends the target lcm message to the state machine.
    void targetList(
        const lcm::ReceiveBuffer* receiveBuffer,
        const string& channel,
        const TargetList* targetListIn
        )
    {
        mStateMachine->updateRoverStatus( *targetListIn );
    }

    // Sends the radio lcm message to the state machine.
    void radioSignalStrength(
        const lcm::ReceiveBuffer* receiveBuffer,
        const string& channel,
        const RadioSignalStrength* signalIn
        )
    {
        mStateMachine->updateRoverStatus( *signalIn );
    }

    // Updates Radio Repeater bool in state machine.
    void repeaterDropComplete(
        const lcm::ReceiveBuffer* receiveBuffer,
        const string& channel,
        const RepeaterDrop* completeIn
        )
    {
        mStateMachine->updateRepeaterComplete( );
    }

private:
    // The state machine to send the lcm messages to.
    StateMachine* mStateMachine;
};

// Runs the autonomous navigation of the rover.
int main()
{


    // We use the BehaviorTreeFactory to register our custom nodes
    BehaviorTreeFactory factory;

    /* There are two ways to register nodes:
    *    - statically, i.e. registering all the nodes one by one.
    *    - dynamically, loading the TreeNodes from a shared library (plugin).
    * */

    // Note: the name used to register should be the same used in the XML.
    // Note that the same operations could be done using DummyNodes::RegisterNodes(factory)
    std::cout << "static linking" << std::endl;
    using namespace TestNodes;

    fillDrinkTable();
    

    factory.registerSimpleCondition("CheckOverallDrinkAvail", std::bind(CheckOverallDrinkAvail));
    factory.registerSimpleCondition("CheckMachineStatus", std::bind(CheckMachineStatus));
    factory.registerSimpleAction("PrintOptions", std::bind(PrintOptions));
    factory.registerNodeType<Prompt>("Prompt");
    factory.registerNodeType<DispenseDrink>("DispenseDrink");
    factory.registerNodeType<CheckSpace>("CheckSpace");
    

    // Trees are created at deployment-time (i.e. at run-time, but only once at the beginning).
    // The currently supported format is XML.
    // IMPORTANT: when the object "tree" goes out of scope, all the TreeNodes are destroyed
    auto tree = factory.createTreeFromText(xml_text);

    // To "execute" a Tree you need to "tick" it.
    // The tick is propagated to the children based on the logic of the tree.
    // In this case, the entire sequence is executed, because all the children
    // of the Sequence return SUCCESS.
    BT::NodeStatus status = BT::NodeStatus::SUCCESS;
    while (status == BT::NodeStatus::SUCCESS){
        //lcm processing
        //map processing
        status = tree.tickRoot();    
    }








    lcm::LCM lcmObject;
    if( !lcmObject.good() )
    {
        cerr << "Error: cannot create LCM\n";
        return 1;
    }

    StateMachine roverStateMachine( lcmObject );
    LcmHandlers lcmHandlers( &roverStateMachine );

    lcmObject.subscribe( "/auton", &LcmHandlers::autonState, &lcmHandlers );
    lcmObject.subscribe( "/course", &LcmHandlers::course, &lcmHandlers );
    lcmObject.subscribe( "/obstacle", &LcmHandlers::obstacle, &lcmHandlers );
    lcmObject.subscribe( "/odometry", &LcmHandlers::odometry, &lcmHandlers );
    lcmObject.subscribe( "/radio", &LcmHandlers::radioSignalStrength, &lcmHandlers );
    lcmObject.subscribe( "/rr_drop_complete", &LcmHandlers::repeaterDropComplete, &lcmHandlers );
    lcmObject.subscribe( "/target_list", &LcmHandlers::targetList, &lcmHandlers );

    while( lcmObject.handle() == 0 )
    {
        roverStateMachine.run();
    }
    return 0;
} // main()
