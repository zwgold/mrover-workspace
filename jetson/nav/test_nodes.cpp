#include "test_nodes.h"
#include "time.h"


namespace TestNodes
{

std::unordered_map<std::string, int> drinkTable;

BT::NodeStatus CheckMachineStatus(){
    srand( time ( NULL ) );
    int r = rand( ) % 10;
    if (r == 0) std::cout << "MACHINE BROKEN" << std::endl;
    else std::cout << "MACHINE OK" << std::endl;
    return ((r == 0) ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS);
}

void fillDrinkTable(){
    drinkTable["coke"] = 5;
    drinkTable["sprite"] = 5;
}

BT::NodeStatus CheckOverallDrinkAvail(){
    for (auto itr : drinkTable){
        if (itr.second > 0) {
            std::cout << "Drinks available" << '\n';
            return BT::NodeStatus::SUCCESS;
        }
    }
    std::cout << "NO DRINKS LEFT :(" << '\n';
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus PrintOptions(){
    for (auto itr : drinkTable){
        std::cout << itr.first << " : " << itr.second << std::endl;
    }
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus DispenseDrink::tick()
{
    auto msg = getInput<std::string>("drink_port");
    if (!msg)
    {
        throw BT::RuntimeError( "missing required input [drink]: ", msg.error() );
    }
    std::string drink = msg.value();
    std::cout << "Dispensing Drink" << std::endl;
    drinkTable[drink] -= 1;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CheckSpace::tick()
{
    auto msg = getInput<std::string>("drink_port");
    if (!msg)
    {
        throw BT::RuntimeError( "missing required input [drink]: ", msg.error() );
    }
    std::string drink = msg.value();
    //std::cout << drink << std::endl;
    bool hasDrink = (drinkTable.find(drink) != drinkTable.end());
    std::string message = (hasDrink ? "Drink Found" : "Drink not available");
    std::cout << message << std::endl;
    return (hasDrink ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE);
}

// This Action simply write a value in the port "text"
BT::NodeStatus Prompt::tick() 
{
    std::cout << "enter your drink choice: ";
    std::string choice;
    std::cin >> choice;
    std::cout << std::endl;
    setOutput("drink_port", choice );
    return BT::NodeStatus::SUCCESS;
}


}
