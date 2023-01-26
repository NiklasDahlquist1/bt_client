

#include "bt_client.hpp"

#include <chrono>
#include <unistd.h>
#include <iostream>

#include "behaviors_example.h"



namespace auction_ns
{

    Auction_client_bt::Auction_client_bt() //: Auction_client()
    {
        std::cout << "Auction_client_bt" << std::endl;
        //initFactory(factory);
        
        //taskXML = getXMLNoTask();
        //taskTree = factory.createTreeFromText(taskXML);
        //initNodes(taskTree);
        

        
        //maybe add setup for xml generation here? TODO
        
    }


    void Auction_client_bt::handleNewTask(auction_msgs::task task) // swtitch to bt for task, initialize new tree nodes, etc.
    {
        taskCurrentStatus = WORKING; // ?
        taskXML = getXMLForTask(task);
        taskTree = factory.createTreeFromText(taskXML);
        initNodes(taskTree);
    }
    void Auction_client_bt::handleNoTask() // switch to no task bt
    {
        taskCurrentStatus = WORKING; // ?
        taskXML = getXMLNoTask();
        taskTree = factory.createTreeFromText(taskXML);
        initNodes(taskTree);
    }
    void Auction_client_bt::executeCurrentBehavior() // tick bt, also check if task is completed
    {
        BT::NodeStatus status;
        status = taskTree.tickRoot(); //tick behavior tree


        //std::cout << "Tree status: " << status << std::endl;

        // TODO, maybe make better?
        if(status == BT::NodeStatus::SUCCESS)
        {
            taskCurrentStatus = COMPLETED;
        }
        else if(status == BT::NodeStatus::FAILURE)
        {
            taskCurrentStatus = FAILED;
        }
        else if(status == BT::NodeStatus::RUNNING)
        {
            taskCurrentStatus = WORKING;
        }
    }
    //TODO, remove. see header
    void Auction_client_bt::public_execute_behavior()
    {
        executeCurrentBehavior();
    }
    Auction_client::taskStatus Auction_client_bt::currentTaskStatus() // check if task is completed, (maybe return bt status? in this case)
    {
        return taskCurrentStatus;
    }
    /*double Auction_client_bt::costForTask(auction_msgs::task task)
    {
        std::cout << "cost for task function not implemented" << std::endl;
        return -1;
    }*/
    void Auction_client_bt::costForTasks(const std::vector<auction_msgs::task>& tasks, std::vector<auction_msgs::price_bid>& pricesToFill)
    {
        return;
    }

    void Auction_client_bt::initFactory(BT::BehaviorTreeFactory& factory)
    {
        return;
    }


    std::string Auction_client_bt::getXMLForTask(auction_msgs::task task)
    {
        if(task.task_name == "task1")
        {
            return "task tree";
        }

        
        std::cout << "Could not find XML for that task, returning no task tree..." << std::endl;
        return getXMLNoTask();
    }
    std::string Auction_client_bt::getXMLNoTask()
    {
        return "xml no task";
    }
    void Auction_client_bt::initNodes(BT::Tree& tree)
    {

    }



















} // namespace auction_ns












