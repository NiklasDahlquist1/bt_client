

#include "client_mbzirc_hex.hpp"

#include <chrono>
#include <unistd.h>
#include <iostream>

#include "behaviors_mbzirc.h"



namespace auction_ns
{







    Client_mbzirc_hex::Client_mbzirc_hex()
    {
        // setup subscribers
        odom_sub = nodeHandle.subscribe("odom", 1000, &Client_mbzirc_hex::odomCB, this);
        //
        state.goalPoint_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("goalPose", 1000);


        std::cout << "client started" << std::endl;
        initFactory(factory);

    }







    void Client_mbzirc_hex::costForTasks(const std::vector<auction_msgs::task>& tasks, std::vector<auction_msgs::price_bid>& pricesToFill)
    {
        auto start = std::chrono::steady_clock::now();
        
        for(auction_msgs::task task : tasks)
        {
            double priceForTask = -1;
            
            if(task.task_name == "moveTo")
            {
                geometry_msgs::Point goal; // check number of args?
                auto parts = BT::splitString(task.task_data, ';');
                goal.x = BT::convertFromString<double>(parts[0]);
                goal.y = BT::convertFromString<double>(parts[1]);
                goal.z = BT::convertFromString<double>(parts[2]);

                
                // copmute cost, based on current state and goal pos

                double cost = pow((state.currentPose.position.x - goal.x), 2) + 
                            pow((state.currentPose.position.y - goal.y), 2) + 
                            pow((state.currentPose.position.z - goal.z), 2);
                priceForTask = cost;
            }
            else if(task.task_name == "test_communication")
            {
                /*
                task: 
                    move to point
                    send data

                    point (x,y,z)
                    data size
                    num of msgs
                    target string
                */
                geometry_msgs::Point goal; // check number of args?
                auto parts = BT::splitString(task.task_data, ';');
                goal.x = BT::convertFromString<double>(parts[0]);
                goal.y = BT::convertFromString<double>(parts[1]);
                goal.z = BT::convertFromString<double>(parts[2]);

                int packet_size = BT::convertFromString<int>(parts[3]);
                int packet_num = BT::convertFromString<int>(parts[4]);
                double msgs_per_sec = BT::convertFromString<int>(parts[5]);
                std::string target_name = BT::convertFromString<std::string>(parts[6]);

                
                // copmute cost, based on current state and goal pos 

                double cost = pow((state.currentPose.position.x - goal.x), 2) + 
                            pow((state.currentPose.position.y - goal.y), 2) + 
                            pow((state.currentPose.position.z - goal.z), 2);
                priceForTask = cost;
            }
            else
            {
                priceForTask = -1;
            }

            //TODO: maybe move this type of logic to the auction server? or is it good here?
            if(this->currentTask.task == task)
            {
                priceForTask = priceForTask * 0.8;
            }



            auction_msgs::price_bid p;
            p.task_ID = task.task_ID;
            p.price = priceForTask;



            pricesToFill.push_back(p);
        }
        auto end = std::chrono::steady_clock::now();
        //std::cout << "Elapsed time in milliseconds: "
        //          << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
        //          << " ms" << std::endl;
    }



    std::string Client_mbzirc_hex::getXMLForTask(auction_msgs::task task)
    {
        if(task.task_name == "moveTo")
        {
            // setup task variables, should be removed and merged into the tree? TODO
            geometry_msgs::Point goal; // check number of args?
            auto parts = BT::splitString(task.task_data, ';');
            goal.x = BT::convertFromString<double>(parts[0]);
            goal.y = BT::convertFromString<double>(parts[1]);
            goal.z = BT::convertFromString<double>(parts[2]);

            this->state.goalPoint = goal;

            std::cout << "Got new task, moveTo. Parameters: " << goal.x << "," << goal.y << "," << goal.z  << std::endl;



            //generate XML here
            static const char* xml_text = R"(
            <root main_tree_to_execute = "MainTree" >
                <BehaviorTree ID="MainTree">
                    <ReactiveFallback name="root">
                        <Action ID="UAVAtPoint"/>
                        <Action ID="moveToGoalPoint"/>
                    </ReactiveFallback>
                </BehaviorTree>
            </root>
            )";


            return xml_text;
        }
        else if(task.task_name == "test_communication")
        {
            /*
            task: 
                move to point
                send data

                point (x,y,z)
                data size
                num of msgs
                msgs per second
                target string
            */
            geometry_msgs::Point goal; // check number of args?
            auto parts = BT::splitString(task.task_data, ';');
            goal.x = BT::convertFromString<double>(parts[0]);
            goal.y = BT::convertFromString<double>(parts[1]);
            goal.z = BT::convertFromString<double>(parts[2]);

            int packet_size = BT::convertFromString<int>(parts[3]);
            int packet_num = BT::convertFromString<int>(parts[4]);
            double msgs_per_sec = BT::convertFromString<int>(parts[5]);
            std::string target_name = BT::convertFromString<std::string>(parts[6]);

                

            this->state.goalPoint = goal;
            this->state.packet_size = packet_size;
            this->state.packet_num = packet_num;
            this->state.target_name = target_name;
            this->state.msgs_per_sec = msgs_per_sec;

            std::cout << "Got new task, test_communication. Parameters: " << goal.x << "," << goal.y << "," << goal.z << std::endl;



            //generate XML here
            static const char* xml_text = R"(
            <root main_tree_to_execute = "MainTree" >
                <BehaviorTree ID="MainTree">
                    <ReactiveSequence>
                        <ReactiveFallback>
                            <Action ID="UAVAtPoint"/>
                            <Action ID="moveToGoalPoint"/>
                        </ReactiveFallback>

                        <ReactiveFallback>
                            <Action ID="SendData"/>
                        </ReactiveFallback>
                    <ReactiveSequence>
                </BehaviorTree>
            </root>
            )";


            return xml_text;
        }
        else
        {
            return "";
        }
    }
    std::string Client_mbzirc_hex::getXMLNoTask()
    {
        std::cout << "get xml for no task" << std::endl;
        // setup task variables, should be removed and merged into the tree? TODO
        //if(state.goalIsSet == false)
        //{
            this->state.goalPoint = state.currentPose.position;
        //    state.goalIsSet = true;
        //}
        std::cout << "goal no task: " << state.goalPoint.x << ", " << state.goalPoint.y << ", " << state.goalPoint.z << std::endl;

        // TODO ???
        //state.goalPoint.z = 2;
        state.setGoalPathPlanner_pub.publish(state.goalPoint);

        static const char* xml_text = R"(
            <root main_tree_to_execute = "MainTree" >
                <BehaviorTree ID="MainTree">
                    <ReactiveFallback name="root">
                        <Action ID="UAVAtPoint"/>
                        <Action ID="moveToGoalPoint"/>
                    </ReactiveFallback>
                </BehaviorTree>
            </root>
            )";
        return xml_text;
    }
    void Client_mbzirc_hex::initNodes(BT::Tree& tree)
    {
        // Iterate through all the nodes and call init() if it is an Action_B
        for( auto& node: tree.nodes )
        {
            // Not a typo: it is "=", not "=="
            if(auto moveToGoalPoint = dynamic_cast<behaviors::MoveToGoalPoint*>( node.get()))
            {
                moveToGoalPoint->init(&state);
            }
            if(auto uAVAtPoint = dynamic_cast<behaviors::UAVAtPoint*>( node.get()))
            {
                uAVAtPoint->init(&state);
            }            

        }
    }
    void Client_mbzirc_hex::initFactory(BT::BehaviorTreeFactory& factory)
    {
        factory.registerNodeType<behaviors::MoveToGoalPoint>("moveToGoalPoint");
        factory.registerNodeType<behaviors::UAVAtPoint>("UAVAtPoint");


        //std::cout << "init nodes" << std::endl;


        return;
    }




    void Client_mbzirc_hex::odomCB(const nav_msgs::Odometry& msg)
    {
        state.currentPose = msg.pose.pose;
        //std::cout << "odom callback" << std::endl;
    }








} // namespace auction_ns












