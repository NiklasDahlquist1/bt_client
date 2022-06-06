

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






































// test client

    Client_uav::Client_uav()
    {
        // setup subscribers
        odom_sub = nodeHandle.subscribe("odom", 1000, &Client_uav::odomCB, this);
        path_sub = nodeHandle.subscribe("path", 1000, &Client_uav::pathCB, this);
        //
        state.goalPoint_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("goalPose", 1000);
        state.setGoalPathPlanner_pub = nodeHandle.advertise<geometry_msgs::Point>("setPathGoal", 1000);


        std::cout << "client started" << std::endl;
        initFactory(factory);

        pathCostServiceClient = nodeHandle.serviceClient<dsp::pathCost>("dsp/path_cost");
    }

/*
    double Client_uav::costForTask(auction_msgs::task task)
    {
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

            return cost;
        }
        else if(task.task_name == "moveTo2D")
        {
            geometry_msgs::Point goal; // check number of args? 
            auto parts = BT::splitString(task.task_data, ';');
            goal.x = BT::convertFromString<double>(parts[0]);
            goal.y = BT::convertFromString<double>(parts[1]);
            goal.z = BT::convertFromString<double>(parts[2]);


            // copmute cost, based on current state and goal pos

//           double cost = pow((state.currentPose.position.x - goal.x), 2) + 
//                         pow((state.currentPose.position.y - goal.y), 2);


            double cost;
            dsp::pathCost srv;
            srv.request.start = state.currentPose.position;
            srv.request.start.z = 0;
            //std::cout << "start: " << state.currentPose.position << std::endl;
            srv.request.stop = goal;
            srv.request.stop.z = 0;
            //std::cout << "stop: " << goal << std::endl;
            

            if (pathCostServiceClient.call(srv))
            {
                //std::cout << "Cost: " << srv.response.cost << std::endl;
                cost = srv.response.cost;

                if(cost > 1e9)
                {
                    cost = -1;
                }
            }
            else
            {
                std::cout << "Failed to call service path_cost" << std::endl;
                cost = -1;
                //return 1;
            }

            std::cout << "Client: " << name <<  " Cost: " << srv.response.cost << std::endl;

            return cost;

        }
        else
        {
            return -1;
        }
    }

    */


    void Client_uav::costForTasks(const std::vector<auction_msgs::task>& tasks, std::vector<auction_msgs::price_bid>& pricesToFill)
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
            else if(task.task_name == "moveTo3D")
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
            else if(task.task_name == "moveTo2D")
            {
                geometry_msgs::Point goal; // check number of args? 
                auto parts = BT::splitString(task.task_data, ';');
                goal.x = BT::convertFromString<double>(parts[0]);
                goal.y = BT::convertFromString<double>(parts[1]);
                goal.z = BT::convertFromString<double>(parts[2]);


                // copmute cost, based on current state and goal pos

               /*double distance2D = pow((state.currentPose.position.x - goal.x), 2) + 
                             pow((state.currentPose.position.y - goal.y), 2);

                if(distance2D < 1)
                {
                    priceForTask = 0.1; // TODO, handle (hack for now, dsp seems to not work very close to the point?)
                    break;
                }*/


                double cost;
                dsp::pathCost srv;
                srv.request.start = state.currentPose.position;
                srv.request.start.z = 0;
                //std::cout << "start: " << state.currentPose.position << std::endl;
                srv.request.stop = goal;
                srv.request.stop.z = 0;
                //std::cout << "stop: " << goal << std::endl;
                

                if (pathCostServiceClient.call(srv))
                {
                    //std::cout << "Cost: " << srv.response.cost << std::endl;
                    cost = srv.response.cost;

                    if(cost > 1e9)
                    {
                        cost = -1;
                    }
                }
                else
                {
                    std::cout << "Failed to call service path_cost" << std::endl;
                    cost = -1;
                    //return 1;
                }
 
        
                    
                //std::cout << "Client: " << name <<  " Cost: " << srv.response.cost << std::endl;

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



    std::string Client_uav::getXMLForTask(auction_msgs::task task)
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


            state.setGoalPathPlanner_pub.publish(goal);

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
        else if(task.task_name == "moveTo2D")
        {
            // setup task variables, should be removed and merged into the tree? TODO
            geometry_msgs::Point goal; // check number of args?
            auto parts = BT::splitString(task.task_data, ';');
            goal.x = BT::convertFromString<double>(parts[0]);
            goal.y = BT::convertFromString<double>(parts[1]);
            //goal.z = BT::convertFromString<double>(parts[2]);
            goal.z = 2;

            this->state.goalPoint = goal;

            std::cout << "Got new task, moveTo2D. Parameters: " << goal.x << "," << goal.y  << std::endl;


            state.setGoalPathPlanner_pub.publish(goal);

            //generate XML here
            static const char* xml_text = R"(
            <root main_tree_to_execute = "MainTree" >
                <BehaviorTree ID="MainTree">
                    <ReactiveFallback name="root">
                        <Action ID="UAVAtPoint2D"/>
                        <Action ID="FollowPath"/>
                    </ReactiveFallback>
                </BehaviorTree>
            </root>
            )";


            return xml_text;
        }
        else if(task.task_name == "moveTo3D")
        {
            // setup task variables, should be removed and merged into the tree? TODO
            geometry_msgs::Point goal; // check number of args?
            auto parts = BT::splitString(task.task_data, ';');
            goal.x = BT::convertFromString<double>(parts[0]);
            goal.y = BT::convertFromString<double>(parts[1]);
            goal.z = BT::convertFromString<double>(parts[2]);
            //goal.z = 0;

            this->state.goalPoint = goal;

            std::cout << "Got new task, moveTo3D. Parameters: " << goal.x << ", " << goal.y << ", "<< goal.z << std::endl;


            state.setGoalPathPlanner_pub.publish(goal);

            //generate XML here
            static const char* xml_text = R"(
            <root main_tree_to_execute = "MainTree" >
                <BehaviorTree ID="MainTree">
                    <ReactiveFallback name="root">
                        <Action ID="UAVAtPoint"/>
                        <Action ID="Follow3DPath"/>
                    </ReactiveFallback>
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
    std::string Client_uav::getXMLNoTask()
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
    void Client_uav::initNodes(BT::Tree& tree)
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
            if(auto followPath = dynamic_cast<behaviors::FollowPath*>( node.get()))
            {
                followPath->init(&state);
            }
            if(auto follow3DPath = dynamic_cast<behaviors::Follow3DPath*>( node.get()))
            {
                follow3DPath->init(&state);
            }

            if(auto uAVAtPoint2D = dynamic_cast<behaviors::UAVAtPoint2D*>( node.get()))
            {
                uAVAtPoint2D->init(&state);
            }
        }
    }
    void Client_uav::initFactory(BT::BehaviorTreeFactory& factory)
    {
        factory.registerNodeType<behaviors::MoveToGoalPoint>("moveToGoalPoint");
        factory.registerNodeType<behaviors::UAVAtPoint>("UAVAtPoint");
        factory.registerNodeType<behaviors::FollowPath>("FollowPath");
        factory.registerNodeType<behaviors::UAVAtPoint2D>("UAVAtPoint2D");
        factory.registerNodeType<behaviors::Follow3DPath>("Follow3DPath");

        //std::cout << "init nodes" << std::endl;


        return;
    }




    void Client_uav::odomCB(const nav_msgs::Odometry& msg)
    {
        state.currentPose = msg.pose.pose;
        //std::cout << "odom callback" << std::endl;
    }
    void Client_uav::pathCB(const nav_msgs::Path& msg)
    {
        state.path = msg;
        state.pathIndex = 0;
    }








} // namespace auction_ns












