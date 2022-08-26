

#include "client_groundAgent.hpp"

#include <chrono>
#include <unistd.h>
#include <iostream>

#include "behaviors_ground.h"








namespace auction_ns
{




    Client_groundAgent::Client_groundAgent()
    {
        // setup subscribers
        odom_sub = nodeHandle.subscribe("odom", 1000, &Client_groundAgent::odomCB, this);
        path_sub = nodeHandle.subscribe("path", 1000, &Client_groundAgent::pathCB, this);
        //
        state.goalPoint_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("goalPose", 1000);
        state.setGoalPathPlanner_pub = nodeHandle.advertise<geometry_msgs::Point>("setPathGoal", 1000);


        std::cout << "ground client started" << std::endl;
        initFactory(factory);

        pathCostServiceClient = nodeHandle.serviceClient<dsp::pathCost>("dsp/path_cost");
        //this->state.setGoal_srv = nodeHandle.serviceClient<ros_turtlebot_control::MoveToPoint>("turtle/move_to_point");

    }



    void Client_groundAgent::costForTasks(const std::vector<auction_msgs::task>& tasks, std::vector<auction_msgs::price_bid>& pricesToFill)
    {
        auto start = std::chrono::steady_clock::now();
        
        for(auction_msgs::task task : tasks)
        {
            double priceForTask = -1;
            


            if(task.task_name == "moveTo2D")
            {
                geometry_msgs::Point goal; // check number of args? 
                auto parts = BT::splitString(task.task_data, ';');
                goal.x = BT::convertFromString<double>(parts[0]);
                goal.y = BT::convertFromString<double>(parts[1]);
                goal.z = BT::convertFromString<double>(parts[2]);


                // copmute cost, based on current state and goal pos

/*
               double cost = pow((state.currentPose.position.x - goal.x), 2) + 
                             pow((state.currentPose.position.y - goal.y), 2);

                if(cost < 1)
                {
                    cost = 0.1; // TODO, handle (hack for now, dsp seems to not work very close to the point?)
                    break;
                }
*/

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
            else if(task.task_name == "moveStraightTo2D")
            {
                geometry_msgs::Point goal; // check number of args? 
                auto parts = BT::splitString(task.task_data, ';');
                goal.x = BT::convertFromString<double>(parts[0]);
                goal.y = BT::convertFromString<double>(parts[1]);
                goal.z = BT::convertFromString<double>(parts[2]);


                // copmute cost, based on current state and goal pos


               double cost = pow((state.currentPose.position.x - goal.x), 2) + 
                             pow((state.currentPose.position.y - goal.y), 2);

                if(cost < 1)
                {
                    cost = 0.1; // TODO, handle (hack for now, dsp seems to not work very close to the point?)
                }

                priceForTask = cost;
            }
            else
            {
                priceForTask = -1;
            }

            //TODO: maybe move this type of logic to the auction server? or is it good here?
            if(this->currentTask.task == task)
            {
                priceForTask = priceForTask * 0.7;//0.8;
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



    std::string Client_groundAgent::getXMLForTask(auction_msgs::task task)
    {
        if(task.task_name == "moveTo2D")
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
                        <Action ID="GroundAtPoint2D"/>
                        <Action ID="GroundFollowPath"/>
                    </ReactiveFallback>
                </BehaviorTree>
            </root>
            )";


            return xml_text;
        }
        else if(task.task_name == "moveStraightTo2D")
        {
            // setup task variables, should be removed and merged into the tree? TODO
            geometry_msgs::Point goal; // check number of args?
            auto parts = BT::splitString(task.task_data, ';');
            goal.x = BT::convertFromString<double>(parts[0]);
            goal.y = BT::convertFromString<double>(parts[1]);
            //goal.z = BT::convertFromString<double>(parts[2]);
            goal.z = 2;

            this->state.goalPoint = goal;

            std::cout << "Got new task, moveStraightTo2D. Parameters: " << goal.x << "," << goal.y  << std::endl;


            state.setGoalPathPlanner_pub.publish(goal);

            //generate XML here
            static const char* xml_text = R"(
            <root main_tree_to_execute = "MainTree" >
                <BehaviorTree ID="MainTree">
                    <ReactiveFallback name="root">
                        <Action ID="GroundAtPoint2D"/>
                        <Action ID="GroundMoveToGoalPoint"/>
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

    std::string Client_groundAgent::getXMLNoTask()
    {
        std::cout << "get xml for no task" << std::endl;
        // setup task variables, should be removed and merged into the tree? TODO
        //static bool goalIsSet = false;
        //if(goalIsSet == false)
        //{
            this->state.goalPoint = state.currentPose.position;
        //    goalIsSet = true;
        //}
        std::cout << "goal no task: " << state.goalPoint.x << ", " << state.goalPoint.y << std::endl;

        // TODO ???
        //state.goalPoint.z = 2;
        geometry_msgs::Point tmp_path = state.currentPose.position;
        tmp_path.x += 0.1;
        state.setGoalPathPlanner_pub.publish(tmp_path);//state.goalPoint);

        static const char* xml_text = R"(
            <root main_tree_to_execute = "MainTree" >
                <BehaviorTree ID="MainTree">
                    <ReactiveFallback name="root">
                        <Action ID="GroundAtPoint2D"/>
                        <Action ID="GroundMoveToGoalPoint"/>
                    </ReactiveFallback>
                </BehaviorTree>
            </root>
            )";
        return xml_text;
    }
    void Client_groundAgent::initNodes(BT::Tree& tree)
    {
        // Iterate through all the nodes and call init() if it is an Action_B
        for( auto& node: tree.nodes )
        {
            // Not a typo: it is "=", not "=="          
            if(auto groundMoveToGoalPoint = dynamic_cast<behaviors::GroundMoveToGoalPoint*>( node.get()))
            {
                groundMoveToGoalPoint->init(&state);
            }
            if(auto groundFollowPath = dynamic_cast<behaviors::GroundFollowPath*>( node.get()))
            {
                groundFollowPath->init(&state);
            }

            if(auto groundAtPoint2D = dynamic_cast<behaviors::GroundAtPoint2D*>( node.get()))
            {
                groundAtPoint2D->init(&state);
            }
        }
    }
    void Client_groundAgent::initFactory(BT::BehaviorTreeFactory& factory)
    {
        factory.registerNodeType<behaviors::GroundAtPoint2D>("GroundAtPoint2D");
        factory.registerNodeType<behaviors::GroundFollowPath>("GroundFollowPath");
        factory.registerNodeType<behaviors::GroundMoveToGoalPoint>("GroundMoveToGoalPoint");


        //std::cout << "init nodes" << std::endl;


        return;
    }




    void Client_groundAgent::odomCB(const nav_msgs::Odometry& msg)
    {
        state.currentPose = msg.pose.pose;
        //std::cout << "odom callback: x:" << msg.pose.pose.position.x << ", y:" << msg.pose.pose.position.y << std::endl;
    }
    void Client_groundAgent::pathCB(const nav_msgs::Path& msg)
    {
        state.path = msg;
        state.pathIndex = 0;
    }


} // end namespace





