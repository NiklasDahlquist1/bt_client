

#include "client_turtlebot.hpp"

#include <chrono>
#include <unistd.h>
#include <iostream>

#include "behaviors_turtlebot.h"








namespace auction_ns
{




    Client_turtlebot::Client_turtlebot()
    {
        // setup subscribers
        odom_sub = nodeHandle.subscribe("odom", 1000, &Client_turtlebot::odomCB, this);
        path_sub = nodeHandle.subscribe("path", 1000, &Client_turtlebot::pathCB, this);
        //
        state.goalPoint_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("goalPose", 1000);
        state.setGoalPathPlanner_pub = nodeHandle.advertise<geometry_msgs::Point>("setPathGoal", 1000);


        std::cout << "ground client started" << std::endl;
        initFactory(factory);

        pathCostServiceClient = nodeHandle.serviceClient<dsp::pathCost>("dsp/path_cost");
        //this->state.setGoal_srv = nodeHandle.serviceClient<ros_turtlebot_control::MoveToPoint>("turtle/move_to_point");

    }



    void Client_turtlebot::costForTasks(const std::vector<auction_msgs::task>& tasks, std::vector<auction_msgs::price_bid>& pricesToFill)
    {
        auto start = std::chrono::steady_clock::now();


        // TODO, test this
        // test check flag from BT, if current task can be abandoned
        if(state.task_can_be_swapped == false)
        {                
            // search for task, and only submit bid for that
            for(const auction_msgs::task& task : tasks)
            {
                double priceForTask = -1; 

                // to make sure that this agent will win his current task
                if(task == this->currentTask.task)
                {
                    priceForTask = 0.01;
                    //std::cout << "TASK CANNOT BE SWAPPED" << "\n";

                    auction_msgs::price_bid p;
                    p.task_ID = task.task_ID;
                    p.price = priceForTask;

                    pricesToFill.push_back(p);
                    return;
                }
                else
                {
                    // we can still participate with the other tasks
                    //priceForTask = -1;
                }
            }            
        }



        
        for(const auction_msgs::task& task : tasks)
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
                    cost = 0;//-1;
                    //return 1;
                }
 
        
                    
                //std::cout << "Client: " << name <<  " Cost: " << srv.response.cost << std::endl;

                priceForTask = cost;
            }
            else if(task.task_name == "pickPlace")
            {
                geometry_msgs::Point goal_pick; 
                geometry_msgs::Point goal_place; 
                auto parts = BT::splitString(task.task_data, ';');
                goal_pick.x = BT::convertFromString<double>(parts[0]);
                goal_pick.y = BT::convertFromString<double>(parts[1]);
                goal_pick.z = 0;

                goal_place.x = BT::convertFromString<double>(parts[2]);
                goal_place.y = BT::convertFromString<double>(parts[3]);
                goal_place.z = 0;


                double cost = 0;
                dsp::pathCost srv;
                bool service_failed = false;


                // calculate cost to pick

                srv.request.start = state.currentPose.position;
                srv.request.start.z = 0;
                //std::cout << "start: " << state.currentPose.position << std::endl;
                srv.request.stop = goal_pick;
                srv.request.stop.z = 0;
                

                if (pathCostServiceClient.call(srv))
                {
                    cost += srv.response.cost;
                    if(cost > 1e9)
                    {
                        service_failed = true;
                    }
                }
                else
                {
                    std::cout << "Failed to call service path_cost1" << std::endl;
                    service_failed = true;
                }
 

                 // calculate cost to place
                srv.request.start = goal_pick;
                srv.request.stop = goal_place;


                if (pathCostServiceClient.call(srv))
                {
                    cost += srv.response.cost;
                    if(cost > 1e9)
                    {
                        service_failed = true;
                    }
                }
                else
                {
                    std::cout << "Failed to call service path_cost2" << std::endl;
                    service_failed = true;
                }


                //
                if(service_failed == true)
                {
                    cost = -1;
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
                priceForTask = priceForTask * 0.85;//0.8;
            }


            // check flag from BT, if current task can be abandoned
            if(state.task_can_be_swapped == false)
            {
                // to make sure that this agent will win his current task
                if(task == this->currentTask.task)
                {
                    priceForTask = 0.01;
                    //std::cout << "TASK CANNOT BE SWAPPED" << "\n";

                }
                else
                {
                    // we can still participate with the other tasks
                    //priceForTask = -1;
                }
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



    std::string Client_turtlebot::getXMLForTask(auction_msgs::task task)
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
        else if(task.task_name == "pickPlace")
        {
            
            geometry_msgs::Point pick_goal;
            geometry_msgs::Point place_goal;

            auto parts = BT::splitString(task.task_data, ';');

            pick_goal.x = BT::convertFromString<double>(parts[0]);
            pick_goal.y = BT::convertFromString<double>(parts[1]);
            pick_goal.z = 0;

            place_goal.x = BT::convertFromString<double>(parts[2]);
            place_goal.y = BT::convertFromString<double>(parts[3]);
            place_goal.z = 0;


            this->state.pick_goal = pick_goal;
            this->state.place_goal = place_goal;
            this->state.pick_complete = false; // maybe fixes the issue?


            std::cout << "Got new task, pickPlace. pick: " << pick_goal.x << ", " << pick_goal.y << " place: " << place_goal.x << ", " << place_goal.x << std::endl;


            state.setGoalPathPlanner_pub.publish(pick_goal);

            //generate XML here
            static const char* xml_text = R"(
            <root main_tree_to_execute = "MainTree" >
                <BehaviorTree ID="MainTree">
                    <ReactiveSequence name="root">

                        <ReactiveFallback>
                            <Action ID="Ground_at_pick_place"/>
                            <Action ID="GroundFollowPath"/>
                        </ReactiveFallback>

                        <ReactiveFallback>
                            <Action ID="Ground_at_place_place"/>
                            <Action ID="GroundFollowPath"/>
                        </ReactiveFallback>

                    </ReactiveSequence>
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

    std::string Client_turtlebot::getXMLNoTask()
    {
        state.task_can_be_swapped = true;

        // setup task variables, should be removed and merged into the tree? TODO
        //static bool goalIsSet = false;



        // save starting point first time, quite hacky, just for experiment TODO: FIX
        if(state.starting_point.z < 1)
        {
            state.starting_point = state.currentPose.position;
            state.starting_point.z = 1000;
            std::cout << "Saved starting pos " << state.starting_point.x << ", " << state.starting_point.y << "\n";
        }

        this->state.goalPoint = state.starting_point;//state.currentPose.position;

        std::cout << "goal no task: " << state.goalPoint.x << ", " << state.goalPoint.y << std::endl;





        // TODO ???
        //state.goalPoint.z = 2;
        //geometry_msgs::Point tmp_path = state.currentPose.position;
        geometry_msgs::Point tmp_path = state.starting_point;//state.currentPose.position;
        //tmp_path.x += 0.1;
        state.setGoalPathPlanner_pub.publish(tmp_path);//state.goalPoint);

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
    void Client_turtlebot::initNodes(BT::Tree& tree)
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

            if(auto ground_at_pick_place = dynamic_cast<behaviors::Ground_at_pick_place*>( node.get()))
            {
                ground_at_pick_place->init(&state);
            }

            if(auto ground_at_place_place = dynamic_cast<behaviors::Ground_at_place_place*>( node.get()))
            {
                ground_at_place_place->init(&state);
            }
        }
    }
    void Client_turtlebot::initFactory(BT::BehaviorTreeFactory& factory)
    {
        factory.registerNodeType<behaviors::GroundAtPoint2D>("GroundAtPoint2D");
        factory.registerNodeType<behaviors::GroundFollowPath>("GroundFollowPath");
        factory.registerNodeType<behaviors::GroundMoveToGoalPoint>("GroundMoveToGoalPoint");
        factory.registerNodeType<behaviors::Ground_at_pick_place>("Ground_at_pick_place");
        factory.registerNodeType<behaviors::Ground_at_place_place>("Ground_at_place_place");


        //std::cout << "init nodes" << std::endl;


        return;
    }




    void Client_turtlebot::odomCB(const nav_msgs::Odometry& msg)
    {
        state.currentPose = msg.pose.pose;
        //std::cout << "odom callback: x:" << msg.pose.pose.position.x << ", y:" << msg.pose.pose.position.y << std::endl;
    }
    void Client_turtlebot::pathCB(const nav_msgs::Path& msg)
    {
        state.path = msg;
        state.pathIndex = 0;
    }


} // end namespace





