

#include "bt_client.hpp"


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
        taskXML = getXMLForTask(task);
        taskTree = factory.createTreeFromText(taskXML);
        initNodes(taskTree);
    }
    void Auction_client_bt::handleNoTask() // switch to no task bt
    {
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
    double Auction_client_bt::costForTask(auction_msgs::task task)
    {
        std::cout << "cost for task function not implemented" << std::endl;
        return -1;
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

        //pathCostClient = nodeHandle.serviceClient<dsp::pathCost>("dsp/path_cost");
    }


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

            double cost = pow((state.currentPose.position.x - goal.x), 2) + 
                          pow((state.currentPose.position.y - goal.y), 2);

            return cost;

            /*dsp::pathCost srv;
            srv.request.start = state.currentPose.position;
            srv.request.stop = state.goalPoint;
            if (pathCostClient.call(srv))
            {
                std::cout << "cost " << srv.response.cost << std::endl;
                return srv.response.cost;
            }
            else
            {
                return -1;
            }*/

        }
        else
        {
            return -1;
        }
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
            goal.z = 0;

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
        else
        {
            return "";
        }
    }
    std::string Client_uav::getXMLNoTask()
    {
        std::cout << "get xml for no task" << std::endl;
        // setup task variables, should be removed and merged into the tree? TODO
        this->state.goalPoint = state.currentPose.position;
        std::cout << "goal no task: " << state.goalPoint.x << ", " << state.goalPoint.y << ", " << state.goalPoint.z << std::endl;


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












