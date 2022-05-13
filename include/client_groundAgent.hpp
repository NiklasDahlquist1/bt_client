








#ifndef CLIENT_GROUNDAGENT_HPP
#define CLIENT_GROUNDAGENT_HPP

#include "ros/ros.h"
#include "bt_client.hpp"


#include "auction_msgs/auction.h"
#include "auction_msgs/bid.h"
#include "auction_msgs/price_bid.h"
#include "auction_msgs/task.h"
#include "auction_msgs/taskArray.h"
#include "auction_msgs/task_allocated.h"
#include "auction_msgs/task_finished.h"


//#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"


#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

//#include "dsp/pathCost.h"


namespace auction_ns
{


    struct ground_agent_state
    {
        geometry_msgs::Pose currentPose;
        ros::Publisher goalPoint_pub;
        ros::Publisher setGoalPathPlanner_pub;


        geometry_msgs::Point goalPoint; // TODO, probably remove?
    

        nav_msgs::Path path;
        int pathIndex = 0;
    };

    class Client_groundAgent : public Auction_client_bt
    {
        private:



        ground_agent_state state;
  

        


        //ros
        ros::Subscriber odom_sub;
        ros::Subscriber path_sub;

        //callbacks
        void odomCB(const nav_msgs::Odometry& msg);
        void pathCB(const nav_msgs::Path& msg);


        double costForTask(auction_msgs::task task);
        std::string getXMLForTask(auction_msgs::task task);
        std::string getXMLNoTask();
        void initNodes(BT::Tree& tree);
        void initFactory(BT::BehaviorTreeFactory& factory);



        protected:
        public:
        using Auction_client_bt::Auction_client_bt;

        Client_groundAgent(); // setup callbacks, state (used for bt), etc. here



    };



























    Client_groundAgent::Client_groundAgent()
    {
        // setup subscribers
        odom_sub = nodeHandle.subscribe("odom", 1000, &Client_groundAgent::odomCB, this);
        path_sub = nodeHandle.subscribe("path", 1000, &Client_groundAgent::pathCB, this);
        //
        state.goalPoint_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("goalPose", 1000);
        state.setGoalPathPlanner_pub = nodeHandle.advertise<geometry_msgs::Point>("setPathGoal", 1000);


        std::cout << "client started" << std::endl;
        initFactory(factory);

        //pathCostClient = nodeHandle.serviceClient<dsp::pathCost>("dsp/path_cost");
    }


    double Client_groundAgent::costForTask(auction_msgs::task task)
    {
        if(task.task_name == "huskyMoveTo")
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
    std::string Client_groundAgent::getXMLNoTask()
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
    void Client_groundAgent::initNodes(BT::Tree& tree)
    {
        /*
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
        */
    }
    void Client_groundAgent::initFactory(BT::BehaviorTreeFactory& factory)
    {
        /*factory.registerNodeType<behaviors::MoveToGoalPoint>("moveToGoalPoint");
        factory.registerNodeType<behaviors::UAVAtPoint>("UAVAtPoint");
        factory.registerNodeType<behaviors::FollowPath>("FollowPath");
        factory.registerNodeType<behaviors::UAVAtPoint2D>("UAVAtPoint2D");
*/





        return;
    }




    void Client_groundAgent::odomCB(const nav_msgs::Odometry& msg)
    {
        state.currentPose = msg.pose.pose;
        //std::cout << "odom callback" << std::endl;
    }
    void Client_groundAgent::pathCB(const nav_msgs::Path& msg)
    {
        state.path = msg;
        state.pathIndex = 0;
    }














} // end namespace























#endif
