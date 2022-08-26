








#ifndef BT_CLIENT_HPP
#define BT_CLIENT_HPP

#include "ros/ros.h"
#include "auction_client.hpp"


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

#include "dsp/pathCost.h"


namespace auction_ns
{




    class Auction_client_bt : public Auction_client
    {
        private:





        //BT
        //factory, etc

        // splitString()
        

        // callbacks
        //void poseCB(const geometry_msgs::Pose& msg);

        //ros::Subscriber currentPose_sub;


        void handleNewTask(auction_msgs::task task); // swtitch to bt for task, initialize new tree nodes, etc.
        void handleNoTask(); // switch to no task bt
        void executeCurrentBehavior(); // tick bt, also check if task is completed
        taskStatus currentTaskStatus(); // check if task is completed, (maybe return bt status? in this case)

        protected:
        BT::BehaviorTreeFactory factory;
        BT::Tree taskTree;
        std::string taskXML;



        taskStatus taskCurrentStatus;

        virtual void costForTasks(const std::vector<auction_msgs::task>& tasks, std::vector<auction_msgs::price_bid>& pricesToFill); // return cost for task, (distance, cost from dsp, time, etc.)
        virtual std::string getXMLForTask(auction_msgs::task task);
        virtual std::string getXMLNoTask();
        virtual void initNodes(BT::Tree& tree);
        virtual void initFactory(BT::BehaviorTreeFactory& factory);

        // generate BT, initialize nodes etc.
        // generate no task BT, init nodes...
        // 

        public:
        using Auction_client::Auction_client;

        Auction_client_bt(); // setup callbacks, state (used for bt), etc. here
    };





















/*
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


        void costForTasks(const std::vector<auction_msgs::task>& tasks, std::vector<auction_msgs::price_bid>& pricesToFill);
        std::string getXMLForTask(auction_msgs::task task);
        std::string getXMLNoTask();
        void initNodes(BT::Tree& tree);
        void initFactory(BT::BehaviorTreeFactory& factory);



        protected:
        public:
        using Auction_client_bt::Auction_client_bt;

        Client_groundAgent(); // setup callbacks, state (used for bt), etc. here

    };
*/




    //TODO, implement client for for other types of agents












} // end namespace


























#endif
