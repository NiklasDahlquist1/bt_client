








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


#include "ros_turtlebot_control/MoveToPoint.h"
//#include "ros_turtlebot_control/MoveToPointRequest.h"
//#include "ros_turtlebot_control/MoveToPointResponse.h"


namespace auction_ns
{


    struct ground_agent_state
    {
        geometry_msgs::Pose currentPose;
        
        geometry_msgs::Point goalPoint;
        //ros::ServiceClient setGoal_srv;
        ros::Publisher goalPoint_pub;
        ros::Publisher setGoalPathPlanner_pub;


    

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


        ros::ServiceClient pathCostServiceClient;


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




























} // end namespace























#endif
