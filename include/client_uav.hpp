



#ifndef CLIENT_UAV_HPP
#define CLIENT_UAV_HPP


#include "bt_client.hpp"



namespace auction_ns
{




    struct uav_state
    {
        geometry_msgs::Pose currentPose;
        ros::Publisher goalPoint_pub;
        ros::Publisher setGoalPathPlanner_pub;


        geometry_msgs::Point goalPoint; // TODO, probably remove?
        bool goalIsSet = false;
        

        nav_msgs::Path path;
        int pathIndex = 0;
    };

    class Client_uav : public Auction_client_bt
    {
        private:



        uav_state state;
 
        ros::ServiceClient pathCostServiceClient;
        


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

        Client_uav(); // setup callbacks, state (used for bt), etc. here

    };




} // end namespace


#endif
