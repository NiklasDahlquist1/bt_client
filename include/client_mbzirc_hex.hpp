



#ifndef CLIENT_MBZIRC_HEX_HPP
#define CLIENT_MBZIRC_HEX_HPP


#include "bt_client.hpp"



namespace auction_ns
{




    struct mbzirc_hex_state
    {
        geometry_msgs::Pose currentPose;
        ros::Publisher goalPoint_pub;
        ros::Publisher setGoalPathPlanner_pub;

            
            
        // publish start send data
        // subscirbe data sent finished


        geometry_msgs::Point goalPoint; // TODO, probably remove?
        bool goalIsSet = false;


        // related to test communication
        int packet_size;
        int packet_num;
        double msgs_per_sec;
        std::string target_name;
        

    };

    class Client_mbzirc_hex : public Auction_client_bt
    {
        private:



        mbzirc_hex_state state;
 
        


        //ros
        ros::Subscriber odom_sub;


        //callbacks
        void odomCB(const nav_msgs::Odometry& msg);


        void costForTasks(const std::vector<auction_msgs::task>& tasks, std::vector<auction_msgs::price_bid>& pricesToFill);
        std::string getXMLForTask(auction_msgs::task task);
        std::string getXMLNoTask();
        void initNodes(BT::Tree& tree);
        void initFactory(BT::BehaviorTreeFactory& factory);



        protected:
        public:
        using Auction_client_bt::Auction_client_bt;

        Client_mbzirc_hex(); // setup callbacks, state (used for bt), etc. here

    };




} // end namespace


#endif
