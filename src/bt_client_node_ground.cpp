

#include "client_groundAgent.hpp"

//#include "ros/ros.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bt_client");

    ros::NodeHandle nh;

    auction_ns::Client_groundAgent client = auction_ns::Client_groundAgent();
    
    ros::Rate r = ros::Rate(1);
    r.sleep();
    ros::spinOnce(); // delay to allow callbacks to update state, etc.

    client.spin_loop(100.0);
    

    return 0;
}





