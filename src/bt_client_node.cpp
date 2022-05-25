#include "bt_client.hpp"
//#include "ros/ros.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bt_client");

    ros::NodeHandle nh;

    auction_ns::Client_uav client = auction_ns::Client_uav();
    
    ros::Rate r = ros::Rate(1);
    r.sleep();
    ros::spinOnce(); // delay to allow callbacks to update state, etc.

    client.spin_loop(150.0);
    

    return 0;
}





