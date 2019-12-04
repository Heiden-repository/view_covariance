#include "ros/ros.h"

#include "covari/Covari.hpp"

int main(int argc, char **argv)
{
    ros::init(argc,argv,"covari_node");

    ros::NodeHandle nh;

    ROS_INFO("%s",ros::this_node::getName().c_str());
    
    Covari cv(nh);
    ros::spin();

    return 0;
}