#include <cstdio>
#include <iostream>
#include <fstream>
#include <string>

#include "ros/ros.h"
#include "covari/Covari.hpp"

void Covari::initSubscriber()
{
    covari_sub=nh_.subscribe("/amcl_pose",10, &Covari::covari_callback, this);
}

void Covari::txtOpen()
{
  outFile.open("covariance.txt");
  outFile << "============================================="<< std::endl;
}

void Covari::txtClose()
{
    outFile.close();
}

void Covari::covari_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& covari_msg)
{
    // ROS_INFO("=============================================");
    // ROS_INFO("%d",covari_msg->header.seq);
    // ROS_INFO("%s",covari_msg->header.frame_id.c_str());

    // ROS_INFO("%lf",covari_msg->pose.pose.position.x);
    // ROS_INFO("%lf",covari_msg->pose.pose.position.y);
    // ROS_INFO("%lf",covari_msg->pose.pose.position.z);
    
    // ROS_INFO("%lf",covari_msg->pose.pose.orientation.x);
    // ROS_INFO("%lf",covari_msg->pose.pose.orientation.y);
    // ROS_INFO("%lf",covari_msg->pose.pose.orientation.z);
    // ROS_INFO("%lf",covari_msg->pose.pose.orientation.w);

    // ROS_INFO("%lf",covari_msg->pose.covariance);

    outFile << "seq : " << covari_msg->header.seq << std::endl;
    outFile << "frame_id : " << covari_msg->header.frame_id << std::endl;

    outFile << "position x : " << covari_msg->pose.pose.position.x << std::endl;
    outFile << "position y : " << covari_msg->pose.pose.position.y << std::endl;
    outFile << "position z : " << covari_msg->pose.pose.position.z << std::endl;

    outFile << "orientation x : " << covari_msg->pose.pose.orientation.x << std::endl;
    outFile << "orientation y : " << covari_msg->pose.pose.orientation.y << std::endl;
    outFile << "orientation z : " << covari_msg->pose.pose.orientation.z << std::endl;
    outFile << "orientation w : " << covari_msg->pose.pose.orientation.w << std::endl;

    outFile << "covariance : ";
    for(int i = 0; i<36; i++)
        outFile << "[" <<covari_msg->pose.covariance[i] << "] ";
    outFile << std::endl <<"============================================="<< std::endl;
}

