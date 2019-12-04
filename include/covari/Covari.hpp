#include <iostream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

class Covari
{
    private:
        ros::NodeHandle nh_;
        geometry_msgs::PoseWithCovarianceStamped covari_msg;
        
        //Subscriber
        ros::Subscriber covari_sub;
        
    public:
        Covari(ros::NodeHandle _nh):
            nh_(_nh)
        {
            initSubscriber();
            txtOpen();
        }

        ~Covari()
        {
            txtClose();
        }
    private:
        void initSubscriber();

        void covari_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& covari_msg);

    //PoseWithCovarianceStamped msg
        void txtOpen();
        void txtClose();
        std::ofstream outFile;

        // string frame_id;
        // int seq;
        // double x,y,z;
        // double quatX,quatY,quatZ,quatW;
        // double[36] covariance;
};