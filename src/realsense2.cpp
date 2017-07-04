#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>
#include <laas_project/PoseOri.h>
using Eigen::MatrixXd;
class RealSense
{
        public:

                RealSense();
                void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg);

        private:
                ros::NodeHandle nh_;
                ros::Publisher pos_ori_pub_;

                ros::Subscriber sub;

                double ex, ey, ez, eta;
                double x, y, z;
                double roll, pitch, yaw;
                laas_project::PoseOri pos_ori;


};


RealSense::RealSense()
{
        
        // Initialize all position variables and orientation variables
        
        x = y = z = ex = ey = ez = eta = 0;
        roll = pitch = yaw = 0;
        pos_ori_pub_ = nh_.advertise<laas_project::PoseOri>("/position_orientations",1000);
        sub = nh_.subscribe("/realsense/odom", 1000, &RealSense::chatterCallback, this);
        


}

void RealSense::chatterCallback(const nav_msgs::Odometry::ConstPtr &msg)
{

    ex = msg -> pose.pose.orientation.x;
    ey = msg -> pose.pose.orientation.y;
    ez = msg -> pose.pose.orientation.z;
    eta = msg -> pose.pose.orientation.w;

    x = msg -> pose.pose.position.x;
    y = msg -> pose.pose.position.y;
    z = msg -> pose.pose.position.z;

    MatrixXd r(3,3);
    //First row
    r(0,0) = 2*(pow(eta,2) + pow(ex,2)) - 1;
    r(0,1) = 2*(ex*ey - eta*ez);
    r(0,2) = 2*(ex*ez + eta*ey);

    //Second row
    r(1,0) = 2*(ex*ey + eta*ez);
    r(1,1) = 2*(pow(eta,2) + pow(ey,2)) - 1;
    r(1,2) = 2*(ey*ez - eta*ex);

    //Third row
    r(2,0) = 2*(ex*ez - eta*ey);
    r(2,1) = 2*(ey*ez + eta*ex);
    r(2,2) = 2*(pow(eta,2) + pow(ez,2)) - 1;

    roll = atan2(r(2,1),r(2,2));
    pitch = atan2(-r(2,0),sqrt(pow(r(2,1),2) + pow(r(2,2),2)));
    yaw = atan2(r(1,0),r(0,0));

    pos_ori.x = x;
    pos_ori.y = y;
    pos_ori.z = z;


    pos_ori.roll = roll;
    pos_ori.yaw = yaw;
    pos_ori.pitch = pitch;

    std::cout << "Position\n";
    std::cout << "  x : " << pos_ori.x << "\n";
    std::cout << "  y : " << pos_ori.y << "\n";
    std::cout << "  z : " << pos_ori.z << "\n";
    //std::endl;

    std::cout << "Orientation\n";
    std::cout << "  roll : " << pos_ori.roll << "\n";
    std::cout << "  pitch : " << pos_ori.pitch << "\n";
    std::cout << "  yaw : " << pos_ori.yaw << "\n";
    //std::endl;
    pos_ori_pub_.publish(pos_ori);


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "realsense");

        RealSense rs;

        ros::Rate loop_rate(10);
        while (ros::ok())
        {

            ros::spinOnce();
            loop_rate.sleep();

        }

        return 0;


}
