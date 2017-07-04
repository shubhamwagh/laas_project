#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>
#include <laas_project/PoseOri.h>
#include <laas_project/or_pose_estimator_state.h>
using Eigen::MatrixXd;
class RealSense
{
        public:

                RealSense();
                void chatterCallback1(const nav_msgs::Odometry::ConstPtr& msg);
                void chatterCallback2(const laas_project::or_pose_estimator_state::ConstPtr& msg);

        private:
                ros::NodeHandle nh_;
                ros::Publisher pos_ori_pub1_;
                ros::Publisher pos_ori_pub2_;

                ros::Subscriber sub1;
                ros::Subscriber sub2;
                

                double ex1, ey1, ez1, eta1, ex2, ey2, ez2, eta2;
                double x1, y1, z1, x2, y2, z2;
                double roll1, pitch1, yaw1, roll2, yaw2, pitch2;
                laas_project::PoseOri pos_ori1;
                laas_project::PoseOri pos_ori2;


};


RealSense::RealSense()
{
        
        // Initialize all position variables and orientation variables
        //euclid variables
        x1 = y1 = z1 = ex1 = ey1 = ez1 = eta1 = 0;
        roll1 = pitch1 = yaw1 = 0;
        
        //mocap variables
        x2 = y2 = z2 = ex2 = ey2 = ez2 = eta2 = 0;
        roll2 = pitch2 = yaw2 = 0;
        
        pos_ori_pub1_ = nh_.advertise<laas_project::PoseOri>("/position_orientations/euclid",1000);
        pos_ori_pub2_ = nh_.advertise<laas_project::PoseOri>("/position_orientations/mocap", 1000);
        sub1 = nh_.subscribe("/realsense/odom", 1000, &RealSense::chatterCallback1, this);
        sub2 = nh_.subscribe("/optitrack/bodies/brunelleschi", 1000, &RealSense::chatterCallback2, this);
        


}

void RealSense::chatterCallback1(const nav_msgs::Odometry::ConstPtr &msg)
{

    ex1 = msg -> pose.pose.orientation.x;
    ey1 = msg -> pose.pose.orientation.y;
    ez1 = msg -> pose.pose.orientation.z;
    eta1 = msg -> pose.pose.orientation.w;

    x1 = msg -> pose.pose.position.x;
    y1 = msg -> pose.pose.position.y;
    z1 = msg -> pose.pose.position.z;

    MatrixXd r(3,3);
    //First row
    r(0,0) = 2*(pow(eta1,2) + pow(ex1,2)) - 1;
    r(0,1) = 2*(ex1*ey1 - eta1*ez1);
    r(0,2) = 2*(ex1*ez1 + eta1*ey1);

    //Second row
    r(1,0) = 2*(ex1*ey1 + eta1*ez1);
    r(1,1) = 2*(pow(eta1,2) + pow(ey1,2)) - 1;
    r(1,2) = 2*(ey1*ez1 - eta1*ex1);

    //Third row
    r(2,0) = 2*(ex1*ez1 - eta1*ey1);
    r(2,1) = 2*(ey1*ez1 + eta1*ex1);
    r(2,2) = 2*(pow(eta1,2) + pow(ez1,2)) - 1;
    
    roll1 = atan2(r(2,1),r(2,2));
    pitch1 = atan2(-r(2,0),sqrt(pow(r(2,1),2) + pow(r(2,2),2)));
    yaw1 = atan2(r(1,0),r(0,0));

    pos_ori1.x = x1;
    pos_ori1.y = y1;
    pos_ori1.z = z1;


    pos_ori1.roll = roll1;
    pos_ori1.yaw = yaw1;
    pos_ori1.pitch = pitch1;

    std::cout << "Position - Euclid\n";
    std::cout << "  x : " << pos_ori1.x << "\n";
    std::cout << "  y : " << pos_ori1.y << "\n";
    std::cout << "  z : " << pos_ori1.z << "\n";
    //std::endl;
	std::cout << "-----------------------------------------\n";
    std::cout << "Orientation - Euclid\n";
    std::cout << "  roll : " << pos_ori1.roll << "\n";
    std::cout << "  pitch : " << pos_ori1.pitch << "\n";
    std::cout << "  yaw : " << pos_ori1.yaw << "\n";
    
    std::cout << "=========================================\n";
    //std::endl;
    pos_ori_pub1_.publish(pos_ori1);


}

void RealSense::chatterCallback2(const laas_project::or_pose_estimator_state::ConstPtr& msg)
{

    ex2 = msg -> pos[0].qx;
    ey2 = msg -> pos[0].qy;
    ez2 = msg -> pos[0].qz;
    eta2 = msg -> pos[0].qw;

    x2 = msg -> pos[0].x;
    y2 = msg -> pos[0].y;
    z2 = msg -> pos[0].z;

    MatrixXd r(3,3);
    //First row
    r(0,0) = 2*(pow(eta2,2) + pow(ex2,2)) - 1;
    r(0,1) = 2*(ex2*ey2 - eta2*ez2);
    r(0,2) = 2*(ex2*ez2 + eta2*ey2);

    //Second row
    r(1,0) = 2*(ex2*ey2 + eta2*ez2);
    r(1,1) = 2*(pow(eta2,2) + pow(ey2,2)) - 1;
    r(1,2) = 2*(ey2*ez2 - eta2*ex2);

    //Third row
    r(2,0) = 2*(ex2*ez2 - eta2*ey2);
    r(2,1) = 2*(ey2*ez2 + eta2*ex2);
    r(2,2) = 2*(pow(eta2,2) + pow(ez2,2)) - 1;

    roll2 = atan2(r(2,1),r(2,2));
    pitch2 = atan2(-r(2,0),sqrt(pow(r(2,1),2) + pow(r(2,2),2)));
    yaw2 = atan2(r(1,0),r(0,0));

    pos_ori2.x = x2;
    pos_ori2.y = y2;
    pos_ori2.z = z2;


    pos_ori2.roll = roll2;
    pos_ori2.yaw = yaw2;
    pos_ori2.pitch = pitch2;

    std::cout << "Position from Mocap\n";
    std::cout << "  x : " << pos_ori2.x << "\n";
    std::cout << "  y : " << pos_ori2.y << "\n";
    std::cout << "  z : " << pos_ori2.z << "\n";
    //std::endl;
	std::cout << "-----------------------------------------\n";
    std::cout << "Orientation from Mocap\n";
    std::cout << "  roll : " << pos_ori2.roll << "\n";
    std::cout << "  pitch : " << pos_ori2.pitch << "\n";
    std::cout << "  yaw : " << pos_ori2.yaw << "\n";
    //std::endl;
    std::cout << "=========================================\n";
    pos_ori_pub2_.publish(pos_ori2);


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
