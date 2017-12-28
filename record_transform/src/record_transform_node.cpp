#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Path.h>
#include <ostream>
#include <sstream>
#include <fstream>
std::string path="/home/ubuntu/";
void estimateCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  std::string newpath = path+"labtf.txt";
  std::cout<<newpath<<std::endl;
  std::ofstream outfile;
  outfile.open(newpath.c_str(),std::ios::app);  
  outfile<<msg->header.stamp.toNSec()<<" "
         <<msg->point.x<<" "
         <<msg->point.y<<" " 
         <<msg->point.z<<" "
         <<0<<" "
         <<0<<" "
         <<0<<" "
         <<0<<std::endl;
  outfile.close();	
}
void leicaposeCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  std::string newpath = path+"leicatf.txt";
  std::cout<<newpath<<std::endl;
  std::ofstream outfile;
  outfile.open(newpath.c_str(),std::ios::app);  
  outfile<<msg->header.stamp.toNSec()<<" "
         <<msg->point.x<<" "
         <<msg->point.y<<" " 
         <<msg->point.z<<" " 
         <<0<<" "
         <<0<<" "
         <<0<<" "
         <<0<<std::endl;
  outfile.close();
}
void viconposeCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    std::string newpath = path+"vicontf.txt";
    std::cout<<newpath<<std::endl;
    std::ofstream outfile;
    outfile.open(newpath.c_str(),std::ios::app);
    outfile<<msg->header.stamp.toNSec()<<" "
           <<msg->point.x<<" "
           <<msg->point.y<<" "
           <<msg->point.z<<" "
           <<0<<" "
           <<0<<" "
           <<0<<" "
           <<0<<std::endl;
    outfile.close();
}
int main(int argc,char** argv)
{
    ros::init(argc,argv,"record_transform");
    ros::NodeHandle nh;
    ros::Subscriber estimatesub = nh.subscribe("/estimate_pose",100,&estimateCallback);
    ros::Subscriber leicasub = nh.subscribe("/actual_point",100,&leicaposeCallback);
    ros::Subscriber viconsub = nh.subscribe("/vicon/actual_point",100,&viconposeCallback);
    ros::spin();
    return 0;
}
