#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Path.h>
#include <math.h>

class Leica
{
public:
    Leica()
    {
        leica_sub = nh.subscribe("/leica/position",100,&Leica::leicaCallback,this);
        leica_pub = nh.advertise<geometry_msgs::PointStamped>("/actual_point",100);
        leicapath_pub = nh.advertise<nav_msgs::Path>("/actual_path",1);
    }
    ~Leica(){}
    void leicaCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
    {
        if(!initialvalue)
        {
            initial_x = msg->point.x;
            initial_y = msg->point.y;
            initial_z = msg->point.z;
            initialvalue=true;
        }
        geometry_msgs::PointStamped pos;
        pos.header = msg->header;
        pos.header.frame_id = "map";
        double theta = 15*3.14/180;
        pos.point.x = cos(theta)*(msg->point.x - initial_x) - sin(theta)*(msg->point.y - initial_y);
        pos.point.y = sin(theta)*(msg->point.x - initial_x) + cos(theta)*(msg->point.y - initial_y);
        pos.point.z = msg->point.z - initial_z;
        leica_pub.publish(pos);

        geometry_msgs::PoseStamped posestamp;
        posestamp.pose.position.x = pos.point.x;
        posestamp.pose.position.y = pos.point.y;
        posestamp.pose.position.z = pos.point.z;
        posestamp.pose.orientation.x = 0;
        posestamp.pose.orientation.y = 0;
        posestamp.pose.orientation.z = 0;
        posestamp.pose.orientation.w = 1;
        path.poses.push_back(posestamp);
        path.header.frame_id = "map";
        leicapath_pub.publish(path);

        std::cout<<"Receive leica position!"<<std::endl;
    }
private:
    ros::NodeHandle nh;
    ros::Subscriber leica_sub;
    ros::Publisher leica_pub;
    ros::Publisher leicapath_pub;
    nav_msgs::Path path;
    double initial_x = 0.0;
    double initial_y = 0.0;
    double initial_z = 0.0;
    bool initialvalue = false;
};

class Vicon
{
public:
    Vicon()
    {
        vicon_sub = nh.subscribe("/vicon/firefly_sbx/firefly_sbx",100,&Vicon::viconCallback,this);
        vicon_pub = nh.advertise<geometry_msgs::PointStamped>("/vicon/actual_point",100);
        viconpath_pub = nh.advertise<nav_msgs::Path>("/vicon/actual_path",1);
    }
    ~Vicon(){}
    void viconCallback(const geometry_msgs::TransformStamped::ConstPtr &msg)
    {
        if(!initialvalue)
        {
            initial_x = msg->transform.translation.x;
            initial_y = msg->transform.translation.y;
            initial_z = msg->transform.translation.z;
            initialvalue=true;
        }
        geometry_msgs::PointStamped pos;
        pos.header = msg->header;
        pos.header.frame_id = "map";
        double theta = 160*3.14/180;
        pos.point.x = cos(theta)*(msg->transform.translation.x - initial_x) - sin(theta)*(msg->transform.translation.y - initial_y);
        pos.point.y = sin(theta)*(msg->transform.translation.x - initial_x) + cos(theta)*(msg->transform.translation.y - initial_y);
        pos.point.z = msg->transform.translation.z - initial_z;
        //pos.point.x = msg->transform.translation.x - initial_x;
        //pos.point.y = msg->transform.translation.y - initial_y;
        //pos.point.z = msg->transform.translation.z - initial_z;
        vicon_pub.publish(pos);

        geometry_msgs::PoseStamped posestamp;
        posestamp.pose.position.x = pos.point.x;
        posestamp.pose.position.y = pos.point.y;
        posestamp.pose.position.z = pos.point.z;
        posestamp.pose.orientation.x = msg->transform.rotation.x;
        posestamp.pose.orientation.y = msg->transform.rotation.y;
        posestamp.pose.orientation.z = msg->transform.rotation.z;
        posestamp.pose.orientation.w = msg->transform.rotation.w;
        path.poses.push_back(posestamp);
        path.header.frame_id = "map";
        viconpath_pub.publish(path);
    }
private:
    ros::NodeHandle nh;
    ros::Subscriber vicon_sub;
    ros::Publisher vicon_pub;
    ros::Publisher viconpath_pub;
    nav_msgs::Path path;
    double initial_x = 0.0;
    double initial_y = 0.0;
    double initial_z = 0.0;
    bool initialvalue = false;
};
int main(int argc,char** argv)
{
    ros::init(argc,argv,"leica_position");
    Leica leica;
    Vicon vicon;
    ros::spin();
    return 0;
}
