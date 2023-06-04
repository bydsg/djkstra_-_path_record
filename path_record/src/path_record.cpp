#include "path_record.hpp"
#include <vector>
#include <ros/ros.h>
#include <tf/tf.h>
#include <fstream>
#include <ros/package.h>
#include <ros/node_handle.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sys/time.h>
#include <time.h>
#include <cmath>
#include <iostream>
#include<sstream>
#include<string>
#include <unistd.h>
using namespace std;

string path_file="/home/by/git_bydsg/by_djstl_ws/src/by_djstl/data/road_path2.txt";

void path_record::record_path(const nav_msgs::Odometry::ConstPtr odometry_msg){
  char p;
  p='c';
   if (this->count == 0)
   {
    double raw, pitch, theta;
     tf::Quaternion q;
     tf::quaternionMsgToTF(odometry_msg->pose.pose.orientation, q);
     tf::Matrix3x3(q).getRPY(raw, pitch, theta);
      FILE *fp_s;
      fp_s = fopen(path_file.c_str(),"a");
      fprintf(fp_s, "%c %lf %lf %lf", p,odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y,theta);                   // Angle
      fprintf(fp_s, "\r\n");  
      fclose(fp_s);
      start_x =  odometry_msg->pose.pose.position.x;
      start_y =  odometry_msg->pose.pose.position.y;
      this->count++;
   }else{
    sample_distance = sample_distance+sqrt(pow((odometry_msg->pose.pose.position.x - start_x), 2) + pow((odometry_msg->pose.pose.position.y- start_y), 2) );
    start_x = odometry_msg->pose.pose.position.x;
    start_y = odometry_msg->pose.pose.position.y;
      
    if ( sample_distance >= S0)
    {
      double raw, pitch, theta;
     tf::Quaternion q;
     tf::quaternionMsgToTF(odometry_msg->pose.pose.orientation, q);
     tf::Matrix3x3(q).getRPY(raw, pitch, theta);
      FILE *fp_s;
      fp_s = fopen(path_file.c_str(),"a");
      fprintf(fp_s, "%c %lf %lf %lf",p, odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y,theta);                   // Angle
      fprintf(fp_s, "\r\n");  
      fclose(fp_s);
      sample_distance = 0;
      this->count++;      
    } 
   }  
}

void path_record::nodeStart(int argc, char **argv){
    ros::init(argc, argv, "path_record");
    ros::NodeHandle nc;
    ros::Subscriber sub_odom = nc.subscribe("/odom", 1, &path_record::record_path, this);//录制轨迹话题小车里程计
    ros::spin();
}

int main(int argc, char *argv[])
{
    
    path_record node;
    node.nodeStart(argc, argv);
    return(0);
}
