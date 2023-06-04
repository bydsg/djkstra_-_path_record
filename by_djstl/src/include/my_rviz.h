#ifndef _MY_RVIZ_H_
#define _MY_RVIZ_H_

#include "headfile.h"

void rviz_road(ros::Publisher marker_pub, std::vector<Road> roads){
    visualization_msgs::Marker points;
    points.header.frame_id = "odom";
    points.header.stamp = ros::Time::now();
    points.ns = "points_and_lines";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.2;
    points.scale.y = 0.2;
    points.color.g = 1.0f;
    points.color.a = 1.0;

    for (size_t i = 0; i < roads.size(); i++)
    {
        for (size_t j = 0; j < roads[i].road_points.size(); j++)
        {
            geometry_msgs::Point p;
            p.x = roads[i].road_points[j].x;
            p.y = roads[i].road_points[j].y;
            p.z = 0;
            points.points.push_back(p);
        }
    }
    marker_pub.publish(points);
}
void rviz_road2(ros::Publisher marker_pub, by_djstl::Path globle_path){
    visualization_msgs::Marker points;
    points.header.frame_id =  "odom";
    points.header.stamp = ros::Time::now();
    points.ns = "points_and_lines";
    points.action =  visualization_msgs::Marker::ADD;
    points.pose.orientation.w =  1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::LINE_STRIP;
    points.scale.x = 0.01;
    points.scale.y = 0.01;
    points.color.r = 1.0f;
    points.color.a = 1.0;
    for (size_t i = 0; i < globle_path.points.size(); i++)
    {  
             geometry_msgs::Point p;
             p.x =  globle_path.points[i].x ;
             p.y =  globle_path.points[i].y;
             p.z =  0;
             points.points.push_back(p);        
    }
    marker_pub.publish(points);
}


#endif