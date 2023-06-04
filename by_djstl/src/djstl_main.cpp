#include "include/headfile.h"
using namespace std;

//加载txt中路径
 std::vector<Road> load_direction_Road(const std::string fileName)
{
    std::vector<Road> roads;
    bool record_points = false;
    Road road;
    std::string line;
    std::string line3;
    std::ifstream fs;
    fs.open(fileName, std::ios::in);
    while (getline(fs, line))
    {
        if (line.length() > 0)
        {
            std::stringstream ss(line);
            std::stringstream sss(line);
            std::string line2;
            int id;
            ss >> line2;
            if (line2 == "road")
            {
                ss >> id;
                road.id = id;
                continue;
            }

            if (line2 == "point")
            {
                record_points = true;
                continue;
            }

            if (line2 != "pre" && record_points)
            {
                Point point;
                point.l = 2;
                point.r = 2;
                ss >> point.x >> point.y >> point.theta;
                road.road_points.push_back(point);
            }

            if (line2 == "pre")
            {
                int count = 0;
                while (getline(sss, line3, ' '))
                {
                    count++;
                    if (count >= 2)
                    {
                        std::stringstream ssss(line3);
                        int id;
                        ssss >> id;
                        road.pre.push_back(id);
                    }
                }
                record_points = false;
                continue;
            }

            if (line2 == "beh")
            {
                int count = 0;
                while (getline(sss, line3, ' '))
                {
                    count++;
                    if (count >= 2)
                    {
                        std::stringstream ssss(line3);
                        int id;
                        ssss >> id;
                        road.beh.push_back(id);
                    }
                }

                record_points = false;
                roads.push_back(road);
                road = {0};
                continue;
            }
        }
    }

    fs.close();
    return roads;
}
class globle_planning
{
public:
    ros::Publisher marker_pub;
    ros::Publisher pub_globle_path;
    ros::Publisher globle_path_marker_pub;
    ros::Subscriber sub_odom;
    ros::Subscriber sub_end_odom;
    vector<Road> roads;
    Point vehicle_pose;

    void nodeStart(int argc, char **argv);
    void odometryGetCallBack(const nav_msgs::Odometry::ConstPtr odometry_msg);
    void end_odom_callback(const geometry_msgs::PoseStamped::ConstPtr msg );
};

void globle_planning::odometryGetCallBack(const nav_msgs::Odometry::ConstPtr odometry_msg){
// 得到车辆的定位信息和速度信息
    ros::Rate loop_rate(10);
    rviz_road(this->marker_pub, this->roads);  
    double raw, pitch, theta;
    tf::Quaternion q;
    tf::quaternionMsgToTF(odometry_msg->pose.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(raw, pitch, theta);
    this->vehicle_pose.theta = theta;
    this->vehicle_pose.x = odometry_msg->pose.pose.position.x;
    this->vehicle_pose.y = odometry_msg->pose.pose.position.y;
    loop_rate.sleep();
}
void globle_planning::end_odom_callback(const geometry_msgs::PoseStamped::ConstPtr msg ){
    ros::Rate loop_rate(10);
    Point end_point;
    end_point.x = msg->pose.position.x;
    end_point.y = msg->pose.position.y;
    double raw, pitch, theta;
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.orientation, q);
    tf::Matrix3x3(q).getRPY(raw, pitch, theta);
    end_point.theta = theta;

    //加载结构地图
    int dist[maxSize], path[maxSize], v, E;
    Graphlnk<int, int>  G; // 声明图对象
    G.inputGraph(this->roads);     // 创建图
    //G.outputGraph();
    int u0 = 10;
    int e0 = 10;
    int e_nearest = 0;

    double min_distance1 =100000;
    int u1=0;
    int turn_flag=0;
    for (size_t i = 0; i < this->roads.size(); i++)
    {
        for (size_t j = 0; j < this->roads[i].road_points.size(); j++)
        {
            double distance = std::sqrt(std::pow(this->vehicle_pose.x-this->roads[i].road_points[j].x, 2) + std::pow(this->vehicle_pose.y-this->roads[i].road_points[j].y, 2));
            if(distance <= min_distance1){
                u0 = 1+i;//离车最近的道路
                min_distance1 = distance;
                u1=j;//离车最近的道路的点的编号
            }
        }
    }
    double min_distance2 =100000;
    for (size_t i = 0; i < this->roads.size(); i++)
    {
        for (size_t j = 0; j < this->roads[i].road_points.size(); j++)
        {
            double distance =(end_point.x-this->roads[i].road_points[j].x)*(end_point.x-this->roads[i].road_points[j].x)+ (end_point.y-this->roads[i].road_points[j].y)*(end_point.y-this->roads[i].road_points[j].y);
            if(distance <= min_distance2){
                e0 = 1+i;//离目标点最近的道路编号
                e_nearest = j;//离目标点最近的道路的点的编号
                min_distance2 = distance;
            }
        }
    }   

    v = G.getVertexPos(u0); // 取得起始顶点的位置
    E = G.getVertexPos(e0); // 取得起始end的位置
    Dijkstra(G, v, dist, path); // 调用Dijkstra函数
    std::vector<int> road_id;
    road_id = printShortestPath(G, v, E, dist, path); // 输出到各个顶点的最短路径

    int e_id = e0 - 1;                                // 终点道路所在的下标

     ////整合路径，并将路径信息转换成msg信息
      by_djstl::Path globle_path;
      for (size_t i = 0; i < road_id.size(); i++)
      {
        if (i != road_id.size() - 1)
        {
            for (size_t j = 0; j < roads[road_id[i] - 1].road_points.size(); j++)
            {
                by_djstl::PathPoint p;
                p.x = roads[road_id[i] - 1].road_points[j].x;
                p.y = roads[road_id[i] - 1].road_points[j].y;
                p.r = roads[road_id[i] - 1].road_points[j].r;
                p.l = roads[road_id[i] - 1].road_points[j].l;
                globle_path.points.push_back(p);
            }
        }
        else
        {
            for (size_t j = u1; j < e_nearest; j++)
            {
                by_djstl::PathPoint p;
                p.x = roads[road_id[i] - 1].road_points[j].x;
                p.y = roads[road_id[i] - 1].road_points[j].y;
                p.r = roads[road_id[i] - 1].road_points[j].r;
                p.l = roads[road_id[i] - 1].road_points[j].l;
                globle_path.points.push_back(p);
            }
        }
      }
 
    rviz_road2(globle_path_marker_pub, globle_path);
    this->pub_globle_path.publish(globle_path);
    loop_rate.sleep();
}
void globle_planning::nodeStart(int argc, char **argv)
{
    ros::init(argc, argv, "globle_planning");
    ros::NodeHandle nc;

    string roadMap_path = "/home/by/git_bydsg/by_djstl_ws/src/by_djstl/data/road_path.txt";
    this->roads = load_direction_Road(roadMap_path);

    this->pub_globle_path = nc.advertise<by_djstl::Path>("globle_path", 10);//发布全局规划路径给局部路径规划
    this->marker_pub = nc.advertise<visualization_msgs::Marker>("road_rviz", 1);//在rviz上显示录制的路径
    this->globle_path_marker_pub =  nc.advertise<visualization_msgs::Marker>("road_rviz_globle_path", 1);//在rviz上显示规划的全局路径

    // 订阅相关节点
    this->sub_odom = nc.subscribe("/odom", 1, &globle_planning::odometryGetCallBack, this);//控制节点
    this->sub_end_odom = nc.subscribe("/move_base_simple/goal", 1, &globle_planning::end_odom_callback, this);//订阅终点位置

 
   ros::spin();
}


int main(int argc, char  *argv[])
{
    globle_planning node;
    node.nodeStart(argc, argv);
    return(0);
}
