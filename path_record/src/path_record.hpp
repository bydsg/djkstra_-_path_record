#include<vector>
#include <ros/ros.h>
#include <time.h>
#include <math.h>
#include <numeric>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistStamped.h>
#include <string>
#include <limits>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#define  S0  0.1     // 采样距离

class path_record
{
private:

    void record_path(const nav_msgs::Odometry::ConstPtr odometry_msg);

public:
   double sample_distance = 0;
   double start_x = 0;
   double start_y = 0;
   int count = 0;
  
  void nodeStart(int argc, char **argv);
} ;



