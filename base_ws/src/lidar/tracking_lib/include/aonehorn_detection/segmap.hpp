#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <cmath>
#include "common/types/type.h"  

// BEV 맵을 생성하고 OccupancyGrid 메시지를 퍼블리시하는 함수
void fillBEVMap(autosense::PointICloudPtr point_obj,
    ros::Publisher& map_pub,  // 퍼블리셔를 함수로 전달
    float resolution, float left_range, float right_range,
    float front_range, float rear_range, float dilu);

