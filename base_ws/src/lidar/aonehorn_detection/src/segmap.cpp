#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "common/types/type.h"
#include <vector>
#include <cmath>

template <typename T>
T clamp(T val, T min, T max) {
    return (val < min) ? min : (val > max) ? max : val;
}

void fillBEVMap(autosense::PointICloudPtr point_obj,
    ros::Publisher& map_pub,  // 퍼블리셔를 함수로 전달
    float resolution, float left_range, float right_range,
    float front_range, float rear_range, float dilu) {

    // OccupancyGrid 메시지 생성
    nav_msgs::OccupancyGrid map_msg;
    int result = static_cast<int>(dilu / resolution);
    
    // map의 크기 결정
    int height = static_cast<int>((left_range + right_range) / resolution);  // 좌우 거리 -> height
    int width = static_cast<int>((front_range + rear_range) / resolution);   // 전후 거리 -> width

    // OccupancyGrid 메시지 설정
    map_msg.header.stamp = ros::Time::now();
    map_msg.header.frame_id = "velodyne";
    map_msg.info.resolution = resolution;
    map_msg.info.width = width;    // x축에 해당하는 크기
    map_msg.info.height = height;  // y축에 해당하는 크기

    // 원점은 좌측 하단에 위치 (left_range 및 rear_range에 따라 설정)
    map_msg.info.origin.position.x = -rear_range;  // 전후 방향의 원점 설정
    map_msg.info.origin.position.y = -right_range;   // 좌우 방향의 원점 설정
    map_msg.info.origin.position.z = 0;
    map_msg.info.origin.orientation.w = 1.0;  // 기본 회전 설정

    // OccupancyGrid의 데이터를 -1로 초기화 (알 수 없는 공간)
    map_msg.data.resize(width * height, 0);  // -1로 초기화 (알 수 없는 공간)

    // 맵 원점은 좌측 하단에 있고, 해당 좌표를 계산하여 셀에 저장
    for (const auto& point : *point_obj) {
        // BEV 좌표 계산 (z 좌표는 무시)
        float map_x_f = (point.x + rear_range) / resolution;  // 전후 거리 (x축)
        float map_y_f = (point.y + right_range) / resolution;   // 좌우 거리 (y축)

        // 정수로 변환하고, 클리핑하여 범위 내로 제한
        int map_x = static_cast<int>(clamp(map_x_f, 0.0f, static_cast<float>(width - 1)));
        int map_y = static_cast<int>(clamp(map_y_f, 0.0f, static_cast<float>(height - 1)));

        // 범위를 벗어나는 포인트는 제외
        if (map_x >= result && map_x < width - result && map_y >= result && map_y < height - result) {
            // 5x5 그리드로 주위 좌표까지 덮기 (자기 자신 포함 25칸)
            for (int dx = -result; dx <= result; ++dx) {
                for (int dy = -result; dy <= result; ++dy) {
                    int nx = map_x + dx;
                    int ny = map_y + dy;

                    // 유효한 좌표 범위인지 다시 한 번 체크
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                        map_msg.data[ny * width + nx] = 1;  // 장애물로 설정
                    }
                }
            }
        }
    }

    // 장애물 간의 빈 공간을 탐색하며 연결하는 작업
    int min_distance = 2;  // 최소 연결 거리 설정 (칸 수)

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (map_msg.data[y * width + x] == 1) {
                // 수평 방향 연결
                for (int nx = x + 1; nx < std::min(x + min_distance, width); ++nx) {
                    if (map_msg.data[y * width + nx] == 1) {
                        for (int fill_x = x + 1; fill_x < nx; ++fill_x) {
                            map_msg.data[y * width + fill_x] = 1;  // 빈 칸을 채움
                        }
                        break;  // 다음 장애물 발견 후 연결 완료
                    }
                }

                // 수직 방향 연결
                for (int ny = y + 1; ny < std::min(y + min_distance, height); ++ny) {
                    if (map_msg.data[ny * width + x] == 1) {
                        for (int fill_y = y + 1; fill_y < ny; ++fill_y) {
                            map_msg.data[fill_y * width + x] = 1;  // 빈 칸을 채움
                        }
                        break;  // 다음 장애물 발견 후 연결 완료
                    }
                }
            }
        }
    }

    // OccupancyGrid 메시지를 퍼블리시
    map_pub.publish(map_msg);
}
