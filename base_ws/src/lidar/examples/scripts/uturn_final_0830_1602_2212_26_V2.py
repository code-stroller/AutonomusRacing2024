#!/usr/bin/env python3
# # -*- coding: utf-8 -*-

import rospy
# import tf
import os
# from std_msgs.msg import Float32MultiArray
# from sensor_msgs.msg import Imu
# from morai_msgs.msg import GPSMessage
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from tracking_msg.msg import TrackingObjectArray
from visualization_msgs.msg import Marker, MarkerArray
# from scipy.spatial import distance
import itertools
from math import sin, cos, pi,sqrt
import math
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseArray, Pose
from custom_msg.msg import PointArray_msg # geometry_msgs/Point[] array -> custom msg 생성 
import numpy as np
import time
from std_msgs.msg import Int32, Bool,String
import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

#############
#### 예선 ####
#############

class GPS2UTM:
    def __init__(self):
        rospy.loginfo("Uturn is Created")

        # ------------------------- Subscriber ----------------------
        #rospy.Subscriber("/lidar/tracking_objects", TrackingObjectArray, self.lidar_callback)
        rospy.Subscriber("/State",String,self.state_callback)
        rospy.Subscriber("odom_gps", Odometry, self.odom_callback)
        rospy.Subscriber("/vehicle_yaw", Float32, self.vehicle_yaw_callback)
        rospy.Subscriber("/tracking_node/cone",Float32MultiArray,self.lidar_callback)
        # -------------------------- Marker ----------------------
        #self.middle_point_pub = rospy.Publisher("middle_point", Marker, queue_size=10)
        self.middle_point_pub = rospy.Publisher('middle_point', MarkerArray, queue_size=10)
        self.rabacone_point_pub = rospy.Publisher('rabacone', MarkerArray, queue_size=10)
        self.all_rabacone_point_pub = rospy.Publisher('all_rabacone', MarkerArray, queue_size=10)
        self.lane_point_pub = rospy.Publisher("lane_point", Marker, queue_size=10)
        # ------------------------- Publisher ----------------------------
        self.target_point_publisher = rospy.Publisher("uturn_point", Float32MultiArray, queue_size=10)
        self.obstacle_state_pub = rospy.Publisher('obstacle_state', String, queue_size=5)
        self.bev_pub = rospy.Publisher('bev', PoseArray, queue_size=10)
        # self.uturn_pub = rospy.Publisher('traffic_labacon', Bool, queue_size=10)
        self.mappub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
        self.obs_list = []



        ##################################
        # U-turn 
        self.State="Rubber_cone_drive"
        self.lfirst=False
        self.rfirst=False
        self.min_distance_threshold = 3

        # self.stable=False
        self.line_space = 10
        self.before_obstacle = []
        self._num_paths = 9

        #점유그리드맵
        self.look_up_L = 3
        self.look_up_R = 3
        self.look_up_F = 7
        self.resolution = 0.1
        self.num_path_point = 14
        self.is_odom = False
        self.current_position = Point()
        self.is_yaw = False


    def state_callback(self,state):
        #self.State=state.data
        self.State = "Rubber_cone_drive"
    def odom_callback(self, msg):
        self.is_odom = True
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y
    def vehicle_yaw_callback(self, msg):
        self.is_yaw = True
        self.vehicle_yaw = msg.data
    def euclidean_distance(self, point1, point2):
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

    def make_path_by_side_distance(self, theta, side_dis, path_point_num):
        l_path = []
        r_path = []
        r = 1.04 / math.tan(math.pi * theta / 180)  # 라디안으로 변환 후 반지름 계산

        if side_dis >= r + 0.01:
            for i in range(path_point_num ):
                angle = (90 / path_point_num) * (i + 1) * math.pi / 180
                sin_val = r * math.sin(angle)
                cos_val = r * (math.cos(angle) - 1)
                l_path.append([sin_val, cos_val])
                r_path.append([sin_val, -cos_val])
            l_path.append([r+0.2, side_dis])
            r_path.append([r+0.2, -side_dis])
            '''elif r > 13:
                for i in range(path_point_num + 1):
                    x = (self.look_up_F / (path_point_num + 1)) * (i + 1)
                    l_path.append([x, 0])
                    r_path.append([x, 0])'''
    
        else:
            for i in range(path_point_num + 1):
                angle = (np.arccos((r - side_dis) / r) / path_point_num) * (i + 1)
                sin_val = r * math.sin(angle)
                cos_val = r * (math.cos(angle) - 1)
                l_path.append([sin_val, cos_val])
                r_path.append([sin_val, -cos_val])

        return l_path, r_path



    def make_occ_grid_map(self, l_distance, r_distance, forward_distance, resolution=0.1):
        return np.zeros((int((l_distance + r_distance) / resolution) + 1, int(forward_distance / resolution) + 1))

    def fill_map(self, map, obj_xs, obj_ys, obj_length, obj_width, resol, left_side, originofmap, fill_probs, yaw_rad):
        obj_xs = int(obj_xs / resol)
        obj_ys = int((+left_side+obj_ys) / resol)
        obj_length = int(obj_length / resol)
        obj_width = int(obj_width / resol)
    
        # `center_x`와 `center_ys` 변수를 `obj_xs`와 `obj_ys`로 변경
        c1 = [
            obj_xs + np.cos(yaw_rad) * obj_length / 2 + np.sin(yaw_rad) * obj_width / 2,
            obj_ys + np.sin(yaw_rad) * obj_length / 2 - np.cos(yaw_rad) * obj_width / 2
        ]
        c2 = [
            obj_xs - np.cos(yaw_rad) * obj_length / 2 + np.sin(yaw_rad) * obj_width / 2,
            obj_ys - np.sin(yaw_rad) * obj_length / 2 - np.cos(yaw_rad) * obj_width / 2
        ]
        c3 = [
            obj_xs - np.cos(yaw_rad) * obj_length / 2 - np.sin(yaw_rad) * obj_width / 2,
            obj_ys - np.sin(yaw_rad) * obj_length / 2 + np.cos(yaw_rad) * obj_width / 2
        ]
        c4 = [
            obj_xs + np.cos(yaw_rad) * obj_length / 2 - np.sin(yaw_rad) * obj_width / 2,
            obj_ys + np.sin(yaw_rad) * obj_length / 2 + np.cos(yaw_rad) * obj_width / 2
        ]

        corners = [c1, c2, c3, c4]

        # Clipping each corner
        clipped_corners = []
        for corner in corners:
            pix_x = int(corner[0])        if(uturn_cone != [] ):
            #print(np.shape(uturn_cone))

            
            #print(mid_point)
            target_point=Float32MultiArray()
            #target_point.data.append(uturn_cone[3][])
            if len(uturn_cone)<5:
                target_point.data.append(0)
            else:
                target_point.data.append(uturn_cone[4][1]/uturn_cone[4][0]*1)

            self.target_point_publisher.publish(target_point)
            #print(np.shape(uturn_cone))
            self.publish_obstacles(uturn_cone, self.middle_point_pub, color=(1.0, 1.0, 0.0))
            #self.publish_obstacles([[1,uturn_cone[4][1]/uturn_cone[4][0]*1]], self.middle_point_pub, color=(1.0, 1.0, 0.0))
            #self.publish_obstacles_array(left_cones,right_cones, self.rabacone_point_pub, color=(1.0, 1.0, 0.0))

        else:
            target_point.data.append(0)
            self.target_point_publisher.publish(target_point)
            # Clipping the x-coordinate
            if pix_x < 0:
                pix_x = 0
            elif pix_x >= len(map[0]):
                pix_x = len(map[0]) - 1

            # Clipping the y-coordinate
            if pix_y < 0:
                pix_y = 0
            elif pix_y >= len(map):
                pix_y = len(map) - 1

            clipped_corners.append([pix_x, pix_y])

        def sort_corners(corners):
            corners = np.array(corners)  # 리스트를 numpy 배열로 변환
            center = np.mean(corners, axis=0)
            angles = np.arctan2(corners[:, 1] - center[1], corners[:, 0] - center[0])
            return corners[np.argsort(angles)]

        # Sort corners in clockwise order
        clipped_corners = sort_corners(clipped_corners)

        def fill_polygon(map, corners):
            # Get the bounding box of the polygon
            min_y = np.min(corners[:, 1])
            max_y = np.max(corners[:, 1])

            # Loop through each scanline
            for y in range(min_y, max_y + 1):
                intersections = []
                for i in range(len(corners)):
                    j = (i + 1) % len(corners)
                    y1, y2 = corners[i][1], corners[j][1]
                    x1, x2 = corners[i][0], corners[j][0]

                    if y1 == y2:
                        continue  # Skip horizontal edges

                    if (y >= y1 and y < y2) or (y >= y2 and y < y1):
                        x_intersect = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
                        intersections.append(int(x_intersect))

                # Sort the intersection points and fill between pairs
                intersections.sort()

                for k in range(0, len(intersections), 2):
                    x_start = intersections[k]
                    x_end = intersections[k + 1]
                    map[y, x_start:x_end + 1] = fill_probs

        fill_polygon(map, clipped_corners)

    
    def interpolate_and_fill(self, obs_1, obs_2, occ_grid_map, max_len, max_wid, origin,fill_probs,yaw_rad):
        obj_x1 = obs_1[0]
        obj_y1 = obs_1[1]
        obj_x2 = obs_2[0]
        obj_y2 = obs_2[1]

        if abs(obj_x1 - obj_x2) >= abs(obj_y1 - obj_y2):
            for j in np.arange(obj_x1, obj_x2, self.resolution):
                y = (j - obj_x1) * (obj_y1 - obj_y2) / (obj_x1 - obj_x2) + obj_y1
                self.fill_map(occ_grid_map, j, y, self.resolution*3, self.resolution*3, self.resolution, self.look_up_L, origin,fill_probs,yaw_rad)
        else:
            for j in np.arange(obj_y1, obj_y2, self.resolution):
                x = (j - obj_y1) * (obj_x1 - obj_x2) / (obj_y1 - obj_y2) + obj_x1
                self.fill_map(occ_grid_map, x, j, self.resolution*3, self.resolution*3, self.resolution, self.look_up_L, origin,fill_probs,yaw_rad)

    def find_obstacle_distance(self, path, occ_grid_map, origin):
        for j in range(len(path) - 1):
            x1, y1 = path[j]
            x2, y2 = path[j + 1]

            if abs(x1 - x2) >= abs(y1 - y2):
                for x_sys in np.arange(x1, x2, self.resolution):
                    xx, yy = self.calculate_coordinates(x_sys, x1, y1, x2, y2, occ_grid_map, origin)
                    yy = int(np.clip(yy + origin[0], 0, occ_grid_map.shape[0] - 1))
                    xx = int(np.clip(xx, 0, occ_grid_map.shape[1] - 1))
                    if occ_grid_map[yy, xx] == 1:
                        return np.sqrt((yy - origin[0]) ** 2 + xx ** 2)
            else:
                for y_sys in np.arange(y1, y2, self.resolution):
                    yy, xx = self.calculate_coordinates(y_sys, y1, x1, y2, x2, occ_grid_map, origin)
                    yy = int(np.clip(yy - origin[0], 0, occ_grid_map.shape[0] - 1))
                    xx = int(np.clip(xx, 0, occ_grid_map.shape[1] - 1))
                    if occ_grid_map[yy, xx] == 1:
                        return np.sqrt((yy + origin[0]) ** 2 + xx ** 2)
        return 40

    def calculate_coordinates(self, sys_coord, coord1, other_coord1, coord2, other_coord2, occ_grid_map, origin):
        coord = sys_coord / self.resolution
        other_coord = ((sys_coord - coord1) * (other_coord2 - other_coord1) / (coord2 - coord1)) + other_coord1
        other_coord = other_coord / self.resolution
        return coord, other_coord

    def calculate_longitude_latitude(self,delta_x,delta_y):
        start_utm_easting, start_utm_northing = self.current_position.x, self.current_position.y
        heading_rad = self.vehicle_yaw

        delta_utm_easting = delta_x * math.cos(heading_rad) - delta_y * math.sin(heading_rad)
        delta_utm_northing = delta_x * math.sin(heading_rad) + delta_y * math.cos(heading_rad)

        end_utm_easting = start_utm_easting + delta_utm_easting
        end_utm_northing = start_utm_northing + delta_utm_northing

        return end_utm_easting, end_utm_northing

    def sort_vertices_clockwise(self,vertices):
        # 다각형의 중심 계산
        center_x = sum([v[0] for v in vertices]) / len(vertices)
        center_y = sum([v[1] for v in vertices]) / len(vertices)

        # 각 꼭짓점의 중심에서의 각도를 계산
        def angle_from_center(vertex):
            return math.atan2(vertex[1] - center_y, vertex[0] - center_x)

        # 각도에 따라 정렬 (시계방향)
        return sorted(vertices, key=angle_from_center)

    def cross_product(self,v1, v2):
        # 벡터 v1과 v2의 크로스 프로덕트의 z 성분을 계산
        return v1[0] * v2[1] - v1[1] * v2[0]

    def is_point_in_convex_polygon(self,polygon, point):
        prev_cross_product = None
        n = len(polygon)

        for i in range(n):
            # 현재 변의 두 꼭짓점
            p1 = polygon[i]
            p2 = polygon[(i + 1) % n]

            # 변 벡터
            edge_vector = (p2[0] - p1[0], p2[1] - p1[1])
            # 점과 첫 번째 꼭짓점을 연결하는 벡터
            point_vector = (point[0] - p1[0], point[1] - p1[1])

            # 현재 변과 점을 이용한 크로스 프로덕트 계산
            cross_prod = self.cross_product(edge_vector, point_vector)

            # 첫 번째 크로스 프로덕트를 저장
            if prev_cross_product is None:
                prev_cross_product = cross_prod

            # 현재 크로스 프로덕트가 방향이 다른 경우, 점이 다각형 외부에 있음
            if cross_prod * prev_cross_product < 0:
                return False

        # 모든 크로스 프로덕트가 같은 방향을 가리키면 점은 다각형 내부에 있음
        return True
    def lidar_callback(self, _data):

        rows = _data.layout.dim[0].size
        cols = _data.layout.dim[1].size

        matrix = [
            _data.data[i * cols:(i + 1) * cols] for i in range(rows)
        ]
        for data in matrix:
            a,b = calculate_longitude_latitude(data[0]+0.24,data[1])
            if self.obs_list == []:
                self.obs_list.append([a, b, data[2], data[3], data[4], data[5], data[6]])
            for i,before_data in enumerate(self.obs_list):
                if self.euclidean_distance([a,b],[before_data[0],before_data[1]])<0.1:
                    break
                if i == len(self.obs_list)-1:
                    self.obs_list.append([a,b,data[2],data[3],data[4],data[5],data[6]])

        roi_list = []
        roi_list.append([self.current_position.x + self.look_up_R*np.sin(self.vehicle_yaw),self.current_position.y - self.look_up_R*np.cos(self.vehicle_yaw)])
        roi_list.append([self.current_position.x - self.look_up_L*np.sin(self.vehicle_yaw),self.current_position.y + self.look_up_L*np.cos(self.vehicle_yaw)])
        roi_list.append([self.current_position.x + self.look_up_R*np.sin(self.vehicle_yaw)+self.look_up_F*np.cos(self.vehicle_yaw),self.current_position.y + self.look_up_F*np.sin(self.vehicle_yaw) - self.look_up_R*np.cos(self.vehicle_yaw)])
        roi_list.append([self.current_position.x - self.look_up_L*np.sin(self.vehicle_yaw)+self.look_up_F*np.cos(self.vehicle_yaw),self.current_position.y + self.look_up_F*np.sin(self.vehicle_yaw) + self.look_up_L*np.cos(self.vehicle_yaw)])
        sorted_polygon = self.sort_vertices_clockwise(roi_list)

        inroi_obs = []
        for obs in self.obs_list:
            if self.is_point_in_convex_polygon(sorted_polygon, [obs[0],obs[1]]):
            inroi_obs.append(obs)
        pairs = [
            pair for pair in itertools.combinations(inroi_obs, 2)
            if self.euclidean_distance(pair[0], pair[1]) <= self.min_distance_threshold
        ]
    
        pairs.extend([[obstacle, obstacle] for obstacle in matrix])

        occ_grid_map = self.make_occ_grid_map(self.look_up_L, self.look_up_R, self.look_up_F, self.resolution)
        origin = (int(self.look_up_L / self.resolution), 0)

        for obs_1, obs_2 in pairs:
            obj_x1, obj_y1, obj_l1, obj_w1 = obs_1[:4]
  
            obj_x2, obj_y2, obj_l2, obj_w2 = obs_2[:4]
            max_len = max(obj_l1, obj_l2)
            max_wid = max(obj_w1, obj_w2)
            obj_yaw1 = obs_1[6]
            obj_yaw2 = obs_2[6]
            if obs_1 == obs_2:
                self.fill_map(occ_grid_map, obj_x1, obj_y1, obj_l1, obj_w1, self.resolution, self.look_up_L, origin,1,obj_yaw1)
            else:
                self.interpolate_and_fill(obs_1, obs_2, occ_grid_map, self.resolution*5, self.resolution*5, origin,1,obj_yaw1)

                
        scd = np.linspace(2, 20, self.num_path_point)
        points_num = 5
        
        distance_list2 = []

        l_find = 0
        r_find = 0
        path = []
        for degre in scd:

            #left_side_path, right_side_path = self.make_path_by_side_distance(degre, min(self.look_up_L, self.look_up_R), points_num)
            right_side_path, left_side_path = self.make_path_by_side_distance(degre, 1, points_num)
            path.append(right_side_path)
            path.insert(0,left_side_path)
            l_d = self.find_obstacle_distance(left_side_path, occ_grid_map, origin)
            r_d = self.find_obstacle_distance(right_side_path, occ_grid_map, origin)

            distance_list2.append(r_d)
            distance_list2.insert(0,l_d)
        uturn_cone = []
        l_theta = scd[0]
        r_theta = scd[0]
        #print(distance_list2)
        paaath_dis = []
        for j in range(3,len(distance_list2)-3):
            paaath_dis.append(distance_list2[j-3]+distance_list2[j-2]+distance_list2[j-1]+distance_list2[j+3]+distance_list2[j+2]+distance_list2[j+1]+distance_list2[j])

        #print(paaath_dis)

        if np.mean(paaath_dis) >=300:
            uturn_cone = path[int(self.num_path_point) +3]
            #print(int(self.num_path_point/2))
        else:
            abc = np.clip(paaath_dis[paaath_dis.index(min(paaath_dis)):].index(max(paaath_dis[paaath_dis.index(min(paaath_dis)):]))+5+paaath_dis.index(min(paaath_dis)),0,len(path))
            #print(abc)
            if abc <14:
                abc = int(self.num_path_point)
            uturn_cone = path[abc]
            




        
        self.publish_obstacles(uturn_cone, self.middle_point_pub, color=(1.0, 1.0, 0.0))
        '''
        for i in ldf[0]:
            #print(i)
            if self.is_real_obstacle(distance_list2, i,window_size = 2):
                uturn_cone = path[i]
                print(1)
        
        if uturn_cone ==[]:
            if max(distance_list2) ==0:
                uturn_cone = path[int(len(distance_list2)/2)]
                print(3)
            else:
                uturn_cone = path[distance_list2.index(max(distance_list2))]
                print(2)
        for path12 in path:

            pairs1 = [ pair for pair in itertools.combinations(path12, 2)]

            for i in pairs1:
                self.interpolate_and_fill(i[0], i[1], occ_grid_map, 0.3, 0.3, origin,0.2)
    print(ans)
        # U턴 판단 논리
        if l_find == 0 and r_find == 0:
            uturn_cone = path[0][1]  # 왼쪽과 오른쪽 모두 탐지되지 않은 경우
            print(1)
        elif l_find != 0 and r_find == 0:
            uturn_cone = path[int(np.clip(r_theta - 1, 0, len(path[0]) - 1))][0]  # 오른쪽만 탐지되지 않은 경우
            print(2)
        elif l_find == 0 and r_find != 0:
            uturn_cone = path[int(np.clip(l_theta - 1, 0, len(path[0]) - 1))][1]  # 왼쪽만 탐지되지 않은 경우
            print(3)
        else:
            
            if l_theta - r_theta >= 0:
                aa = 0
            else:
                aa = 1
            uturn_cone = path[abs(l_theta - r_theta)][aa] 
            print("123123213")
            print(l_theta)
            print(r_theta)
            print(4)

        
        '''

        self.publish_grid_map(self.mappub,occ_grid_map)
        self.publish_obstacles_array_one(matrix, self.all_rabacone_point_pub, color=(1.0, 1.0, 0.0))
        
        #print(uturn_cone != [])
        if(uturn_cone != [] ):
            #print(np.shape(uturn_cone))

            
            #print(mid_point)
            target_point=Float32MultiArray()
            #target_point.data.append(uturn_cone[3][])
            if len(uturn_cone)<5:

                target_point.data.append(1)
                target_point.data.append(0)
            else:
                for uturn_point in uturn_cone:
                    target_point.data.append(uturn_point[0])
                    target_point.data.append(uturn_point[1])

            self.target_point_publisher.publish(target_point)
            #print(np.shape(uturn_cone))
            self.publish_obstacles(uturn_cone, self.middle_point_pub, color=(1.0, 1.0, 0.0))
            #self.publish_obstacles([[1,uturn_cone[4][1]/uturn_cone[4][0]*1]], self.middle_point_pub, color=(1.0, 1.0, 0.0))
            #self.publish_obstacles_array(left_cones,right_cones, self.rabacone_point_pub, color=(1.0, 1.0, 0.0))

        else:
            target_point.data.append(0)
            self.target_point_publisher.publish(target_point)
        uturn_cone = None
        left_cones = None
        right_cones = None
    def cal_obs_data(self,delta_x,delta_y):
        x=delta_x
        y=delta_y

        obs_angle = np.rad2deg(math.atan2(y,x))
        obs_dist = np.sqrt(x**2+y**2)
        
        return obs_angle, obs_dist
    
    def publish_obstacles(self, obs, publisher, color):
        #print(len(obs))
        if obs is not None:
            #print(obs)
            marker_array = MarkerArray()
            for i, odf in enumerate(obs):
                #print(odf)
                x, y = odf[0],odf[1]
                # Marker 메시지를 생성하여 장애물들을 크고 입체적으로 시각화
                marker = Marker()
                marker.header.frame_id = "velodyne"  # 필요에 따라 적절한 프레임으로 변경
                marker.header.stamp = rospy.Time.now()
                marker.ns = "obstacles"
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD

                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = 0.0  # 입체적으로 보이기 위해 z 좌표를 0 이상으로 설정
                marker.scale.x = 0.6  # 포인트 크기
                marker.scale.y = 0.6
                marker.scale.z = 0.6
                marker.color.a = 1.0
                marker.color.r = color[0]
                marker.color.g = color[1]
                marker.color.b = color[2]
                marker_array.markers.append(marker)
            publisher.publish(marker_array)
    def publish_grid_map(self,pub,grid_map):


        # OccupancyGrid 메시지 생성
        grid_msg = OccupancyGrid()
        grid_msg.header = Header()
        grid_msg.header.frame_id = "velodyne"

        grid_msg.info.resolution = self.resolution  # 셀 하나의 크기 (미터 단위)
        grid_msg.info.width = grid_map.shape[1]
        grid_msg.info.height = grid_map.shape[0]
        grid_msg.info.origin.position.x = 0.0
        grid_msg.info.origin.position.y = -self.look_up_L
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0

        # OccupancyGrid 데이터 채우기
        grid_data = grid_map.flatten()
        grid_data = (grid_data * 100).astype(np.int8)  # 0에서 100 사이의 값으로 스케일링
        grid_msg.data = grid_data.tolist()

            # 퍼블리시
        pub.publish(grid_msg)
    def publish_obstacles_array(self, left,right, publisher, color):
        if left is not None:
            # print(obs)
            marker_array = MarkerArray()
            for idx, objexz in enumerate(left):

                x, y = objexz[0], objexz[1]
                # Marker 메시지를 생성하여 장애물들을 크고 입체적으로 시각화
                marker = Marker()
                marker.header.frame_id = "velodyne"  # 필요에 따라 적절한 프레임으로 변경
                marker.header.stamp = rospy.Time.now()
                marker.ns = "obstacles"
                marker.id = idx
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD

                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = 0.0  # 입체적으로 보이기 위해 z 좌표를 0 이상으로 설정
                marker.scale.x = 0.6  # 포인트 크기
                marker.scale.y = 0.6
                marker.scale.z = 0.6
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker_array.markers.append(marker)
        if right is not None:
            # print(obs)
            for idx, objexz in enumerate(right):
                x, y = objexz[0], objexz[1]
                # Marker 메시지를 생성하여 장애물들을 크고 입체적으로 시각화
                marker = Marker()
                marker.header.frame_id = "velodyne"  # 필요에 따라 적절한 프레임으로 변경
                marker.header.stamp = rospy.Time.now()
                marker.ns = "obstacles"
                marker.id = idx + len(left)
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD

                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = 0.0  # 입체적으로 보이기 위해 z 좌표를 0 이상으로 설정
                marker.scale.x = 0.6  # 포인트 크기
                marker.scale.y = 0.6
                marker.scale.z = 0.6
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker_array.markers.append(marker)
            publisher.publish(marker_array)
    def publish_obstacles_array_one(self, left, publisher, color):
        if left is not None:
            # print(obs)
            marker_array = MarkerArray()
            for idx, objexz in enumerate(left):

                x, y = objexz[0], objexz[1]
                # Marker 메시지를 생성하여 장애물들을 크고 입체적으로 시각화
                marker = Marker()
                marker.header.frame_id = "velodyne"  # 필요에 따라 적절한 프레임으로 변경
                marker.header.stamp = rospy.Time.now()
                marker.ns = "obstacles"
                marker.id = idx + 100
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD

                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = -1.0  # 입체적으로 보이기 위해 z 좌표를 0 이상으로 설정
                marker.scale.x = 0.8  # 포인트 크기
                marker.scale.y = 0.8
                marker.scale.z = 1.0
                marker.color.a = 1.0
                marker.color.r = 0.3
                marker.color.g = 0.3
                marker.color.b = 0.3
                marker_array.markers.append(marker)
            publisher.publish(marker_array)
    def calculate_bounding_box_center(self, bev_coords):
        center_x = (bev_coords[0] + bev_coords[2] + bev_coords[4] + bev_coords[6]) / 4
        center_y = (bev_coords[1] + bev_coords[3] + bev_coords[5] + bev_coords[7]) / 4
        return center_x, center_y

    def calculate_bounding_box_dimensions(self, bev_coords):
        width = math.sqrt((bev_coords[2] - bev_coords[0]) ** 2 + (bev_coords[3] - bev_coords[1]) ** 2)
        height = math.sqrt((bev_coords[4] - bev_coords[2]) ** 2 + (bev_coords[5] - bev_coords[3]) ** 2)
        return width, height

    def calculate_angle_with_vehicle(self, center_x, center_y, vehicle_x, vehicle_y):
        angle_rad = math.atan2(center_y - vehicle_y, center_x - vehicle_x)
        angle_deg = math.degrees(angle_rad)
        return angle_deg

    def calculate_distance_to_vehicle(self, center_x, center_y, vehicle_x, vehicle_y):
        distance = math.sqrt((center_x - vehicle_x) ** 2 + (center_y - vehicle_y) ** 2)
        return distance


def run():
    rospy.init_node("uturn")
    new_classs= GPS2UTM()
    rospy.spin()
    

if __name__ == '__main__':
    run()
