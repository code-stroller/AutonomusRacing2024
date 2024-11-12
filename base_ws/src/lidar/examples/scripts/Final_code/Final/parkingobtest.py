#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point,TwistStamped
from tracking_msg.msg import TrackingObjectArray
# from gps import GPS2UTM
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import numpy as np
from math import atan,cos,sin,tan
import time
from std_msgs.msg import Int32, Float32, String

# 주차 표지판 받았을 때 실행 

class LidarReceiver():
    def __init__(self):
        # super().__init__()

        rospy.loginfo("Jung OOng's empty, LiDAR Receiver Object is Created")
        rospy.Subscriber("/lidar/tracking_objects", TrackingObjectArray, self.lidar_callback)
        rospy.Subscriber("/odom_gps", Odometry, self.odom_callback)
        rospy.Subscriber("/vel",TwistStamped,self.velocity_callback)
        rospy.Subscriber("/State", String ,self.state_callback)
        rospy.Subscriber("/vehicle_yaw", Float32, self.vehicle_yaw_callback)

        self.parking_marker_pub = rospy.Publisher('parking_marker', Marker, queue_size=10)
        self.parking_lot_pub = rospy.Publisher('parking_plot', Int32, queue_size = 1)
        self.array=[]

        self.vel=float("inf")
        self.is_odom=False
        self.current_position = Point()

        self.is_state=False
        self.is_yaw=False

        self.accuracy_check=0
        self.before_num = float("inf")

        self.LW=0.25
        self.calculate_idx = 0
        self.parking_num = -1
        self.lidar_header = 0
        

    
    def velocity_callback(self, vel):
        self.is_vel=True
        self.vel=np.sqrt((vel.twist.linear.x)**2+(vel.twist.linear.y)**2)
        if np.isnan(self.vel):
            self.vel=0

    
    def odom_callback(self,msg):
        self.is_odom=True
        self.current_position.x=msg.pose.pose.position.x
        self.current_position.y=msg.pose.pose.position.y
        # self.vehicle_yaw = msg.pose.pose.position.z + 0.1


    def vehicle_yaw_callback(self, msg):
        self.is_yaw = True
        self.vehicle_yaw = msg.data

        
    def state_callback(self, msg):  
        '''
        해당 미션에 들어올 경우에만 계산 시작하도록 하기위해 사용
        미션 들어왔는지 여부는 제어팀에서 GPS이용해서 정함
        '''
        self.is_state = True
        self.State = msg.data

        # self.State = "Parking"
        # print('static_trigger : ', self.static_trigger)
    
        
    
    def lidar_callback(self, msg):
        '''
        현재 평행 주차구역 감지 방법은 도로쪽 모서리의 좌표를 직접 이용하여 계산하는 방식임. 
        두 모서리의 좌표의 중심을 계산 한 후 이 점 근처에 라바콘이 있는지 판별함. 
        중심점에 클러스터링 된 데이터가 있을경우(라바콘) 막혀있다는 뜻이니 따로 발행하는것 없음
        중심점 근처에 없을경우 해당 구역의 번호를 발행해 제어쪽에서 제어 가능하도록 함
        '''
        if self.is_state and self.State == "Parking":

            self.lidar_header = msg.header #마커 헤더 표시용

            self.marker_array = MarkerArray()
            parking_lots = [
                # 주차 미션에서 도로쪽 각 모서리 좌표 저장용 코드
                # 지석이가 딴 최종 주차 Bag 에서 가져옴
                [(302480.4487,4123762.6654), (302478.0663,4123758.3998)],  # 구역 1  
                [(302478.0663,4123758.3998), (302476.0672,4123754.8171)], # 구역 2
                [(302476.0672,4123754.8171), (302473.7855,4123750.5588)] # 구역 3 
                
                
            ]
                # 종인이형이랑 직접 GPS가져다 두고 한 값. 
                # [(302480.2301,4123762.547), (302477.8601,4123758.2216)],  # 구역 1  
                # [(302477.8601,4123758.2216), (302475.4082,4123753.8655)], # 구역 2
                # [(302475.4082,4123753.8655), (302472.2757,4123748.2583)] # 구역 3 
                
                
            obstacle_points = []
            
            for obj in msg.array:
                if len(obj.bev.data) < 8:
                    rospy.logwarn("Invalid bounding box format: %s", obj.bev.data)
                    continue

                bbox_center_x, bbox_center_y = self.calculate_bounding_box_center(obj.bev.data)
                # -> ENU 
                self.delta_x = bbox_center_x
                self.delta_y = bbox_center_y
                end_utm_easting, end_utm_northing = self.calculate_longitude_latitude(self.delta_x,self.delta_y)

                # print(end_utm_easting,end_utm_northing)

                #조건에 맞는 물체만 포함.
                if -5<bbox_center_y<0. and bbox_center_x < 9.:
                    obstacle_points.append((end_utm_easting, end_utm_northing))
                    # print("x,y",bbox_center_x,bbox_center_y)
                    print(end_utm_easting,end_utm_northing)
                
            #주차장 각 모서리를 이용해 
            for i, parking_lot in enumerate(parking_lots):
                is_inside_obstacle = self.is_point_inside_polygon(parking_lot, obstacle_points)
                print('obj is in Parking place num', i, is_inside_obstacle)
                if is_inside_obstacle:
                    self.parking_lot_pub.publish(i+1)
                


      ############################################################################################
      
    def parking_place_marker(self, header, frame_id, x, y, id):
        marker1 = Marker()
        marker1.header = header
        marker1.header.frame_id = frame_id
        marker1.type = Marker.SPHERE  # You can choose the marker type you prefer
        marker1.action = Marker.ADD
        marker1.ns = "marker set2"
        marker1.id=id
        marker1.pose.position.x = x
        marker1.pose.position.y = y
        marker1.pose.position.z = 0
        marker1.scale.x = 1.0  # Adjust the marker size as needed
        marker1.scale.y = 1.0
        marker1.scale.z = 1.0
        marker1.color.a = 1.0  # Alpha (transparency)
        marker1.color.r = 0.0  # Red color
        marker1.color.g = 0.0  # Green color
        marker1.color.b = 1.0  # Blue color
        self.parking_marker_pub.publish(marker1)

    def calculate_bounding_box_center(self, bev_coords):
        # bev_coords: [x1, y1, x2, y2, x3, y3, x4, y4]
        center_x = (bev_coords[0] + bev_coords[2] + bev_coords[4] + bev_coords[6]) / 4
        center_y = (bev_coords[1] + bev_coords[3] + bev_coords[5] + bev_coords[7]) / 4
        return center_x, center_y



    def is_point_inside_polygon(self, rectangle_points, objects):
        '''
        주차장의 모서리 두 점을 이용해서 두 점의 중심을 원점으로 하여 좌표 변환. 
        원점으로부터의 거리를 이용해서 비어있는지 측정하는 방식
        '''
        # 거리 임계값
        dis_threshold = 1.75   # 실제로 객체가 가까운지 판단할 거리 기준
        start_distance_threshold = 7  # 계산을 시작할 거리 기준

        # 두 점의 중심 계산
        p1x, p1y = rectangle_points[0]
        p2x, p2y = rectangle_points[1]
        center_x = (p1x + p2x) / 2 
        center_y = (p1y + p2y) / 2 
        
        # 로컬 좌표로 변환
        center_x_local, center_y_local = self.calculate_local_coordinates(center_x, center_y)
        print(f"center_x_local coord {center_x_local:.2f}")
        print(f"center_y_local coord {center_y_local:.2f}")
        
        # 로컬 좌표 기준으로 음수인지 확인
        if center_x_local > 0:
            center_dist = math.sqrt((center_x_local)**2 + (center_y_local)**2)
            # print(f"calc {center_dist:.2f}")

            if center_dist < start_distance_threshold:  # 중간점이 8m 이내에 들어왔을때 계산 시작(주변 라바콘 다 보이는 시점)
                # 주어진 객체들과 중심점 간의 거리 계산
                for obj in objects:
                    # 객체와 중심점 사이의 거리
                    obj_to_center = math.sqrt((obj[0] - center_x)**2 + (obj[1] - center_y)**2)

                    # 만약 일정 거리 안에 객체가 있으면 False 반환 (비어 있지 않음)
                    if obj_to_center < dis_threshold:
                        return False
                    # print('calculated obj:', obj )

                # 일정 거리 안에 아무 객체도 없으면 True 반환 (비어 있음)

                
                return True
        else:
            pass


    def calculate_local_coordinates(self, utm_easting, utm_northing):
        """
        UTM 좌표 (utm_easting, utm_northing)를 로컬 좌표계 (delta_x, delta_y)로 변환하는 함수.
        """
        
        # 차량의 헤딩(방위각) 라디안 값
        heading_rad = self.vehicle_yaw

        # 현재 위치로부터 UTM 좌표까지의 변화량
        delta_utm_easting = utm_easting - self.current_position.x
        delta_utm_northing = utm_northing - self.current_position.y

        # UTM 좌표에서 로컬 좌표로 변환
        delta_x = delta_utm_easting * math.cos(heading_rad) + delta_utm_northing * math.sin(heading_rad)
        delta_y = -delta_utm_easting * math.sin(heading_rad) + delta_utm_northing * math.cos(heading_rad)

        # 차량의 좌표계를 기준으로 로컬 좌표 반환
        delta_x -= self.LW  # LW 값을 보정하여 로컬 좌표계로 변환
        return delta_x, delta_y
            

    def calculate_longitude_latitude(self,delta_x,delta_y):
        
        heading_rad = self.vehicle_yaw
        # print("heading_rad", heading_rad)

        delta_x+=self.LW

        delta_utm_easting = delta_x * math.cos(heading_rad) - delta_y * math.sin(heading_rad)
        delta_utm_northing = delta_x * math.sin(heading_rad) + delta_y * math.cos(heading_rad)

        end_utm_easting = self.current_position.x + delta_utm_easting
        end_utm_northing = self.current_position.y + delta_utm_northing

        return end_utm_easting, end_utm_northing
    
    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw


if __name__ == '__main__':
    try:
        rospy.init_node('parking_plot_recognition', anonymous=True)
        rospy.loginfo("Node started")
        receiver = LidarReceiver()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
