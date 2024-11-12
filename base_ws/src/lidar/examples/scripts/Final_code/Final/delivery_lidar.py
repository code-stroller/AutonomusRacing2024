#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
# import tf
import os
import rospkg
from std_msgs.msg import Float32MultiArray,Bool,Int32,Int64,Float32, String
from sensor_msgs.msg import Imu
# from morai_msgs.msg import GPSMessage
from geometry_msgs.msg import Point,Twist,TwistStamped
from tracking_msg.msg import TrackingObjectArray
from math import pi,sqrt
import math
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from custom_msg.msg import PointArray_msg # geometry_msgs/Point[] array -> custom msg 생성 
import numpy as np
import time
from nav_msgs.msg import Odometry
#from microstrain_inertial_msgs.msg import FilterHeading
#from microstrain_inertial_delivery_zone_arriveds.delivery_zone_arrived import FilterHeading
import copy

#############
#### 본선 ####
#############
class LidarProcessor:
    '''
    Lidar 통해 얻은 물체의 좌표를 UTM으로 변환해서 저장하는 Class. 
    물체의 좌표로부터 특정 거리 이내에 찍히는 점들은 무시하고 먼 거리일시 추가.
    '''
    def __init__(self, tolerance, initial_points=None):
        """
        LidarProcessor 클래스 초기화
        :param tolerance: 유효범위 (거리 임계값)
        :param initial_points: 초기 UTM 좌표 리스트 (기본값: None)
        """
        self.tolerance = tolerance  # 유효범위 (거리 임계값)

        # 초기 배열을 받았을 경우 해당 배열을 사용하고, 없으면 빈 배열로 초기화
        if initial_points is None:
            self.utm_points = []  # 저장된 UTM 좌표 리스트 (각 좌표는 (x, y, easting, northing) 튜플 형태)
        else:
            self.utm_points = initial_points  # 초기화 시 받은 배열로 설정

    def distance(self, point1, point2):
        # 두 점 사이의 거리 계산
        return math.sqrt((point1[2] - point2[2]) ** 2 + (point1[3] - point2[3]) ** 2)

    def add_point(self, new_point):
        # 이미 저장된 점들과 비교하여 유효범위 내의 점이 있는지 확인
        for point in self.utm_points:
            if self.distance(point, new_point) < self.tolerance:
                # 유효범위 내에 기존 점이 있으면 새로 추가하지 않고 기존 점 유지
                return

        # 유효범위를 벗어나면 새 점을 추가
        self.utm_points.append(new_point)

    def process_lidar_point(self, lidar_point):
        # 라이다 좌표를 UTM으로 변환하고 점을 추가
        utm_x = lidar_point[2]
        utm_y = lidar_point[3]
        self.add_point(lidar_point)


''' 노이즈 줄이는 방법 추가하려고 했는데 일단 시간이 없어서 거리 기반으로 추가하는걸로 진행
class LidarProcessor:
    def __init__(self, tolerance, min_nearby_count=3, time_window=5, initial_points=None):
        """
        LidarProcessor 클래스 초기화
        :param tolerance: 유효범위 (거리 임계값)
        :param min_nearby_count: 점을 추가하기 위해 주변에 필요한 최소 점 개수
        :param time_window: 임시 배열에 점을 유지할 시간 (초)
        :param initial_points: 초기 UTM 좌표 리스트 (기본값: None)
        """
        self.tolerance = tolerance  # 유효범위 (거리 임계값)
        self.min_nearby_count = min_nearby_count  # 근처 점이 몇 번 찍혀야 추가할지
        self.time_window = time_window  # 임시 배열에 점을 유지할 시간 (초)
        if initial_points is None:
            self.utm_points = []  # 저장된 UTM 좌표 리스트 (각 좌표는 (x, y, easting, northing) 튜플 형태)
        else:
            self.utm_points = initial_points  # 초기화 시 받은 배열로 설정
        self.temp_points = []  # 임시로 점을 저장할 리스트, (x, y, timestamp)

    def distance(self, point1, point2):
        # 두 점 사이의 거리 계산
        return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    def add_to_temp(self, new_point):
        """
        새로운 점을 임시 배열에 추가.
        :param new_point: 추가할 새 점 (x, y 좌표)
        """
        current_time = time.time()
        self.temp_points.append((new_point[0], new_point[1], current_time))

        # 임시 배열에서 오래된 점을 제거 (time_window 초 이내의 점만 유지)
        self.temp_points = [(x, y, t) for (x, y, t) in self.temp_points if current_time - t <= self.time_window]

    def count_nearby_points(self, new_point):
        """
        임시 배열에서 주어진 새 점 근처에 있는 점들의 개수를 카운트
        :param new_point: 새로 들어온 점 (x, y 좌표)
        :return: 근처에 찍힌 점들의 개수
        """
        nearby_count = 0
        for point in self.temp_points:
            if self.distance((point[0], point[1]), new_point) < self.tolerance:
                nearby_count += 1
        return nearby_count

    def add_point(self, new_point):
        """
        근처에 찍힌 점의 개수가 일정 숫자 이상일 때만 utm_points에 추가
        """
        if self.count_nearby_points(new_point) >= self.min_nearby_count:
            # 조건을 만족할 때만 새 점을 추가
            self.utm_points.append(new_point)

    def process_lidar_point(self, lidar_point):
        """
        라이다로부터 점이 들어오면 임시 배열에 저장 후 근처에 찍힌 횟수를 체크.
        일정 횟수 이상일 때 utm_points에 추가
        """
        utm_x = lidar_point[2]
        utm_y = lidar_point[3]
        new_point = (utm_x, utm_y)

        # 새 점을 임시 배열에 추가
        self.add_to_temp(new_point)

        # 일정 숫자 이상 근처에 찍힌 점만 utm_points에 추가
        self.add_point(new_point)

    def get_utm_points(self):
        """
        현재까지 추가된 UTM 좌표 리스트를 반환
        """
        return self.utm_points
'''


class DELIVERY:
    def __init__(self):
        rospy.loginfo("GPS2UTM is Created")
        #반사율 없는 토픽
        # rospy.Subscriber("/lidar/tracking_objects", TrackingObjectArray, self.lidar_callback)

        #반사율 이용할때 사용
        
        rospy.Subscriber("/tracking_filter_node/filtered_tracking_object", TrackingObjectArray, self.lidar_callback)
        
        rospy.Subscriber("odom_gps", Odometry, self.odom_callback)
        rospy.Subscriber("/traffic_sign",Int32,self.vision_callback)
        rospy.Subscriber("/vel",TwistStamped,self.velocity_callback)
        
        self.marker_pub = rospy.Publisher('markers', MarkerArray, queue_size=1)  # 라이다 맵에서 모든 표지판위치 마커

        self.marker1_pub = rospy.Publisher('utm_marker', Marker, queue_size=1)  # 23년에 디버깅용으로 사용하던 토픽인듯?
        self.delivery_utm_pub=rospy.Publisher('utm_sign', Float32MultiArray, queue_size=1)      #23년 배달 목적지 발행하던것

        self.delivery_stop_pub=rospy.Publisher('/observed_sign', Bool, queue_size=1)    #배달미션 시작 트리거, 멀티마스터

        
        rospy.Subscriber("/State", String ,self.state_callback)
        rospy.Subscriber("/vehicle_yaw", Float32, self.vehicle_yaw_callback)

        
        #최종 배달 표지판 좌표 발행, 멀티마스터
        self.delivery_pickup_utm_pub=rospy.Publisher('/pickup_utm_sign', Float32MultiArray, queue_size=1)   #최종 배달 UTM좌표
        self.delivery_pickup_marker_utm_pub=rospy.Publisher('/pickup_marker_utm_sign', Marker, queue_size=1)        #최종 배달 마커 UTM발행용
        self.delivery_pickup_marker_pub=rospy.Publisher('/pickup_marker_sign', Marker, queue_size=1)        #최종 배달 마커 라이다발행용
        self.delivery_dest_utm_pub=rospy.Publisher('/dest_sign_local_coord', Float32MultiArray, queue_size=1)   #최종 배달 UTM좌표

        self.delivery_sign_marker_pub = rospy.Publisher('/sign_markers', MarkerArray, queue_size=1)  # 표지판 전체 발행(UTM)
        
        #----------------변수 초기화---------------------
        self.temp1=self.temp2= self.tempa=self.tempb=0
        self.wheelbase=0.25
        self.is_gps=False
        self.current_time=0
        self.UTM= None
        #----------------------------------------------       
        self.vehicle_yaw=0.
        self.vel=float("inf")
        self.is_odom=False
        self.delivery_vison_flag=False

        #----------------------------------------------중요!!!!-----------------------------------------
        # 하나씩 테스트 할때는 delivery_lidar_flag 값 지정해가면서 하면 되는데, 실 주행시에는 무조건 False 해야함!!
        self.delivery_lidar_flag=True      #픽업장소 False, 배달장소 True
        self.Delivery_target_idx = 1 # A에 따른 목표 B number   실 주행시에는 None해서 콜백 받은 값을 사용해야함
        self.State = "Pick_up" # "Pick_up", "Delivery"


        self.is_yaw = False
        self.is_state = False

        self.current_position=Point()
        self.marker_array = MarkerArray()
        self.permonent_singn_utm = []

        self.delivery_sign_marker_array = MarkerArray()     #인식된 표지판 표시용 마커
        self.Delivery_End=False
        self.utm_end = False
        self.vision_num=None        #실 주행시에는 주석처리하기. 테스트용
        self.rospack = rospkg.RosPack()




    
    def odom_callback(self,delivery_zone_arrived):    
        self.is_odom=True
        self.current_position.x=delivery_zone_arrived.pose.pose.position.x
        self.current_position.y=delivery_zone_arrived.pose.pose.position.y
        self.vehicle_yaw=delivery_zone_arrived.pose.pose.position.z
    

    def vision_callback(self,vision_sign):

        #### Delivery: topic from vision ####
        # A1:11, A2:12, A3:13 , idx = B1:0, B2:1, B3:2
        self.vision_num = vision_sign.data

        # self.vision_num =12 #테스트용 코드
        if self.vision_num==11 or self.vision_num==12 or self.vision_num==13 and self.delivery_vison_flag==False:
            self.Delivery_target_idx = {11:0, 12:1, 13:2}.get(self.vision_num,None)
            print("=================")
            print("=================")
            print("=================")
            print("=================")
            if self.Delivery_target_idx is not None: #변수이름 바꿨는데 사용하는곳은 따로 없음
                self.delivery_vison_flag=True
    

    def velocity_callback(self, vel):
        self.is_vel=True
        self.vel=np.sqrt((vel.twist.linear.x)**2+(vel.twist.linear.y)**2)
        if np.isnan(self.vel):
            self.vel=0
        # print("vel")
        # print("self.vel",self.vel)


    
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
        # print('static_trigger : ', self.static_trigger)
    


    def lidar_callback(self, _data):   
        total_obj_cnt = _data.size
        delivery_signs=[]
        
        # print("=================")
        # print("total_cnt")
        # print(total_obj_cnt)
        # print("=================")
        # print("rate")
        # print(1/(time.time()-self.current_time))

        delivery_sign_cnt=0
        self.current_time=time.time()
        #print("---------------------------------")
        #print("Delivery_target_idx",self.Delivery_target_idx)
        for obj in _data.array:
            if len(obj.bev.data) < 8:       #데이터 유효성 판단
                rospy.logwarn("Invalid bounding box format: %s", obj.bev.data)
                continue
            bbox_center_x, bbox_center_y = self.calculate_bounding_box_center(obj.bev.data)     #인식된 객체의 중심점 계산
            bbox_width, bbox_length = self.calculate_bounding_box_dimensions(obj.bev.data)      #인식된 객체 폭, 높이 계산

            self.delta_x = bbox_center_x + self.wheelbase   #UTM 변환 위한 라이다-GPS 거리 보정
            self.delta_y = bbox_center_y
            end_utm_easting, end_utm_northing = self.calculate_longitude_latitude() #UTM 좌표 계산
         
            x=bbox_center_x     #라이다기준 물체 위치
            y=bbox_center_y

            if self.State == "Pick_up": 
                #위치, 크기가 조건에 맞을경우 표지판 배열에 추가 

                # 반사율 사용 안할때 
                # if(0<x<15 and -7<y<0 and 0.5>bbox_length>0.1 and 0.9>bbox_width>0.3):   #도착지점 최적화
                # if(0<x<15 and -7<y<0 and 1>bbox_length>0.05 and 0.9>bbox_width>0.3):   #도착지점 최적화
                if(0<x<15 and -7<y<0 and 0.3>bbox_length>0.1 and 0.8>bbox_width>0.4):   #출발지점 최적화

                    # print("x,y",x,y,"W",bbox_width,"H",bbox_length, "height", )
                    #print("obs_x: {:.6f}, obs_y: {:.6f}".format(end_utm_easting,end_utm_northing))
                    delivery_signs.append((x,y,end_utm_easting,end_utm_northing))
                    delivery_sign_cnt+=1

                    #위에있는건 전역 맵에 찍어주는 코드, 아래있는건 라이다 맵에 찍어주는것 
                    #아래쪽으로 위치 옮김
                    self.add_marker_utm(_data.header,"map",end_utm_easting,end_utm_northing,delivery_sign_cnt)
                    self.add_marker(_data.header,"velodyne",x,y,delivery_sign_cnt)
                    


                    process_after_sign_pickup = LidarProcessor(3, self.permonent_singn_utm) #인스턴스 생성

                    for sign_points in delivery_signs:  #delivery_signs에 저장되어있는 UTM 좌표 가져오기위함.
                        process_after_sign_pickup.process_lidar_point(sign_points)

                    self.permonent_singn_utm = process_after_sign_pickup.utm_points #연산끝난후 배열 가져옴
                    # print("Saved Coord:", self.permonent_singn_utm)

                    if self.permonent_singn_utm:    #출발지점 좌표가 1개라도 들어왔을 경우  좌표 발행
                        utm_array_delivery_pickup_zone_arrived = Float32MultiArray()

                        # delivery_signs_UTM 배열에서 UTM 좌표들만 추출하여 data에 추가
                        utm_array_delivery_pickup_zone_arrived.data.append(self.permonent_singn_utm[0][0])  # easting
                        utm_array_delivery_pickup_zone_arrived.data.append(self.permonent_singn_utm[0][1])  # northing

                        # 추가한 데이터로 메시지 발행
                        self.delivery_pickup_utm_pub.publish(utm_array_delivery_pickup_zone_arrived)   #1번만 발행되는 상황임
                        

                        self.pub_delivery_pickup_marker(_data.header,"velodyne",self.permonent_singn_utm[0][0],
                                                            self.permonent_singn_utm[0][1], 300)
                        self.pub_delivery_pickup_marker_utm(_data.header,"map",self.permonent_singn_utm[0][2],
                                                            self.permonent_singn_utm[0][3], 301)
                        
                        # for sign in self.permonent_singn_utm:
                        #     print("marker_coordinate", sign[0], sign[1])
                        # print('lets change state')


                        #실 주행시에는 제어에서 받아야해서 주석처리해야함
                        self.permonent_singn_utm = []
                        self.State = "Delivery" # "Pick_up", "Delivery"

                        

            elif self.State == "Delivery":
                #위치, 크기가 조건에 맞을경우 표지판 배열에 추가
                if(0<x<15 and -7<y<0 and 0.5>bbox_length>0.1 and 0.8>bbox_width>0.4):   
                    # print("x,y",x,y,"W",bbox_width,"H",bbox_length)
                    #print("obs_x: {:.6f}, obs_y: {:.6f}".format(end_utm_easting,end_utm_northing))
                    delivery_signs.append((x,y,end_utm_easting,end_utm_northing))
                    delivery_sign_cnt+=1

                    #위에있는건 전역 맵에 찍어주는 코드, 아래있는건 라이다 맵에 찍어주는것 
                    #아래쪽으로 위치 옮김
                    self.add_marker_utm(_data.header,"map",end_utm_easting,end_utm_northing,delivery_sign_cnt)
                    self.add_marker(_data.header,"velodyne",x,y,delivery_sign_cnt)

                    
                    process_after_sign_dest = LidarProcessor(3, self.permonent_singn_utm)    #
                    for sign_points in delivery_signs:
                        process_after_sign_dest.process_lidar_point(sign_points)

                    self.permonent_singn_utm = process_after_sign_dest.utm_points #연산끝난후 배열 가져옴
                    # print("Saved Coord:", self.permonent_singn_utm)

                    if self.permonent_singn_utm:    #도착지점 표지판 1개라도 들어왔을 경우 플래너쪽으로 도착지점 시나리오 전환 가능하도록 발행
                        utm_array_delivery_dest_zone_arrived = Float32MultiArray()
                        delivery_zone_arrived = True
                        self.delivery_stop_pub.publish(delivery_zone_arrived)
                        

                        #비전에서 얻은 순서의 배열에 값이 들어가 있으면 해당 좌표 추가후 발행.
                        if len(self.permonent_singn_utm) > self.Delivery_target_idx:
                            utm_array_delivery_dest_zone_arrived.data.append(self.permonent_singn_utm[self.Delivery_target_idx][2])  # easting
                            utm_array_delivery_dest_zone_arrived.data.append(self.permonent_singn_utm[self.Delivery_target_idx][3])  # northing
                            
                            # 추가한 데이터로 메시지 발행
                            self.delivery_dest_utm_pub.publish(utm_array_delivery_dest_zone_arrived)   #1번만 발행되는 상황임

                            #배달 목적지 마커로 출력(라이다, 전역맵)
                            self.pub_delivery_pickup_marker(_data.header,"velodyne",self.permonent_singn_utm[self.Delivery_target_idx][0],
                                                            self.permonent_singn_utm[self.Delivery_target_idx][1], 300)
                            self.pub_delivery_pickup_marker_utm(_data.header,"map",self.permonent_singn_utm[self.Delivery_target_idx][2],
                                                            self.permonent_singn_utm[self.Delivery_target_idx][3], 301)
                        else:
                            pass
                    
                    # 마커 발행용.
                    # for sign in self.permonent_singn_utm:
                    #     #위에있는건 라이다 맵에 찍어주는것 아래있는건 전역 맵에 찍어주는 코드
                        # print("marker_coordinate", sign[0], sign[1])
                    #     self.add_marker(_data.header,"velodyne",sign[0], sign[1], delivery_sign_cnt)
                    #     self.add_marker_utm(_data.header,"map",sign[2], sign[3] ,delivery_sign_cnt+50)
                    #     pass

                    else:   #이부분은 추가해야하는지 약간 의문
                        delivery_zone_arrived = False

                


                

        #print("**************")
        #print("cnt",delivery_sign_cnt)
        #print("**************")

        # 배달 도착지점 플래너에 알려주는 변수
        delivery_zone_arrived=Bool()  #이전에 msg였음


     
        print('State:', self.State, ', obj count:', len(self.permonent_singn_utm), ', time:', time.time())
            

        #print("self.UTM",self.UTM)
    
        if self.UTM is not None :
            target_utm_delivery_zone_arrived = Float32MultiArray()
            target_utm_delivery_zone_arrived.data = self.UTM  # You can put any other 32-bit float numbers here   
            self.delivery_utm_pub.publish(target_utm_delivery_zone_arrived) 

            # marker1 = Marker()
            # marker1.header = _data.header
            # marker1.header.frame_id = "map"
            # marker1.type = Marker.SPHERE  # You can choose the marker type you prefer
            # marker1.action = Marker.ADD
            # marker1.pose.position.x = self.UTM[0]
            # marker1.pose.position.y = self.UTM[1]
            # marker1.pose.position.z = 1.0
            # marker1.scale.x = 1.0  # Adjust the marker size as needed
            # marker1.scale.y = 1.0
            # marker1.scale.z = 1.0
            # marker1.color.a = 1.0  # Alpha (transparency)
            # marker1.color.r = 1.0  # Red color
            # marker1.color.g = 0.0  # Green color
            # marker1.color.b = 0.0  # Blue color
            # marker1.id=100
            # self.marker1_pub.publish(marker1)

        # custom_delivery_zone_arrived/PointArray_delivery_zone_arrived.delivery_zone_arrived 메시지 생성
        self.marker_pub.publish(self.marker_array)
        self.delivery_sign_marker_pub.publish(self.delivery_sign_marker_array)
        pass

    def pub_delivery_pickup_marker(self, header, frame_id, x, y, id):
        marker1 = Marker()
        marker1.header = copy.deepcopy(header)  # 헤더 깊은 복사
        marker1.header.frame_id = frame_id
        marker1.type = Marker.SPHERE  # You can choose the marker type you prefer
        marker1.action = Marker.ADD
        marker1.ns = "marker set1"
        marker1.id=id
        marker1.pose.position.x = x
        marker1.pose.position.y = y
        marker1.pose.position.z = 1.0
        marker1.scale.x = 1.0  # Adjust the marker size as needed
        marker1.scale.y = 1.0
        marker1.scale.z = 1.0
        marker1.color.a = 1.0  # Alpha (transparency)
        marker1.color.r = 1.0  # Red color
        marker1.color.g = 0.0  # Green color
        marker1.color.b = 0.0  # Blue color
        marker1.lifetime = rospy.Duration(1.0)
        self.delivery_pickup_marker_pub.publish(marker1)

    def pub_delivery_pickup_marker_utm(self, header, frame_id, x, y, id):
        marker1 = Marker()
        marker1.header = copy.deepcopy(header)  # 헤더 깊은 복사
        marker1.header.frame_id = frame_id
        marker1.type = Marker.SPHERE  # You can choose the marker type you prefer
        marker1.action = Marker.ADD
        marker1.ns = "marker set1"
        marker1.id=id
        marker1.pose.position.x = x
        marker1.pose.position.y = y
        marker1.pose.position.z = 1.0
        marker1.scale.x = 1.0  # Adjust the marker size as needed
        marker1.scale.y = 1.0
        marker1.scale.z = 1.0
        marker1.color.a = 1.0  # Alpha (transparency)
        marker1.color.r = 1.0  # Red color
        marker1.color.g = 0.0  # Green color
        marker1.color.b = 0.0  # Blue color
        marker1.lifetime = rospy.Duration(1.0)
        self.delivery_pickup_marker_utm_pub.publish(marker1)

    def add_marker(self, header, frame_id, x, y, id):
        marker1 = Marker()
        marker1.header = copy.deepcopy(header)  # 헤더 깊은 복사
        marker1.header.frame_id = frame_id
        marker1.type = Marker.SPHERE  # You can choose the marker type you prefer
        marker1.action = Marker.ADD
        marker1.ns = "marker set1"
        marker1.id=id
        marker1.pose.position.x = x
        marker1.pose.position.y = y
        marker1.pose.position.z = 1.0
        marker1.scale.x = 1.0  # Adjust the marker size as needed
        marker1.scale.y = 1.0
        marker1.scale.z = 1.0
        marker1.color.a = 1.0  # Alpha (transparency)
        marker1.color.r = 0.0  # Red color
        marker1.color.g = 0.0  # Green color
        marker1.color.b = 1.0  # Blue color
        marker1.lifetime = rospy.Duration(1.0)
        self.marker_array.markers.append(marker1)

        
    def add_marker_utm(self, header, frame_id, x, y, id):
        marker1 = Marker()
        marker1.header = header
        marker1.header.frame_id = frame_id
        marker1.type = Marker.SPHERE  # You can choose the marker type you prefer
        marker1.action = Marker.ADD
        marker1.ns = "marker set2"
        marker1.id=id
        marker1.pose.position.x = x
        marker1.pose.position.y = y
        marker1.pose.position.z = 1.0
        marker1.scale.x = 1.0  # Adjust the marker size as needed
        marker1.scale.y = 1.0
        marker1.scale.z = 1.0
        marker1.color.a = 1.0  # Alpha (transparency)
        marker1.color.r = 0.0  # Red color
        marker1.color.g = 0.0  # Green color
        marker1.color.b = 1.0  # Blue color
        self.delivery_sign_marker_array.markers.append(marker1)




    def cal_obs_data(self,delta_x,delta_y):
        x=delta_x            
        y=delta_y
        obs_angle = np.rad2deg(math.atan2(y,x))
        obs_dist = np.sqrt(x**2+y**2)

        return obs_angle, obs_dist
        
    def calculate_longitude_latitude(self):
        
        # 시작 위치의 UTM 좌표 구하기
        start_utm_easting, start_utm_northing = self.current_position.x, self.current_position.y
        heading_rad = self.vehicle_yaw

        delta_utm_easting = self.delta_x * math.cos(heading_rad) - self.delta_y * math.sin(heading_rad)
        delta_utm_northing = self.delta_x * math.sin(heading_rad) + self.delta_y * math.cos(heading_rad)

        # 시작 위치에 UTM 좌표 변화량을 더하여 최종 UTM 좌표 구하기
        end_utm_easting = start_utm_easting + delta_utm_easting
        end_utm_northing = start_utm_northing + delta_utm_northing

        return end_utm_easting, end_utm_northing
    


    #------------------heading각 계산----------------------------
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

    def calculate_bounding_box_center(self, bev_coords):
        
        center_x = (bev_coords[0] + bev_coords[2] + bev_coords[4] + bev_coords[6]) / 4
        center_y = (bev_coords[1] + bev_coords[3] + bev_coords[5] + bev_coords[7]) / 4
        return center_x, center_y

    def calculate_bounding_box_dimensions(self, bev_coords):
        width = math.sqrt((bev_coords[2] - bev_coords[0]) ** 2 + (bev_coords[3] - bev_coords[1]) ** 2)
        length = math.sqrt((bev_coords[4] - bev_coords[2]) ** 2 + (bev_coords[5] - bev_coords[3]) ** 2)
        return width, length

    def calculate_angle_with_vehicle(self, center_x, center_y, vehicle_x, vehicle_y):
        angle_rad = math.atan2(center_y - vehicle_y, center_x - vehicle_x)
        angle_deg = math.degrees(angle_rad)
        return angle_deg

    def calculate_distance_to_vehicle(self, center_x, center_y, vehicle_x, vehicle_y):
        distance = math.sqrt((center_x - vehicle_x) ** 2 + (center_y - vehicle_y) ** 2)
        return distance


def run():
    rospy.init_node("Delivery")
    new_classs= DELIVERY()
    rospy.spin()
    

if __name__ == '__main__':
    run()