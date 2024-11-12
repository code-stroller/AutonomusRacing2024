#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
# import tf
import os
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from tracking_msg.msg import TrackingObjectArray
from visualization_msgs.msg import Marker, MarkerArray
# from scipy.spatial import distance
from math import pi,sqrt
import math
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseArray, Pose
from custom_msg.msg import PointArray_msg # geometry_msgs/Point[] array -> custom msg 생성 
import numpy as np
import time
from std_msgs.msg import Int32, Bool,String

##############
#### 예선 ####
##############

class Uturn:
    def __init__(self):
        rospy.loginfo("Uturn is Created")

        # ------------------------- Subscriber ----------------------
        rospy.Subscriber("/lidar/tracking_objects", TrackingObjectArray, self.lidar_callback)
        # rospy.Subscriber("/State",String,self.state_callback)
        # -------------------------- Marker ----------------------
        self.middle_point_pub = rospy.Publisher("middle_point", Marker, queue_size=10)
        self.lane_point_pub = rospy.Publisher("lane_point", Marker, queue_size=10)
        # ------------------------- Publisher ----------------------------
        self.target_point_publisher = rospy.Publisher("uturn_point", Float32MultiArray, queue_size=1)

        # U-turn 
        self.ROI_1st_cnt=0
        self.State=None
        self.a = False
        self.State="Rubber_cone_drive"

    # def state_callback(self,state):
    #     self.State=state.data
    #     self.State="Rubber_cone_drive"

    def lidar_callback(self, _data):
        
        total_obj_cnt = _data.size                          
        
        self.current_time=time.time()
        
        bev_msg = PoseArray()
        bev_msg.header = _data.header
        
        obj = _data.array
        
        close_uturn_cone=[]
        far_uturn_cone=[]

        compare_threshold = 4
     
        # ------------------- processing roi data --------------------
        for i, obj in enumerate(obj):

            if len(obj.bev.data) < 8:
                rospy.logwarn("Invalid bounding box format: %s", obj.bev.data)
                continue

            bbox_center_x, bbox_center_y = self.calculate_bounding_box_center(obj.bev.data)
            bbox_width, bbox_height = self.calculate_bounding_box_dimensions(obj.bev.data)
            _,d = self.cal_obs_data(bbox_center_x,bbox_center_y)

            ##################### U-turn ##################
            # print(self.State)
            if(self.State=="Rubber_cone_drive"):
                if 0.2<bbox_center_x<compare_threshold and 0<bbox_center_y<6 and 0.03<bbox_width<0.25 and 0.1<bbox_height<0.4:  # RIGHT TURN  
                    close_uturn_cone.append((d,bbox_center_x,bbox_center_y))
                    print(f"{i}st close_cone detected")

                elif compare_threshold<bbox_center_x<10 and 0<bbox_center_y<6 and 0.03<bbox_width<0.25 and 0.1<bbox_height<0.4: 
                    if(0.03<bbox_width<0.25 and 0.1<bbox_height<0.5):
                        print(f"{i}st far_cone detected")
                        far_uturn_cone.append((d,bbox_center_x,bbox_center_y))


        close_uturn_cone.sort(key=lambda x: x[0])
        rospy.logwarn(f'close cone : {len(close_uturn_cone)}')
        rospy.logwarn(f'far cone : {len(far_uturn_cone)}')
        mid_point=None

        FRONT_OFFSET = 2
        SIDE_OFFSET = 5.5

        if(self.State=="Rubber_cone_drive"):
            
            if len(close_uturn_cone) <= 1 and len(far_uturn_cone) >= 2:
                lane_point = None
                mid_point = (0,0)
                rospy.logwarn(f'aaaaaaaaaaaaaaaa')
            elif len(close_uturn_cone) >= 2 or len(close_uturn_cone) >= 2:
                lane_point = (close_uturn_cone[0][1], close_uturn_cone[0][2])
                mid_point = (lane_point[0], lane_point[1] - SIDE_OFFSET)
                rospy.logwarn(f'bbbbbbbbbbbbbb')
            elif len(close_uturn_cone) >= 2 and len(far_uturn_cone) <= 0:
                lane_point = (close_uturn_cone[0][1], close_uturn_cone[0][2])
                mid_point = (0,0)
                rospy.logwarn(f'dddddddddddddd')
            else:
                lane_point = None
                mid_point = (0,0)
                rospy.logwarn(f'eeeeeeeeeeeeeeeee')



            print("=================")
            print("mid_point",mid_point)
            print("===================")

            
            if(mid_point is not None):
                target_point=Float32MultiArray()
                target_point.data.append(mid_point[0])
                target_point.data.append(mid_point[1])
                self.target_point_publisher.publish(target_point)
                self.publish_obstacles(mid_point, self.middle_point_pub, color=(0.0, 1.0, 0.0))  # 초록색으로 시각화
                self.publish_obstacles(lane_point, self.lane_point_pub, color=(1.0, 0.0, 0.0))  # 초록색으로 시각화

        else:
            print("Not in rubber cone")



    ############################################################################################

    def publish_obstacles_to_array(self,point, marker_array, color=(0.0, 0.0, 1.0), marker_id=0):
        marker = Marker()
        marker.header.frame_id = "velodyne"  # 프레임 ID 설정
        marker.type = marker.CUBE  # 마커 형태
        marker.action = marker.ADD
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0
        marker.scale.x = 0.8
        marker.scale.y = 0.8
        marker.scale.z = 0.8
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.id = marker_id  # 마커 ID 설정
        
        marker_array.markers.append(marker)  # MarkerArray에 마커 추가
    
    def cal_obs_data(self,delta_x,delta_y):
        x=delta_x
        y=delta_y
        obs_angle = np.rad2deg(math.atan2(y,x))
        obs_dist = np.sqrt(x**2+y**2)
        
        return obs_angle, obs_dist
    
    def publish_obstacles(self, obs, publisher, color):
        if obs is not None:
            x, y = obs[0],obs[1]
            # Marker 메시지를 생성하여 장애물들을 크고 입체적으로 시각화
            marker = Marker()
            marker.header.frame_id = "velodyne"  # 필요에 따라 적절한 프레임으로 변경
            marker.header.stamp = rospy.Time.now()
            marker.ns = "obstacles"
            marker.id = 0
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

            publisher.publish(marker)


    def calculate_bounding_box_center(self, bev_coords):
        center_x = (bev_coords[0] + bev_coords[2] + bev_coords[4] + bev_coords[6]) / 4
        center_y = (bev_coords[1] + bev_coords[3] + bev_coords[5] + bev_coords[7]) / 4
        return center_x, center_y

    def calculate_bounding_box_dimensions(self, bev_coords):
        width = math.sqrt((bev_coords[2] - bev_coords[0]) ** 2 + (bev_coords[3] - bev_coords[1]) ** 2)
        height = math.sqrt((bev_coords[4] - bev_coords[2]) ** 2 + (bev_coords[5] - bev_coords[3]) ** 2)
        return width, height


def run():
    rospy.init_node("uturn")
    new_classs= Uturn()
    rospy.spin()
    

if __name__ == '__main__':
    run()
