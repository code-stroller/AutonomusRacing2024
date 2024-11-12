#!/usr/bin/env python
# -*- coding: utf-8 -*-

from asyncio import set_event_loop
from re import I
import rospy
from erp_driver.msg import erpCmdMsg, erpStatusMsg
from geometry_msgs.msg import Point, TwistStamped
from nav_msgs.msg import Odometry, Path
# from morai_msgs.msg import CtrlCmd,EgoVehicleStatus
import numpy as np
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from std_msgs.msg import Float32, String, Float32MultiArray, Int32
from geometry_msgs.msg import Point32, PoseStamped
import time
from visualization_msgs.msg import Marker

AUTO_MODE = 1
MANUAL_MODE = 0

class LabaconDrive:
    def __init__(self):
        rospy.init_node('labacon_drive', anonymous=True)
        # =============================== Subscriber =============================== #
        rospy.Subscriber("/erp42_status", erpStatusMsg, self.status_callback)
        rospy.Subscriber('/mid_point', Float32MultiArray, self.labacon_callback)
        # ================================== Publisher =============================== #
        self.erp_42_ctrl_pub = rospy.Publisher("/erp42_ctrl_cmd", erpCmdMsg, queue_size=1)
        # =====================================
        self.erpCmd_msg = erpCmdMsg()
        self.erpStatus_msg = erpStatusMsg()
        # ====================================
        self.is_laba = False
        self.is_status = False
        self.mid_points = []
        self.prev_mid_point_x = 0
        self.prev_mid_point_y = 0
        
        frameRate = 100
        rate = rospy.Rate(frameRate)

        self.pid_laba = pidControl_Laba()
        self.pid = pidControl()

        while not rospy.is_shutdown():
            if not self.is_status:
                rospy.loginfo("ERP Status msg do not arrived....")

            elif not self.is_laba:
                rospy.loginfo("Labacone midpoint msg do not arrived....")
                

            else:
                if self.erpStatus_msg.control_mode == AUTO_MODE:
                    self.main()
                else:
                    self.pid_laba.init_variable()
                    self.pid.init_variable()
            rate.sleep()

    def main(self):
        curvature = self.calculate_curvature(self.mid_points)

        rospy.loginfo(f"Calculated curvature: {curvature:.5f}")

        if curvature > 0.1: 
            self.pid_laba.set_p_gain(0.8)
        elif curvature > 0.05:
            self.pid_laba.set_p_gain(0.5)
        else:
            self.pid_laba.set_p_gain(0.2)

        steering = self.cal_laba_steer(self.mid_point_x, self.mid_point_y)
        throttle_command, goal_velocity = self.control_labacone_velocity(curvature)
        
        brake = self.calculate_brake(goal_velocity)
        
        self.erpCmd_msg.gear = 0
        self.erpCmd_msg.steer = int(steering)
        self.erpCmd_msg.speed = int(throttle_command)
        self.erpCmd_msg.brake = int(brake)

        # print(self.erpCmd_msg.gear)
        # print(self.erpCmd_msg.steer)
        # print(self.erpCmd_msg.speed)
        # print(self.erpCmd_msg.brake)

        print('Current_velocity : {}'.format(self.erpStatus_msg.speed))
        # print(f'steering : {steering}, throttle : {throttle_command}')
        print("Target_Velocity : {:.2f}, Target_steering : {:.2f}".format(goal_velocity, math.degrees(steering/2000*0.4922)))
        self.erp_42_ctrl_pub.publish(self.erpCmd_msg)

    # 수정 : 브레이크 조절 함수 ( 브레이크 범위가 1~200 이라 , 5 정도 주면 안멈추고 스무스하게 속도전환함)
    def calculate_brake(self, goal_velocity):
        if self.erpStatus_msg.speed > (goal_velocity + 2)*10:
            brake = 5
        else:
            brake = 0
        return brake

    def cal_laba_steer(self, labacone_point_x, labacone_point_y):
        error_x = labacone_point_x
        error_y = -labacone_point_y

        # 목표 지점까지의 각도를 계산
        theta = math.atan2(error_y, error_x)

        # 수정 : erp 단위로 수정
        target_steering_erp = ( theta / 0.4922 ) * 2000

        # PID 제어의 P 값을 사용하여 조향각 계산
        steering_output = self.pid_laba.pid_Laba(target_steering_erp)
        rospy.logwarn(f'thisssss  {int(max(min(steering_output, 2000), -2000))}')
        # 조향각 제한
        return int(max(min(steering_output, 2000), -2000))

    def control_labacone_velocity(self, curvature):
        target_velocity = 0

        if curvature > 0.1:
            target_velocity = 0
        elif curvature > 0.05:
            target_velocity = 0
        else:
            target_velocity = 0

        goal_velocity = target_velocity
        # PID 제어기를 사용해 목표 속도와 현재 속도의 차이를 조정
        throttle_command = round(self.pid.pid(target_velocity * 10, self.erpStatus_msg.speed))

        # 결과를 0에서 200 사이로 제한
        throttle_command = max(min(throttle_command, 200), 0)

        return throttle_command, goal_velocity

    def labacon_callback(self, msg):
        self.is_laba = True
        x_values = msg.data[0::2]  # x 값만 추출
        y_values = msg.data[1::2]  # y 값만 추출
        self.mid_points = [(x, y) for x, y in zip(x_values, y_values)]
        self.mid_point_x = x_values[0]
        self.mid_point_y = y_values[0]

        if math.isnan(self.mid_point_x):
            self.mid_point_x = self.prev_mid_point_x
        if math.isnan(self.mid_point_y):
            self.mid_point_y = self.prev_mid_point_y

        self.prev_mid_point_x = self.mid_point_x
        self.prev_mid_point_y = self.mid_point_y

    def status_callback(self, msg):
        self.is_status = True
        self.erpStatus_msg = msg

    def calculate_curvature(self, points):
        if len(points) < 2:
            return 0 

        x = [p[0] for p in points]
        y = [p[1] for p in points]

        coefficients = np.polyfit(x, y, 2)

        curvature = 2 * coefficients[0] / (1 + (2 * coefficients[0] * np.mean(x) + coefficients[1])**2)**1.5

        return np.abs(curvature)

# ============================================== PID =========================================

class pidControl_Laba:
    def __init__(self):
        self.p_gain = 0.75
        self.i_gain = 0
        self.d_gain = 0.005
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.1

    def pid_Laba(self, error):

        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error-self.prev_error) / self.controlTime

        output = p_control + self.i_control + d_control
        self.prev_error = error

        return output
    
    # 수정 : 메소드 추가
    def set_p_gain(self, p_gain):
        self.p_gain = p_gain

    def init_variable(self):
        self.prev_error = 0
        self.i_control = 0

class pidControl:
    def __init__(self):
        self.p_gain = 3.5
        self.i_gain = 0.3
        self.d_gain = 0.003
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.1
    
    def pid(self, target_vel, current_vel):
        
        error = target_vel - current_vel

        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error-self.prev_error) / self.controlTime

        output = p_control + self.i_control + d_control
        self.prev_error = error

        return output
    
    def init_variable(self):
        self.prev_error = 0
        self.i_control = 0

if __name__ == '__main__':
    try:
        LabaconDrive()
    except rospy.ROSInterruptException:
        pass
