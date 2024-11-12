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
    
    from math import pi,sqrt
    import math
    from geometry_msgs.msg import Quaternion
    from geometry_msgs.msg import PoseArray, Pose
    from custom_msg.msg import PointArray_msg # geometry_msgs/Point[] array -> custom msg 생성 
    import numpy as np
    import time
    from std_msgs.msg import Int32, Bool,String
    
    #############
    #### 예선 ####
    #############
    '''
    class Node:
        def __init__(self, x, y, theta):
            self.x = x
            self.y = y
            self.theta = theta  # 진행 방향 각도 (라디안)
            self.parent = None
            self.cost = 0.0
    
    
    class RRTStarLimitedSteering:
        def __init__(self, start, goal, obstacle_list, x_limits, y_limits, step_size=0.5, max_iter=2000,
                     goal_sample_rate=0.3, wall_distance=1.0, max_cone_distance=2.0):
            self.start = Node(start[0], start[1], 0.0)  # 초기 각도는 0도 (진행 방향)
            self.goal = Node(goal[0], goal[1], 0.0)
            self.obstacle_list = obstacle_list
            self.x_limits = x_limits
            self.y_limits = y_limits
            self.step_size = step_size
            self.max_iter = max_iter
            self.goal_sample_rate = goal_sample_rate
            self.wall_distance = wall_distance
            self.max_cone_distance = max_cone_distance
            self.node_list = [self.start]
    
        def plan(self):
            for _ in range(self.max_iter):
                rnd_node = self.get_random_node()
                nearest_node = self.get_nearest_node(self.node_list, rnd_node)
                new_node = self.steer(nearest_node, rnd_node)
    
                if self.check_collision(new_node, self.obstacle_list) and self.is_near_wall(new_node):
                    near_nodes = self.find_near_nodes(new_node)
                    new_node = self.choose_parent(new_node, near_nodes)
                    self.node_list.append(new_node)
                    self.rewire(new_node, near_nodes)
    
                if self.calc_dist_to_goal(new_node.x, new_node.y) <= self.step_size:
                    final_node = self.steer(new_node, self.goal)
                    if self.check_collision(final_node, self.obstacle_list) and self.is_near_wall(final_node):
                        return self.generate_final_course(len(self.node_list) - 1)
    
            return None
    
        def steer(self, from_node, to_node):
            new_node = Node(from_node.x, from_node.y, from_node.theta)
            dist, angle_to_target = self.calc_distance_and_angle(from_node, to_node)
    
            # 방향 제한: -20도, 0도, 20도만 허용
            angle_options = [-np.radians(180), 0, np.radians(180)]
            best_option = min(angle_options, key=lambda angle: abs(angle - (angle_to_target - from_node.theta)))
    
            # 선택된 각도로 방향 변경
            new_theta = from_node.theta + best_option
            new_node.theta = new_theta
    
            new_node.x += self.step_size * np.cos(new_theta)
            new_node.y += self.step_size * np.sin(new_theta)
            new_node.parent = from_node
            new_node.cost = from_node.cost + dist
    
            return new_node
    
        def choose_parent(self, new_node, near_nodes):
            if not near_nodes:
                return new_node
    
            costs = []
            for near_node in near_nodes:
                dist, _ = self.calc_distance_and_angle(near_node, new_node)
                if self.check_collision(new_node, self.obstacle_list):
                    costs.append(near_node.cost + dist)
                else:
                    costs.append(float('inf'))
    
            min_cost = min(costs)
            min_ind = near_nodes[costs.index(min_cost)]
            new_node.cost = min_cost
            new_node.parent = min_ind
    
            return new_node
    
        def rewire(self, new_node, near_nodes):
            for near_node in near_nodes:
                dist, _ = self.calc_distance_and_angle(new_node, near_node)
                cost = new_node.cost + dist
    
                if near_node.cost > cost and self.check_collision(near_node, self.obstacle_list):
                    near_node.parent = new_node
                    near_node.cost = cost
    
        def get_random_node(self):
            if np.random.rand() > self.goal_sample_rate:
                rnd = [np.random.uniform(self.x_limits[0], self.x_limits[1]),
                       np.random.uniform(self.y_limits[0], self.y_limits[1])]
            else:
                rnd = [self.goal.x, self.goal.y]
    
            theta = np.random.uniform(-np.pi, np.pi)
            return Node(rnd[0], rnd[1], theta)
    
        def get_nearest_node(self, node_list, rnd_node):
            dlist = [(node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y) ** 2 for node in node_list]
            min_ind = dlist.index(min(dlist))
            return node_list[min_ind]
    
        def find_near_nodes(self, new_node):
            n_nodes = len(self.node_list) + 1
            r = min(50.0 * (np.log(n_nodes) / n_nodes) ** 0.5, self.step_size * 5.0)
            dlist = [(node.x - new_node.x) ** 2 + (node.y - new_node.y) ** 2 for node in self.node_list]
            near_inds = [dlist.index(i) for i in dlist if i <= r ** 2]
    
            return [self.node_list[i] for i in near_inds]
    
        def check_collision(self, node, obstacle_list):
            for (ox, oy) in obstacle_list:
                dx = ox - node.x
                dy = oy - node.y
                if dx * dx + dy * dy <= 2 ** 2:
                    return False  # collision
    
            return True  # safe
    
        def is_near_wall(self, node):
            """왼쪽 장애물 벽에 가까운지 확인하는 함수"""
            for (ox, oy, size) in self.obstacle_list:
                distance_to_cone = np.hypot(node.x - ox, node.y - oy)
                if node.x > ox and distance_to_cone <= self.max_cone_distance and abs(
                        oy - node.y) <= size + self.wall_distance:
                    return True
            return False
    
        def calc_dist_to_goal(self, x, y):
            return np.hypot(x - self.goal.x, y - self.goal.y)
    
        def generate_final_course(self, goal_ind):
            path = [[self.goal.x, self.goal.y]]
            node = self.node_list[goal_ind]
            while node.parent is not None:
                path.append([node.x, node.y])
                node = node.parent
            path.append([self.start.x, self.start.y])
    
            return path
    
        @staticmethod
        def calc_distance_and_angle(from_node, to_node):
            dx = to_node.x - from_node.x
            dy = to_node.y - from_node.y
            dist = np.hypot(dx, dy)
            theta = np.arctan2(dy, dx)
            return dist, theta'''
    
    
    class GPS2UTM:
        def __init__(self):
            rospy.loginfo("Uturn is Created")
    
            # ------------------------- Subscriber ----------------------
            rospy.Subscriber("/lidar/tracking_objects", TrackingObjectArray, self.lidar_callback)
            rospy.Subscriber("/State",String,self.state_callback)
    
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
    
            # U-turn 
            self.State="Rubber_cone_drive"
            self.lfirst=False
            self.rfirst=False
            self.min_distance_threshold = 2
    
            self.ltime = time.time()
            self.rtime = time.time()
            # self.stable=False
            self.line_space = 10
    
        class Node:
            def __init__(self, x, y, theta):
                self.x = x
                self.y = y
                self.theta = theta  # 진행 방향 각도 (라디안)
                self.parent = None
                self.cost = 0.0
    
        class RRTStarLimitedSteering:
            def __init__(self, start, goal, obstacle_list, x_limits, y_limits, step_size=0.5, max_iter=2000,
                         goal_sample_rate=0.3, wall_distance=1.0, max_cone_distance=2.0):
                self.start = Node(start[0], start[1], 0.0)  # 초기 각도는 0도 (진행 방향)
                self.goal = Node(goal[0], goal[1], 0.0)
                self.obstacle_list = obstacle_list
                self.x_limits = x_limits
                self.y_limits = y_limits
                self.step_size = step_size
                self.max_iter = max_iter
                self.goal_sample_rate = goal_sample_rate
                self.wall_distance = wall_distance
                self.max_cone_distance = max_cone_distance
                self.node_list = [self.start]
    
            def plan(self):
                for _ in range(self.max_iter):
                    rnd_node = self.get_random_node()
                    nearest_node = self.get_nearest_node(self.node_list, rnd_node)
                    new_node = self.steer(nearest_node, rnd_node)
    
                    if self.check_collision(new_node, self.obstacle_list) and self.is_near_wall(new_node):
                        near_nodes = self.find_near_nodes(new_node)
                        new_node = self.choose_parent(new_node, near_nodes)
                        self.node_list.append(new_node)
                        self.rewire(new_node, near_nodes)
    
                    if self.calc_dist_to_goal(new_node.x, new_node.y) <= self.step_size:
                        final_node = self.steer(new_node, self.goal)
                        if self.check_collision(final_node, self.obstacle_list) and self.is_near_wall(final_node):
                            return self.generate_final_course(len(self.node_list) - 1)
    
                return None
    
            def steer(self, from_node, to_node):
                new_node = Node(from_node.x, from_node.y, from_node.theta)
                dist, angle_to_target = self.calc_distance_and_angle(from_node, to_node)
    
                # 방향 제한: -20도, 0도, 20도만 허용
                angle_options = [-np.radians(180), 0, np.radians(180)]
                best_option = min(angle_options, key=lambda angle: abs(angle - (angle_to_target - from_node.theta)))
    
                # 선택된 각도로 방향 변경
                new_theta = from_node.theta + best_option
                new_node.theta = new_theta
    
                new_node.x += self.step_size * np.cos(new_theta)
                new_node.y += self.step_size * np.sin(new_theta)
                new_node.parent = from_node
                new_node.cost = from_node.cost + dist
    
                return new_node
    
            def choose_parent(self, new_node, near_nodes):
                if not near_nodes:
                    return new_node
    
                costs = []
                for near_node in near_nodes:
                    dist, _ = self.calc_distance_and_angle(near_node, new_node)
                    if self.check_collision(new_node, self.obstacle_list):
                        costs.append(near_node.cost + dist)
                    else:
                        costs.append(float('inf'))
    
                min_cost = min(costs)
                min_ind = near_nodes[costs.index(min_cost)]
                new_node.cost = min_cost
                new_node.parent = min_ind
    
                return new_node
    
            def rewire(self, new_node, near_nodes):
                for near_node in near_nodes:
                    dist, _ = self.calc_distance_and_angle(new_node, near_node)
                    cost = new_node.cost + dist
    
                    if near_node.cost > cost and self.check_collision(near_node, self.obstacle_list):
                        near_node.parent = new_node
                        near_node.cost = cost
    
            def get_random_node(self):
                if np.random.rand() > self.goal_sample_rate:
                    rnd = [np.random.uniform(self.x_limits[0], self.x_limits[1]),
                           np.random.uniform(self.y_limits[0], self.y_limits[1])]
                else:
                    rnd = [self.goal.x, self.goal.y]
    
                theta = np.random.uniform(-np.pi, np.pi)
                return Node(rnd[0], rnd[1], theta)
    
            def get_nearest_node(self, node_list, rnd_node):
                dlist = [(node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y) ** 2 for node in node_list]
                min_ind = dlist.index(min(dlist))
                return node_list[min_ind]
    
            def find_near_nodes(self, new_node):
                n_nodes = len(self.node_list) + 1
                r = min(50.0 * (np.log(n_nodes) / n_nodes) ** 0.5, self.step_size * 5.0)
                dlist = [(node.x - new_node.x) ** 2 + (node.y - new_node.y) ** 2 for node in self.node_list]
                near_inds = [dlist.index(i) for i in dlist if i <= r ** 2]
    
                return [self.node_list[i] for i in near_inds]
    
            def check_collision(self, node, obstacle_list):
                for (ox, oy) in obstacle_list:
                    dx = ox - node.x
                    dy = oy - node.y
                    if dx * dx + dy * dy <= 2 ** 2:
                        return False  # collision
    
                return True  # safe
    
            def is_near_wall(self, node):
                """왼쪽 장애물 벽에 가까운지 확인하는 함수"""
                for (ox, oy) in self.obstacle_list:
                    distance_to_cone = np.hypot(node.x - ox, node.y - oy)
                    if node.x > ox and distance_to_cone <= self.max_cone_distance and abs(
                            oy - node.y) <= 2 + self.wall_distance:
                        return True
                return False
    
            def calc_dist_to_goal(self, x, y):
                return np.hypot(x - self.goal.x, y - self.goal.y)
    
            def generate_final_course(self, goal_ind):
                path = [[self.goal.x, self.goal.y]]
                node = self.node_list[goal_ind]
                while node.parent is not None:
                    path.append([node.x, node.y])
                    node = node.parent
                path.append([self.start.x, self.start.y])
    
                return path
    
            @staticmethod
            def calc_distance_and_angle(from_node, to_node):
                dx = to_node.x - from_node.x
                dy = to_node.y - from_node.y
                dist = np.hypot(dx, dy)
                theta = np.arctan2(dy, dx)
                return dist, theta
        def state_callback(self,state):
            #self.State=state.data
            self.State = "Rubber_cone_drive"
    
    
    
        def bezier_curve(self, points, num_point):
            points = np.array(points)
            n = len(points) - 1
            t = np.linspace(0, 1, num_point)
            polynomial_array = np.array([self.bernstein_poly(i, n, t) for i in range(n+1)])
            curve = np.dot(points.T, polynomial_array).T
            return curve
        def comb_math(self, n, k):
            return math.factorial(n) // (math.factorial(k) * math.factorial(n - k))
        def bernstein_poly(self, i, n, t):
            return self.comb_math(n, i) * (t**(n-i)) * (1 - t)**i
    
    
    
    
        def generate_spline(self,cones, num_points):
            cones = np.array(cones)
            curve_points = []
            for i in range(len(cones) - 1):
                for j in range(num_points):
                    t = j / (num_points - 1)
                    point = (1 - t) * cones[i] + t * cones[i + 1]
                    curve_points.append(point)
            return np.array(curve_points)
    
        def euclidean_distance(self, point1, point2):
            return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
    
        # 새로운 콘을 리스트에 추가할지 여부를 결정하는 함수
        def should_add_cone(self, cones_list, new_cone, min_distance):
            if not cones_list:
                return True
            last_cone = cones_list[-1]
    
            return self.euclidean_distance(last_cone, new_cone) < min_distance and np.pi/3>np.arctan2(-last_cone[1]+new_cone[1],last_cone[0]-new_cone[0]) and 0.0 <np.arctan2(-last_cone[1]+new_cone[1],last_cone[0]-new_cone[0])
    
        def find_OK_bet_R_L_bool(self, left, right):
            for left_index in left:
                for right_index in right:
                    if self.euclidean_distance(left_index,right_index)>self.min_distance_threshold:
                        return False   
            return True
    def lidar_callback(self, _data):
        total_obj_cnt = _data.size    
        self.L_closet_obs1 = None
        self.current_time = time.time()
        pointcloud = []
    
        bev_msg = PoseArray()
        bev_msg.header = _data.header

        obj = _data.array
    
        left_cones = []
        right_cones = []
        obj_collector = []

        for i, obj in enumerate(obj):
            if len(obj.bev.data) < 8:
                rospy.logwarn("Invalid bounding box format: %s", obj.bev.data)
                continue
            bbox_center_x, bbox_center_y = self.calculate_bounding_box_center(obj.bev.data)
            bbox_width, bbox_height = self.calculate_bounding_box_dimensions(obj.bev.data)

            if obj.height == 1.2345678806304932:
                obj_collector.append([bbox_center_x, bbox_center_y, bbox_width, bbox_height])
    
        left_cones.append((-0.4, 1.5))
        right_cones.append((-0.4, -1.5))
        obj_collector.sort(key=lambda x: x[0])
        lfirst = False
        rfirst = False

        for bbox_center_x, bbox_center_y, bbox_width, bbox_height in obj_collector:
            if self.State == "Rubber_cone_drive":
                new_cone = (bbox_center_x, bbox_center_y)
                if len(left_cones) > 10 or len(right_cones) > 10: 
                    break
                if lfirst == False and 0 < bbox_center_x < 3 and -0.1 < bbox_center_y < 3:
                    print("L find")
                    lfirst = True
                    left_cones.append(new_cone)
                elif lfirst == True and -1 < bbox_center_y < 1.5:
                    if self.should_add_cone(left_cones, new_cone, self.min_distance_threshold):
                        left_cones.append(new_cone)

        # 여기서 RRTStarLimitedSteering 클래스 사용
        x_limits = (0, 12)
        y_limits = (0, 12)
        rrt_star = RRTStarLimitedSteering([0, 0], left_cones[-1], left_cones[:-1], x_limits, y_limits)
        path = rrt_star.plan()

        self.publish_obstacles_array_one(obj_collector, self.all_rabacone_point_pub, color=(1.0, 1.0, 0.0))

        if len(left_cones) >= 2 and len(right_cones) < 2:
            left_curve = self.bezier_curve(left_cones, self.line_space)
            right_curve = [(row[0], row[1] - 3) for row in left_curve]
        elif len(left_cones) < 2 and len(right_cones) >= 2:
            right_curve = self.bezier_curve(right_cones, self.line_space)
            left_curve = [(row[0], row[1] + 3) for row in right_curve]
        elif len(left_cones) < 2 and len(right_cones) < 2:
            return
        else:
            left_curve = self.bezier_curve(left_cones, self.line_space)
            right_curve = self.bezier_curve(right_cones, self.line_space)

        uturn_cone = []
        for left, right in zip(left_curve, right_curve):
            if (left[0] + right[0]) > 0:
                uturn_cone.append([(left[0] + right[0]) / 2, (left[1] + right[1]) / 2])
    
        uturn_cone.sort(key=lambda x: x[0])

        if uturn_cone:
            target_point = Float32MultiArray()
            target_point.data.append(uturn_cone[4][1] * 3)
            self.target_point_publisher.publish(target_point)
            self.publish_obstacles(path, self.middle_point_pub, color=(1.0, 1.0, 0.0))
            self.publish_obstacles_array(left_cones, right_cones, self.rabacone_point_pub, color=(1.0, 1.0, 0.0))
    
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

