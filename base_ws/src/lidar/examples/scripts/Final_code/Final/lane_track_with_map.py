import numpy as np
import rospy
from tracking_msg.msg import TrackingObjectArray
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from erp_driver.msg import erpStatusMsg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sklearn.linear_model import RANSACRegressor
from std_msgs.msg import Float32

# 점유 맵 설정
MAX_WIDTH = 150  # 맵의 너비
MAX_LENGTH = 50  # 맵의 길이
RESOLUTION = 0.1  # 한 칸당 0.1m
EXPANSION_RADIUS = 1  # 객체의 점유 반경 (셀 단위)
HEADING_THRESHOLD = 0.1  # 헤딩 변화 임계값 (라디안)

occupancy_map = np.zeros((MAX_LENGTH, MAX_WIDTH), dtype=np.int8)
map_publisher = None
marker_publisher = None
lane_error_publisher = None

current_speed = 0.0  # m/s 단위
current_heading = 0.0  # 라디안 단위
previous_heading = 0.0  # 이전 헤딩값

def update_occupancy_map(tracking_msg):
    '''
                    ^  MAX_WIDTH
                    |
                    |
                    |
         car        |
                    |
                    |
                    |
                    |
                    x
    <------------y
    MAX_LENGTH
    
    점유 맵 좌표계.
    lane_detection_testing.py 참조
    
    '''



    global occupancy_map, current_heading, previous_heading
    new_map = np.zeros((MAX_LENGTH, MAX_WIDTH), dtype=np.int8)  # 새 지도 생성
    
    # 헤딩 변화량 계산
    delta_heading = current_heading - previous_heading
    previous_heading = current_heading  # 이전 헤딩값 갱신
    
    # 차량의 이동량 계산
    dx = -current_speed * 0.1  # 이동한 거리 (x 축)
    dy = 0.0  # y 축 이동은 없음 (직선 기준)
    
    # 회전 변환 행렬 적용 (헤딩 변화가 임계값을 초과하는 경우에만)
    if abs(delta_heading) > HEADING_THRESHOLD:
        cos_delta = np.cos(delta_heading)
        sin_delta = np.sin(delta_heading)
    else:
        cos_delta = 1.0  # 회전 변환을 하지 않음
        sin_delta = 0.0
    
    # 기존 점들을 이동하여 SLAM 효과 구현 (회전 및 평행 이동)
    for y in range(MAX_LENGTH):
        for x in range(MAX_WIDTH):
            if occupancy_map[y, x] > 0:
                # 기존 점의 위치를 기준으로 이동 후 회전
                x_shifted = x - (MAX_WIDTH // 2)  # 중심 기준으로 이동
                y_shifted = y - (MAX_LENGTH // 2)
                
                # 회전 변환 적용 (임계값을 초과할 때만 회전 반영)
                new_x = int((x_shifted * cos_delta - y_shifted * sin_delta) + dx / RESOLUTION + MAX_WIDTH // 2)
                new_y = int((x_shifted * sin_delta + y_shifted * cos_delta) + dy / RESOLUTION + MAX_LENGTH // 2)
                
                # 새로운 위치가 맵 범위 내에 있으면 값을 이동
                if 0 <= new_x < MAX_WIDTH and 0 <= new_y < MAX_LENGTH:
                    new_map[new_y, new_x] = occupancy_map[y, x]
    
    # 새로운 트래킹 데이터로 점유 맵 갱신
    for obj in tracking_msg.array:
        obj_position = obj.point
        x_center = int((obj_position.x + (MAX_WIDTH * RESOLUTION) / 2) / RESOLUTION)
        y_center = int((obj_position.y + (MAX_LENGTH * RESOLUTION) / 2) / RESOLUTION)
        
        for dx in range(-EXPANSION_RADIUS, EXPANSION_RADIUS + 1):
            for dy in range(-EXPANSION_RADIUS, EXPANSION_RADIUS + 1):
                x = x_center + dx
                y = y_center + dy
                if 0 <= x < MAX_WIDTH and 0 <= y < MAX_LENGTH:
                    new_map[y, x] = 100  # 새 데이터 추가

    occupancy_map[:, :] = new_map  # 업데이트된 맵을 기존 맵에 반영

    # 왼쪽과 오른쪽에 RANSAC을 사용하여 직선 검출
    lines = find_lines_with_ransac_split(occupancy_map)
    # print(lines)
    # lines 딕셔너리에서 'left'와 'right' 키가 없을 경우 None을 기본값으로 설정
    left_line = lines.get('left', None)
    right_line = lines.get('right', None)

    # calculate_center_offset 함수를 호출하여 중간 지점과 좌/우측 선까지의 거리 계산
    lane_error = calculate_center_offset(
        left_line=left_line,
        right_line=right_line,
        distance_ahead=2.0  # 차량 전방 거리를 기준으로 계산
    )

    
    lane_error_publisher.publish(lane_error)

    # print('cetner point', center_point, 'left dist', left_distance, 'right dist', right_distance)    

    # 검출된 직선을 RViz에 시각화
    publish_lines_in_rviz(lines)

    # OccupancyGrid 메시지로 변환하여 발행
    publish_occupancy_map()

def publish_occupancy_map():
    occupancy_grid = OccupancyGrid()
    occupancy_grid.header = Header()
    occupancy_grid.header.stamp = rospy.Time.now()
    occupancy_grid.header.frame_id = "velodyne"

    occupancy_grid.info.resolution = RESOLUTION
    occupancy_grid.info.width = MAX_WIDTH
    occupancy_grid.info.height = MAX_LENGTH
    occupancy_grid.info.origin.position.x = -MAX_WIDTH * RESOLUTION / 2
    occupancy_grid.info.origin.position.y = -MAX_LENGTH * RESOLUTION / 2
    occupancy_grid.info.origin.position.z = 0

    occupancy_grid.data = occupancy_map.flatten().tolist()
    map_publisher.publish(occupancy_grid)

def publish_lines_in_rviz(lines):
    marker = Marker()
    marker.header.frame_id = "velodyne"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "lane_lines"
    marker.id = 0
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 0.05  # 선의 두께 설정
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0

    # 왼쪽과 오른쪽 선 추가
    for key, (start, end) in lines.items():
        start_point = Point()
        start_point.x = (start[0] - MAX_WIDTH / 2) * RESOLUTION
        start_point.y = (start[1] - MAX_LENGTH / 2) * RESOLUTION
        start_point.z = 0.0

        end_point = Point()
        end_point.x = (end[0] - MAX_WIDTH / 2) * RESOLUTION
        end_point.y = (end[1] - MAX_LENGTH / 2) * RESOLUTION
        end_point.z = 0.0

        marker.points.append(start_point)
        marker.points.append(end_point)

    marker_publisher.publish(marker)

def tracking_callback(data):
    update_occupancy_map(data)

def erp42_status_callback(status_msg):
    global current_speed
    current_speed = status_msg.speed / 10.0  # 속도를 m/s로 변환

def odom_gps_callback(odom_msg):
    global current_heading
    current_heading = odom_msg.pose.pose.position.z  # 라디안 단위의 헤딩값


def vehicle_yaw_callback(msg):
    
    # current_heading = msg.data
    pass

def find_lines_with_ransac_split(occupancy_map):
    #RANSAC으로 선 만듦.
    points = np.column_stack(np.where(occupancy_map > 0))  # occupancy_map에서 값이 0이 아닌 좌표들만 추출
    
    center_y = occupancy_map.shape[0] // 2

    left_points = points[points[:, 0] < center_y]
    right_points = points[points[:, 0] >= center_y]

    line_segments = {}

    #혹시 될까 싶어서 넣었는데 안되는듯.
    def compute_weights(points):
        distances = np.abs(points[:, 0] - center_y)
        return 1 / ((distances + 1e-5))

    if len(left_points) > 1:
        X_left = left_points[:, 1].reshape(-1, 1)  # x 좌표 (열)
        y_left = left_points[:, 0]  # y 좌표 (행)
        
        weights_left = compute_weights(left_points)
        ransac_left = RANSACRegressor()
        ransac_left.fit(X_left, y_left, sample_weight=weights_left)
        
        line_x_left = np.array([X_left.min(), X_left.max()])
        line_y_left = ransac_left.predict(line_x_left.reshape(-1, 1))
        
        line_segments['right'] = ((line_x_left[0], line_y_left[0]), (line_x_left[1], line_y_left[1]))

    if len(right_points) > 1:
        X_right = right_points[:, 1].reshape(-1, 1)
        y_right = right_points[:, 0]
        
        weights_right = compute_weights(right_points)
        ransac_right = RANSACRegressor()
        ransac_right.fit(X_right, y_right, sample_weight=weights_right)
        
        line_x_right = np.array([X_right.min(), X_right.max()])
        line_y_right = ransac_right.predict(line_x_right.reshape(-1, 1))
        
        line_segments['left'] = ((line_x_right[0], line_y_right[0]), (line_x_right[1], line_y_right[1]))

    return line_segments

def calculate_center_offset(left_line=None, right_line=None, distance_ahead=1.0):
    '''
    중심에서 얼마나 떨어져 있는지 계산하는 함수. 
    좌, 우 선을 이용해서 중심점을 계산 한 다음 차량의 일정 거리 앞 점이랑 차이를 비교해서 발행


                    ^  MAX_WIDTH
                    |
                    |
                    |
         car        |
                    |
                    |
                    |
                    |
                    x
    <------------y
    MAX_LENGTH
    '''
    if left_line and right_line:
        # Extract points in the form (y, x)
        y1, x1 = left_line[0]
        y2, x2 = left_line[1]
        y3, x3 = right_line[0]
        y4, x4 = right_line[1]

        # Calculate slopes and intercepts for the left and right lines in terms of x = my + c
        left_slope = (x2 - x1) / (y2 - y1)
        left_intercept = x1 - left_slope * y1
        
        right_slope = (x4 - x3) / (y4 - y3)
        right_intercept = x3 - right_slope * y3

        mid_slope = (left_slope + right_slope) / 2
        mid_intercept = (left_intercept + right_intercept) / 2
        
        print(f"Both line exist")

    elif left_line:
        # 좌측 선만 있을 때 가상의 중심선 설정
        y1, x1 = left_line[0]
        y2, x2 = left_line[1]
        
        left_slope = (x2 - x1) / (y2 - y1)
        left_intercept = x1 - left_slope * y1

        mid_slope = left_slope
        mid_intercept = left_intercept - 1.75 / RESOLUTION  # 좌측에서 1.75m 우측으로 이동
        print(f"only left line exist")

    elif right_line:
        # 우측 선만 있을 때 가상의 중심선 설정
        y3, x3 = right_line[0]
        y4, x4 = right_line[1]
        
        right_slope = (x4 - x3) / (y4 - y3)
        right_intercept = x3 - right_slope * y3

        mid_slope = right_slope
        mid_intercept = right_intercept + 1.75 / RESOLUTION  # 우측에서 1.75m 좌측으로 이동
        print(f"only right line exist")

    else:
        # 선이 없는 경우 기본값으로 반환
        return lane_error/100


    # Target y-value ahead in the lane
    x_target = distance_ahead + MAX_WIDTH/2
    # print(x_target)


    # 중간선에서 y값 계산
    y_mid =  mid_slope * x_target + mid_intercept

    # 차량은 항상 y 축 중간에 있음.(원점)
    y_self = MAX_LENGTH/2
    # print('center_y coord',y_mid)

    #단위는 RESOLUTION(초기작성시 10cm)
    lane_error = y_self-y_mid
    


    
    # print(f"Midpoint Line: y = {mid_slope}x + {mid_intercept}")
    print(f"Distance between mid and self at y={x_target}: {lane_error/10}")
    
    return lane_error/10



def main():
    global map_publisher, marker_publisher, lane_error_publisher
    rospy.init_node('occupancy_map_node', anonymous=True)

    map_publisher = rospy.Publisher('/map_lane', OccupancyGrid, queue_size=10)
    marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
    lane_error_publisher = rospy.Publisher('/lidar/lane_error', Float32, queue_size=10)
    
    rospy.Subscriber('/tracking_filter_node/filtered_tracking_object', TrackingObjectArray, tracking_callback)
    rospy.Subscriber('/erp42_status', erpStatusMsg, erp42_status_callback)
    rospy.Subscriber('/odom_gps', Odometry, odom_gps_callback)
    
    rospy.Subscriber("/vehicle_yaw", Float32, vehicle_yaw_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
