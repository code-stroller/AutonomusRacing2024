import numpy as np
import rospy
from tracking_msg.msg import TrackingObjectArray
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

# 점유 맵 설정
MAP_SIZE = 10  # 100x100 크기 (예시)
RESOLUTION = 1  # 한 칸당 0.1m (해상도를 높임)
EXPANSION_RADIUS = 1  # 객체의 점유 반경 (셀 단위)

occupancy_map = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.int8)

def update_occupancy_map(tracking_msg):
    global occupancy_map
    occupancy_map.fill(0)  # 맵 초기화
    
    for obj in tracking_msg.array:
        obj_position = obj.point
        x_center = int((obj_position.x + (MAP_SIZE * RESOLUTION) / 2) / RESOLUTION)
        y_center = int((obj_position.y + (MAP_SIZE * RESOLUTION) / 2) / RESOLUTION)
        
        # 객체 중심 좌표에서 반경 내의 셀을 점유
        for dx in range(-EXPANSION_RADIUS, EXPANSION_RADIUS + 1):
            for dy in range(-EXPANSION_RADIUS, EXPANSION_RADIUS + 1):
                x = x_center + dx
                y = y_center + dy
                # 점유 맵 내에서 유효한 범위만 점유 처리
                if 0 <= x < MAP_SIZE and 0 <= y < MAP_SIZE:
                    occupancy_map[0, 5] = 1

    # OccupancyGrid 메시지로 변환하여 발행
    publish_occupancy_map()

def publish_occupancy_map():
    occupancy_grid = OccupancyGrid()
    occupancy_grid.header = Header()
    occupancy_grid.header.stamp = rospy.Time.now()
    occupancy_grid.header.frame_id = "velodyne"

    # 맵 메타데이터 설정
    occupancy_grid.info.resolution = RESOLUTION
    occupancy_grid.info.width = MAP_SIZE
    occupancy_grid.info.height = MAP_SIZE
    occupancy_grid.info.origin.position.x = -MAP_SIZE * RESOLUTION / 2
    occupancy_grid.info.origin.position.y = -MAP_SIZE * RESOLUTION / 2
    occupancy_grid.info.origin.position.z = 0

    # Occupancy 맵 데이터를 OccupancyGrid로 변환하여 저장
    occupancy_grid.data = occupancy_map.flatten().tolist()
    
    map_publisher.publish(occupancy_grid)

def tracking_callback(data):
    # TrackingObjectArray 메시지 수신 후 점유 맵 업데이트
    update_occupancy_map(data)

def main():
    global map_publisher
    rospy.init_node('occupancy_map_node', anonymous=True)

    # map 토픽 발행자 설정
    map_publisher = rospy.Publisher('/map_test', OccupancyGrid, queue_size=10)
    rospy.Subscriber('/tracking_filter_node/filtered_tracking_object', TrackingObjectArray, tracking_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
