#!/usr/bin/env python3

import rospy
import numpy as np
import time
import tf
import math
import threading
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from shapely.geometry import Point as ShapelyPoint, Polygon
from erp_driver.msg import erpCmdMsg, erpStatusMsg

def distance(x1,y1,x2,y2):
    return ((x1-x2)**2 + (y1-y2)**2)**0.5

######################## ===================================================================
#### Visualization ##### ===================================================================
######################## ===================================================================

# 장애물 Marker를 표시 (rviz occ map 위의 빨간 점)
def make_obstacle_marker(obstacle_cells, frame_id='velodyne'):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()

    marker.ns = "obstacles"
    marker.id = 0
    marker.type = Marker.POINTS
    marker.action = Marker.ADD

    # 점 크기
    marker.scale.x = 0.2
    marker.scale.y = 0.2

    # 색상 (빨간색)
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    # 매 프레임마다 새로운 점 갱신
    marker.points = []  # 초기화

    for x, y in obstacle_cells:
        p = Point()
        p.x = x
        p.y = y
        p.z = -0.7
        marker.points.append(p)

    return marker


# marker의 warn : orientation 초기화(정규화 문제)
def paths_to_marker_array(paths, frame_id='base_link'):
    marker_array = MarkerArray()
    tt = rospy.Time.now()

    for i, path in enumerate(paths):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.lifetime = rospy.Duration(0.1)
        marker.header.stamp = tt
        marker.ns = "candidate_paths"
        marker.id = i
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # line width

        # Color (e.g., greenish)
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.2

        marker.points = []
        for x, y, rad, cl__ in path:
            pt = Point()
            pt.x = x
            pt.y = y
            pt.z = 0.2
            marker.points.append(pt)

        marker_array.markers.append(marker)
        
    return marker_array

# marker의 warn : orientation 초기화(정규화 문제)
def car_area_maker_array():
    
    fr = 1 #front/ real
    rl = 0.6 #right left
    
    corners = [
        (fr, rl, 0.0),
        (-fr, rl, 0.0),
        (-fr, -rl, 0.0),
        (fr, -rl, 0.0)
    ]
    marker = Marker()
    marker.header.frame_id = 'base_link'
    marker.header.stamp = rospy.Time.now()

    marker.ns = "rectangle"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD

    marker.scale.x = 0.1  # 선 두께 (meters)

    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    marker.points = []

    # 선의 좌표 순서
    for corner in corners:
        p = Point()
        p.x, p.y, p.z = corner
        marker.points.append(p)
    
    # 사각형 닫기 위해 첫 점 다시 추가
    p = Point()
    p.x, p.y, p.z = corners[0]
    marker.points.append(p)

    return marker

def path_area_maker(p_fl, p_rl, p_rr, p_fr,id):
    
    fr = 1 #front/ real
    rl = 0.6 #right left
    
    corners = [
        (p_fl[0], p_fl[1], 0.0),
        (p_rl[0], p_rl[1], 0.0),
        (p_rr[0], p_rr[1], 0.0),
        (p_fr[0], p_fr[1], 0.0)
    ]
    marker = Marker()
    marker.header.frame_id = 'base_link'
    marker.header.stamp = rospy.Time.now()

    marker.ns = "rectangle"
    marker.id = id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD

    marker.scale.x = 0.1  # 선 두께 (meters)

    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    # 선의 좌표 순서
    for corner in corners:
        p = Point()
        p.x, p.y, p.z = corner
        marker.points.append(p)
    
    # 사각형 닫기 위해 첫 점 다시 추가
    p = Point()
    p.x, p.y, p.z = corners[0]
    marker.points.append(p)

    return marker

def make_circle_marker(center_x, center_y, radius, marker_id, frame_id='map', resolution=0.1):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.lifetime = rospy.Duration(0.1)
    marker.header.stamp = rospy.Time.now()
    marker.ns = "circle"
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD

    marker.scale.x = 0.05  # 선 두께

    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 0.3

    marker.points = []
    num_points = int(2 * math.pi / resolution) + 1

    for i in range(num_points):
        angle = i * resolution
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        p = Point()
        p.x = x
        p.y = y
        p.z = 0.21
        marker.points.append(p)

    marker.points.append(marker.points[0])  # 원 닫기

    return marker

def convert_to_path_msg(path_points):
    path_msg = Path()
    # path_msg.header = Header()
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = 'base_link'

    for pt in path_points:
        pose = PoseStamped()
        pose.header = path_msg.header
        pose.pose.position.x = pt[0]
        pose.pose.position.y = pt[1]
        pose.pose.position.z = 0.0

        # orientation: 정면을 바라보는 단위 quaternion
        quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        path_msg.poses.append(pose)

    return path_msg

######################################################################################
######################################################################################

# callback 함수의 비동기 문제로 obs cell 참조가 제대로 안될 수 있음?
obstacle_cells = []
obstacle_lock = threading.Lock()
def occ_callback(msg):
    global obstacle_cells
    res = msg.info.resolution
    w = msg.info.width
    h = msg.info.height
    origin_x = msg.info.origin.position.x
    origin_y = msg.info.origin.position.y

    data = msg.data

    new_cells = []
    for idx, val in enumerate(data):
        if val == 100:
            row = idx // w
            col = idx % w

            # lidar frame 기준 장애물 위치, 0.2는 셀 크기 반영
            obs_x = origin_x + (col * res) + 0.1
            obs_y = origin_y + (row * res) + 0.1

            new_cells.append((obs_x, obs_y))
    with obstacle_lock:
        obstacle_cells = new_cells 
    # [[x1, y1,], [x2, y2], ...]

    marker_pub = rospy.Publisher('/obstacle_points', Marker, queue_size=10)
    # Marker 생성
    marker_msg = make_obstacle_marker(obstacle_cells, frame_id=msg.header.frame_id)
    # 퍼블리시
    marker_pub.publish(marker_msg)


car_x = 0
car_y = 0
car_yaw = 0
car_deg = 0

def utm_Callback(msg):
    global car_x, car_y
    
    # 위치 정보
    car_x = msg.pose.pose.position.x
    car_y = msg.pose.pose.position.y

def yaw_Callback(msg):
    global car_yaw
    car_yaw = msg.data


gpath2loc = []
gpath2loc_lock = threading.Lock()
def path_callback(msg):
    global gpath2loc
    new_gpath2loc = []  # 이전 경로 초기화

    for pose_stamped in msg.poses:
        gx = pose_stamped.pose.position.x
        gy = pose_stamped.pose.position.y

        dx = gx - car_x
        dy = gy - car_y

        loc_x = math.cos(-car_yaw)*dx - math.sin(-car_yaw)*dy
        loc_y = math.sin(-car_yaw)*dx + math.cos(-car_yaw)*dy
        
        new_gpath2loc.append((loc_x,loc_y))
    with gpath2loc_lock:
        gpath2loc = new_gpath2loc





def run():

    rospy.init_node('obs_local_planner')

    # publisher
    select_path_pub = rospy.Publisher('/select_path', Path, queue_size=10)
    path_viz = rospy.Publisher('/path_viz', MarkerArray, queue_size=1)
    patharea_viz = rospy.Publisher('/path_area_viz', MarkerArray, queue_size=1)
    car_viz = rospy.Publisher('/car_viz', Marker, queue_size=1)
    circlearea_pub = rospy.Publisher("/circle_markers", MarkerArray, queue_size=10)

    # subscriber
    rospy.Subscriber('/local_map',OccupancyGrid , occ_callback)
    rospy.Subscriber('/global_path',Path , path_callback)
    rospy.Subscriber('/odom_gps', Odometry, utm_Callback)
    rospy.Subscriber('/vehicle_yaw', Float32, yaw_Callback)
    # rospy.Subscriber("/erp42_status", erpStatusMsg, erp_status_callback)

    wheelbase = 1.2
    deg_max = 30
    pathlen_max = 5
    pahtlen_min = 3
    angle_deg_list_right = np.arange(-deg_max, 0, 5)        # -90도 ~ 90도, 5도 간격
    angle_deg_list_left = np.arange(0, deg_max+1, 5)
    angle_rad_list_right = np.deg2rad(angle_deg_list_right)   # 라디안 변환
    angle_rad_list_left = np.deg2rad(angle_deg_list_left)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        start_time = time.time()

        selected_idx = -1


        all_paths = []  # 모든 각도별 경로 저장
        all_path_len = []

        # ------------------------------------------------------- #
                            # 주행 경로 생성 #
        # ------------------------------------------------------- #

        for steering_deg in range(-30,30,10):
            x = -0.6 # 논홀로노믹은 뒷바퀴 차축 기준
            y = 0
            yaw = 0
            l = 0
            steering_rad = np.deg2rad(steering_deg)
            path = []
            dt = 0.1
            v = 1.3

            rp = 1 - abs(steering_deg / deg_max)
            sim_max = 3
            sim_min = 1.5
            range_max = int((sim_min + (sim_max - sim_min) * rp)/0.1)
            

            for i in range(range_max+1):
                path.append((x, y, yaw,l)) # x, y, 점에서 각도, 누적 길이
                t = i * dt
                # Kinematic bicycle update equations
                dx = v * np.cos(yaw) * dt
                dy = v * np.sin(yaw) * dt
                dl = (dx**2 + dy**2)**0.5
                x += dx
                y += dy
                l += dl
                yaw += (v / wheelbase) * np.tan(steering_rad) * dt
        
            all_paths.append(path)

        # path_area_marker_array = MarkerArray()
        max_safe_len = len(all_paths)

        # -------------------------------------------------------- #
                            # 경로상 장애물 확인 #
        # -------------------------------------------------------- #

        # 장애물 point mutex 
        obscell_copy = []
        with obstacle_lock:
            obscell_copy = obstacle_cells[:] 
        # global path -> localization mutex
        gpath2loc_copy = []
        with gpath2loc_lock:
            gpath2loc_copy = gpath2loc

        safe_path_idx = []
        pathlen2obs = []
        
        # check safe path
        for path_idx, path_ in enumerate(all_paths):
            safe_flag = True
            for path_point in path_:
                
                p_x_rear = path_point[0]
                p_y_rear = path_point[1]
                pyaw = path_point[2]

                p_x_front = p_x_rear + wheelbase * math.cos(pyaw)
                p_y_front = p_y_rear + wheelbase * math.sin(pyaw)

                for op in obscell_copy:
                    # occ_map은 velodyne 기준이고 path는 base link 여서 매칭이 안되는 문제가 발생
                    # 이를 맞춰주기 위해 장애물 지도가 base link기준 0.82m 앞에 있음을 고려.

                    # path point는 차량 뒤축 중심. 앞축에 대한 장애물 확인도 필요.
                    # 앞축 : front_d
                    # 뒤축 : rear_d

                    rear_d = distance(p_x_rear, p_y_rear, op[0] + 0.82, op[1])
                    front_d = distance(p_x_front, p_y_front, op[0] + 0.82, op[1])

                    if rear_d <= 0.6 or front_d <= 0.6:
                        safe_flag = False
                        pathlen2obs.append((path_idx, path_point[3]))
                        break
                if safe_flag == False:
                    break
            if safe_flag == True:
                safe_path_idx.append(path_idx)
                pathlen2obs.append((path_idx, path_[-1][3]))
            

        # 안전한 경로중 선택하는 알고리즘

        if len(safe_path_idx) == max_safe_len: 
            # 장애물이 없으면 그냥 경로 따라가기
            print("NO obstacle")

            selected_idx = 3 # 임시.
            param1_score = []
            for idx in safe_path_idx:
                
                path_last_point = all_paths[idx][-1]
                plp_x = path_last_point[0]
                plp_y = path_last_point[1]

                min_dist = 100
                if len(gpath2loc_copy) > 0: # received
                    for gp in gpath2loc_copy:
                        dist = distance(plp_x, plp_y, gp[0], gp[1])
                        # print('dist' ,dist)
                        if dist < min_dist:
                            min_dist = dist
                else:
                    rospy.logwarn("Path Not Received")
                
                param1_score.append(min_dist)
            
            
            
            # normalize
            param1_score = np.array(param1_score)
            p1max = np.max(param1_score)
            p1min = np.min(param1_score)
            param1_score = 1 - (param1_score - p1min) / (p1max - p1min)

            selected_idx = safe_path_idx[np.argmax(param1_score)]

        elif len(safe_path_idx) == 0:

            print("No possible path")
            
            selected_idx = 3
            # # 장애물과 충돌하기 바로 직전이면 
            #     # 정지 후 일정 거리 후진.

            # # 장애물과 충돌할 정도가 아니면
            #     # all path에서 가장 길이가 긴 경로를 선택
            # param1_score = []
            # param1_score = pathlen2obs
            # param1_score = np.array(param1_score)
            # p1max = np.max(param1_score)
            # p1min = np.min(param1_score)
            # param1_score = (param1_score - p1min) / (p1max - p1min)
            # selected_idx = np.argmax(param1_score)

        else:
            print("")
            print("testcase")
            print(f"all path len  : {len(all_path_len)}")
            print(f"safe path len : {len(safe_path_idx)}")
            print(safe_path_idx)
            for idx in range(len(all_paths)):
                print(f"idx : {idx} original : {all_paths[idx][-1][3]:.2f} tobs : {pathlen2obs[idx][1]:.2f}")

            param1_score = []
            param2_score = []
            # param1 (path_tracking param)
            # local 경로의 마지막 점이 global path와 얼마나 가까운지 판단하여 score 부여.
            # 가장 *높은* score를 가진 경로가 선택됨.

            for idx in safe_path_idx:
                
                path_last_point = all_paths[idx][-1]
                plp_x = path_last_point[0]
                plp_y = path_last_point[1]

                min_dist = 100
                if len(gpath2loc_copy) > 0: # received
                    for gp in gpath2loc_copy:
                        dist = distance(plp_x, plp_y, gp[0], gp[1])
                        # print('dist' ,dist)
                        if dist < min_dist:
                            min_dist = dist
                else:
                    rospy.logwarn("Path Not Received")
                
                param1_score.append(min_dist)
            
            
            
            # normalize
            param1_score = np.array(param1_score)
            p1max = np.max(param1_score)
            p1min = np.min(param1_score)
            param1_score = 1 - (param1_score - p1min) / (p1max - p1min)

            # print(f"param 1 score {param1_score}")

            
            # param2 (longest param)
            # safe path들에 남은 거리에 따라서 score를 부여

            for idx in safe_path_idx:
                cl = all_paths[idx][-1][3] # 누적 길이
                param2_score.append(cl)

            param2_score = np.array(param2_score)
            p2max = np.max(param2_score)
            p2min = np.min(param2_score)
            param2_score = (param2_score - p2min) / (p2max - p2min)

            # print(f"param 2 score {param2_score}")

            # Combine both scores
            p1gain = 0.5
            p2gain = 1

            total_score = p1gain * param1_score + p2gain * param2_score
            selected_idx = safe_path_idx[np.argmax(total_score)]
            
            
            
            
        # visualization # ----------------------------
        if selected_idx != -1:
            print('sel : ',selected_idx)
            safe_paths = []
            # for idx in safe_path_idx:
            #     safe_paths.append(all_paths[idx])
            safe_paths.append(all_paths[selected_idx])
            pathmkarr = paths_to_marker_array(all_paths)
            path_viz.publish(pathmkarr)
            

            ppoint = []
            for safe_path in safe_paths:
                for safe_point in safe_path:
                    sp_x_rear = safe_point[0]
                    sp_y_rear = safe_point[1]
                    spyaw = safe_point[2]

                    sp_x_front = sp_x_rear+ wheelbase * math.cos(spyaw)
                    sp_y_front = sp_y_rear + wheelbase * math.sin(spyaw)
                    ppoint.append((sp_x_rear, sp_y_rear, sp_x_front, sp_y_front)) # for circle viz

            cmarker_array = MarkerArray()
            r_radius = 0.6
            f_radius = 0.6
            for idx, (x1, y1, x2, y2) in enumerate(ppoint):
                
                r_marker = make_circle_marker(x1, y1, r_radius, marker_id=idx, frame_id='base_link')
                f_marker = make_circle_marker(x2, y2, f_radius, marker_id=idx+100, frame_id='base_link')
                cmarker_array.markers.append(r_marker)
                cmarker_array.markers.append(f_marker)

            circlearea_pub.publish(cmarker_array)

        carareamkarr = car_area_maker_array()
        car_viz.publish(carareamkarr)
        

        end_time = time.time()
        # time.sleep(0.1)
        Hz = 1 / (end_time - start_time)
        print(f"Loop Hz: {Hz:.1f} Hz")

        # 목표 발행 주기? 10Hz 이상. velodnye이 10Hz임





if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass