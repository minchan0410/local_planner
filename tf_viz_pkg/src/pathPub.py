#!/usr/bin/env python3
import rospy
import time
import csv
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

## git change test


def create_straight_path():
    rospy.init_node('Global_path_published')
    path_pub = rospy.Publisher('/global_path', Path, queue_size=10)

    path_msg = Path()
    path_msg.header.frame_id = "map"
    path_x = []
    path_y = []
    with open("/home/minchan/ck_ws/src/tf_viz_pkg/pathlog/yeon.csv",'r',newline='') as f:
        reader = csv.reader(f)
        for row in reader:
            x = float(row[0])
            y = float(row[1])
            path_x.append(x)
            path_y.append(y)
    
    for idx in range(len(path_x)):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = path_x[idx]
        pose.pose.position.y = path_y[idx]
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0  # 방향 없이 직진
        path_msg.poses.append(pose)

    while not rospy.is_shutdown():
        path_msg.header.stamp = rospy.Time.now()  # 계속 갱신
        path_pub.publish(path_msg)
        time.sleep(0.1)


if __name__ == '__main__':
    try:
        create_straight_path()
    except rospy.ROSInterruptException:
        pass