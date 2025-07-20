#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np
from std_msgs.msg import Header

# 퍼블리셔 생성
pub = None

def callback(msg):
    global pub

    # 1. ROS PointCloud2 -> numpy 변환
    points_list = []
    for p in pc2.read_points(msg, skip_nans=True):
        x, y, z = p[:3]
        points_list.append([x, y, z])
    points = np.array(points_list, dtype=np.float32)

    # 2. Open3D로 변환
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # ======= 원하는 포인트클라우드 처리 예시 =======
    # 예시: x > 0 영역만 남기기
    points_filtered = points[points[:, 0] > 0]

    # 3. numpy -> PointCloud2 메시지 변환
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = msg.header.frame_id  # 원래 frame_id 사용
    processed_msg = pc2.create_cloud_xyz32(header, points_filtered)

    # 4. 퍼블리시
    pub.publish(processed_msg)
    rospy.loginfo("Published processed point cloud with %d points", len(points_filtered))


def listener():
    global pub
    rospy.init_node('ouster_pointcloud_processor', anonymous=True)
    rospy.Subscriber('/ouster/points', PointCloud2, callback)
    pub = rospy.Publisher('/processed_points', PointCloud2, queue_size=1)
    rospy.loginfo("Node started: Subscribing to /ouster/points and publishing to /processed_points")
    rospy.spin()


if __name__ == '__main__':
    listener()
