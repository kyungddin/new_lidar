#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from std_msgs.msg import Header
import open3d as o3d

pub_1 = None
pub_2 = None

# 파라미터
VOXEL_SIZE = 0.03  # 다운샘플링 voxel 크기 (m)
MAX_DISTANCE = 5.0  # 최대 거리 제한 (m)

def color_points(msg, color):
    points_list = []
    for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        x, y, z = p
        points_list.append([x, y, z])
    points = np.array(points_list, dtype=np.float32)
    if points.shape[0] == 0:
        return None

    # --- Open3D PointCloud 생성 ---
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # 1. 최대 거리 필터링
    distances = np.linalg.norm(points, axis=1)
    mask = distances <= MAX_DISTANCE
    pcd = pcd.select_by_index(np.where(mask)[0])

    if len(pcd.points) == 0:
        return None

    # 2. Voxel Downsampling
    pcd_down = pcd.voxel_down_sample(voxel_size=VOXEL_SIZE)

    points_down = np.asarray(pcd_down.points)
    if points_down.shape[0] == 0:
        return None

    # 색상 생성
    colors = np.tile(color, (points_down.shape[0], 1))

    points_rgb = []
    for i in range(points_down.shape[0]):
        x, y, z = points_down[i, :3]
        r, g, b = colors[i]
        rgb = (int(r) << 16) | (int(g) << 8) | int(b)
        rgb_float = np.frombuffer(np.uint32(rgb).tobytes(), dtype=np.float32)[0]
        points_rgb.append([x, y, z, rgb_float])

    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = msg.header.frame_id

    processed_msg = pc2.create_cloud(header,
                                     fields=[
                                         pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
                                         pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
                                         pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
                                         pc2.PointField('rgb', 12, pc2.PointField.FLOAT32, 1),
                                     ],
                                     points=points_rgb)
    return processed_msg

def callback_points(msg):
    global pub_1
    colored_msg = color_points(msg, [255, 0, 0])  # 빨강
    if colored_msg:
        pub_1.publish(colored_msg)

def callback_points2(msg):
    global pub_2
    colored_msg = color_points(msg, [0, 0, 255])  # 파랑
    if colored_msg:
        pub_2.publish(colored_msg)

def listener():
    global pub_1, pub_2
    rospy.init_node('ouster_dual_return_color', anonymous=True)
    rospy.Subscriber('/ouster/points', PointCloud2, callback_points)
    rospy.Subscriber('/ouster/points2', PointCloud2, callback_points2)
    pub_1 = rospy.Publisher('/processed_points1', PointCloud2, queue_size=1)
    pub_2 = rospy.Publisher('/processed_points2', PointCloud2, queue_size=1)
    rospy.loginfo("Subscribed to /points and /points2, publishing colored point clouds with downsampling and distance filtering")
    rospy.spin()

if __name__ == '__main__':
    listener()

# 처음으로 완전하게 ros topic 통신과 open3d 중간 처리를 구현해낸 코드..!