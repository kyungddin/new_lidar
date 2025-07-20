#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion

# ROI (거울로 판단할 영역) 설정
ROI = {
    "xmin": -0.5, "xmax": 0.5,
    "ymin": -0.5, "ymax": 0.5,
    "zmin": 0.0,  "zmax": 2.0
}

dual_points1 = np.empty((0, 3), dtype=np.float32)
dual_points2 = np.empty((0, 3), dtype=np.float32)

pub_label = None
pub_marker = None

def in_roi(points):
    if points.size == 0:
        return points
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    mask = (x >= ROI["xmin"]) & (x <= ROI["xmax"]) & \
           (y >= ROI["ymin"]) & (y <= ROI["ymax"]) & \
           (z >= ROI["zmin"]) & (z <= ROI["zmax"])
    return points[mask]

def publish_mirror_box(center):
    marker = Marker()
    marker.header.frame_id = "os_sensor"  # 라이다 프레임에 맞게 수정하세요
    marker.header.stamp = rospy.Time.now()
    marker.ns = "mirror"
    marker.id = 0
    marker.type = Marker.CUBE
    marker.action = Marker.ADD

    # 입력받은 중심 좌표에 마커 위치 설정
    marker.pose.position.x = center[0]
    marker.pose.position.y = center[1]
    marker.pose.position.z = center[2]
    marker.pose.orientation = Quaternion(0, 0, 0, 1)

    # ROI 크기로 마커 크기 설정
    marker.scale.x = ROI['xmax'] - ROI['xmin']
    marker.scale.y = ROI['ymax'] - ROI['ymin']
    marker.scale.z = ROI['zmax'] - ROI['zmin']

    # 빨간색, 반투명
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 0.4

    pub_marker.publish(marker)

def callback_points1(msg):
    global dual_points1
    points = []
    for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        points.append([p[0], p[1], p[2]])
    dual_points1 = np.array(points, dtype=np.float32) if points else np.empty((0, 3))

def callback_points2(msg):
    global dual_points2, pub_label
    points = []
    for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        points.append([p[0], p[1], p[2]])
    dual_points2 = np.array(points, dtype=np.float32) if points else np.empty((0, 3))

    # ROI 필터링
    roi_p1 = in_roi(dual_points1)
    roi_p2 = in_roi(dual_points2)

    mirror_detected = False
    if roi_p1.shape[0] > 10 and roi_p2.shape[0] > 10:
        mirror_detected = True

    pub_label.publish(Bool(data=mirror_detected))

    if mirror_detected:
        # 두 포인트 클라우드 내 ROI 영역 포인트 중심 좌표 평균 계산
        center1 = np.mean(roi_p1, axis=0)
        center2 = np.mean(roi_p2, axis=0)
        center = (center1 + center2) / 2.0
        publish_mirror_box(center)
    else:
        # 박스 제거용 빈 Marker 메시지 (DELETE 액션)
        marker = Marker()
        marker.header.frame_id = "os_sensor"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "mirror"
        marker.id = 0
        marker.action = Marker.DELETE
        pub_marker.publish(marker)

def listener():
    global pub_label, pub_marker
    rospy.init_node('mirror_labeling_node', anonymous=True)
    rospy.Subscriber('/processed_points1', PointCloud2, callback_points1)
    rospy.Subscriber('/processed_points2', PointCloud2, callback_points2)
    pub_label = rospy.Publisher('/mirror_label', Bool, queue_size=1)
    pub_marker = rospy.Publisher('/mirror_marker', Marker, queue_size=1)
    rospy.loginfo("Mirror labeling node started, publishing label and RViz marker")
    rospy.spin()

if __name__ == '__main__':
    listener()
