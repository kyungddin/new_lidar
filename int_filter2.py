#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mirror_frame_plane_node.py

- /processed_points1: 거울 프레임(사각 테두리) 포인트
- /processed_points2: 거울 면(표면) 포인트
1) 면 포인트로 평면(PCA) 추정
2) 프레임 포인트를 그 평면에 투영 → 2D convex hull → 사각형 코너 근사
3) 프레임 선(노란색) + 거울 면(파란 반투명 CUBE) RViz 표시
4) 감지 여부 Bool 퍼블리시 (/mirror_label)

작성: ChatGPT 확장 버전
"""

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Point

try:
    from scipy.spatial import ConvexHull
    HAVE_SCIPY = True
except ImportError:
    HAVE_SCIPY = False

try:
    from scipy.spatial.transform import Rotation as R
    HAVE_SCIPY_ROT = True
except ImportError:
    HAVE_SCIPY_ROT = False


# ----------------------------
# 전역 변수 초기화
# ----------------------------
dual_points1 = np.empty((0, 3), dtype=np.float32)  # 프레임 포인트 저장
dual_points2 = np.empty((0, 3), dtype=np.float32)  # 면 포인트 저장

pub_label = None
pub_marker = None


# 파라미터
MIN_FRAME_PTS = 20   # 프레임 최소 포인트 수
MIN_PLANE_PTS = 50   # 면 최소 포인트 수
MIN_AREA = 0.01      # 거울 면적 최소 (m^2)


# ----------------------------
# PointCloud2 메시지를 numpy Nx3 배열로 변환
# ----------------------------
def cloud_to_numpy(msg):
    points = [p[:3] for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)]
    if not points:
        return np.empty((0, 3), dtype=np.float32)
    arr = np.array(points, dtype=np.float32)
    # NaN 또는 Inf 제거
    mask = np.isfinite(arr).all(axis=1)
    return arr[mask]


# ----------------------------
# 평면 피팅 (PCA)
# ----------------------------
def fit_plane_pca(points):
    if points.shape[0] < 3:
        return None, None
    center = np.mean(points, axis=0)
    cov = np.cov(points.T)
    eigvals, eigvecs = np.linalg.eigh(cov)
    normal = eigvecs[:, np.argmin(eigvals)]
    norm = np.linalg.norm(normal)
    if norm < 1e-9:
        normal = np.array([0, 0, 1], dtype=np.float32)
    else:
        normal = normal / norm
    return center, normal


# ----------------------------
# 평면 기준축(u, v, n) 생성
# ----------------------------
def make_plane_basis(normal):
    n = normal / np.linalg.norm(normal)
    ref = np.array([0, 0, 1], dtype=np.float32) if abs(n[2]) < 0.9 else np.array([1, 0, 0], dtype=np.float32)
    u = np.cross(n, ref)
    u_norm = np.linalg.norm(u)
    if u_norm < 1e-9:
        u = np.array([1, 0, 0], dtype=np.float32)
    else:
        u /= u_norm
    v = np.cross(n, u)
    v /= np.linalg.norm(v)
    return u, v, n


# ----------------------------
# 3D 점들을 평면 기준축(u, v)에 투영하여 2D 좌표로 변환
# ----------------------------
def project_to_plane(points, center, u, v):
    rel = points - center
    return np.stack([np.dot(rel, u), np.dot(rel, v)], axis=1)


# ----------------------------
# 2D convex hull 계산 (SciPy 없으면 축 정렬 박스 사용)
# ----------------------------
def get_hull_2d(points_2d):
    if points_2d.shape[0] < 3:
        return None
    if HAVE_SCIPY:
        hull = ConvexHull(points_2d)
        return points_2d[hull.vertices]
    else:
        min_xy = points_2d.min(axis=0)
        max_xy = points_2d.max(axis=0)
        return np.array([
            [min_xy[0], min_xy[1]],
            [max_xy[0], min_xy[1]],
            [max_xy[0], max_xy[1]],
            [min_xy[0], max_xy[1]],
        ], dtype=np.float32)


# ----------------------------
# convex hull로부터 사각형 근사 (PCA 이용 회전 후 axis-aligned bounding box)
# ----------------------------
def approximate_rectangle_from_hull(points_2d):
    if points_2d.shape[0] < 3:
        return None
    center = np.mean(points_2d, axis=0)
    cov = np.cov(points_2d.T)
    eigvals, eigvecs = np.linalg.eigh(cov)
    order = np.argsort(-eigvals)
    basis = eigvecs[:, order]
    pts_local = (points_2d - center).dot(basis)
    min_local = pts_local.min(axis=0)
    max_local = pts_local.max(axis=0)

    rect_local = np.array([
        [min_local[0], min_local[1]],
        [max_local[0], min_local[1]],
        [max_local[0], max_local[1]],
        [min_local[0], max_local[1]],
    ], dtype=np.float32)

    rect_2d = rect_local.dot(basis.T) + center
    return rect_2d


# ----------------------------
# 2D 평면 좌표를 3D 좌표로 변환
# ----------------------------
def plane_to_3d(rect_2d, center, u, v):
    return np.array([center + u * uv[0] + v * uv[1] for uv in rect_2d], dtype=np.float32)


# ----------------------------
# 법선 벡터를 Quaternion으로 변환 (마커 방향 지정용)
# ----------------------------
def normal_to_quaternion(normal):
    n = normal / np.linalg.norm(normal)
    z_axis = np.array([0, 0, 1], dtype=np.float32)
    dot = np.clip(np.dot(z_axis, n), -1.0, 1.0)

    if HAVE_SCIPY_ROT:
        axis = np.cross(z_axis, n)
        norm_axis = np.linalg.norm(axis)
        if norm_axis < 1e-9:
            # 거의 같은 방향 또는 반대 방향
            if dot > 0.9999:
                return Quaternion(0, 0, 0, 1)
            else:
                return Quaternion(1, 0, 0, 0)  # 180도 회전 (X축 기준)
        axis /= norm_axis
        angle = np.arccos(dot)
        rot = R.from_rotvec(axis * angle)
        q = rot.as_quat()  # [x, y, z, w]
        return Quaternion(q[0], q[1], q[2], q[3])
    else:
        axis = np.cross(z_axis, n)
        norm_axis = np.linalg.norm(axis)
        if norm_axis < 1e-9:
            return Quaternion(0, 0, 0, 1)
        axis /= norm_axis
        angle = np.arccos(dot)
        s = np.sin(angle / 2.0)
        return Quaternion(axis[0]*s, axis[1]*s, axis[2]*s, np.cos(angle/2.0))


# ----------------------------
# 프레임 마커 퍼블리시 (노란색 선)
# ----------------------------
def publish_frame_marker(frame_pts3):
    marker = Marker()
    marker.header.frame_id = "os_sensor"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "mirror_frame"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.02
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.pose.orientation = Quaternion(0, 0, 0, 1)

    for p in frame_pts3:
        marker.points.append(Point(*p))
    # 루프 닫기
    marker.points.append(Point(*frame_pts3[0]))

    pub_marker.publish(marker)


# ----------------------------
# 평면 마커 퍼블리시 (파란 반투명 CUBE)
# ----------------------------
def publish_plane_marker(center, normal, size_u, size_v, thickness=0.01):
    marker = Marker()
    marker.header.frame_id = "os_sensor"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "mirror_plane"
    marker.id = 1
    marker.type = Marker.CUBE
    marker.action = Marker.ADD

    marker.scale.x = size_u
    marker.scale.y = size_v
    marker.scale.z = thickness

    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.color.a = 0.3

    marker.pose.position = Point(*center)
    marker.pose.orientation = normal_to_quaternion(normal)

    pub_marker.publish(marker)


# ----------------------------
# 마커 삭제
# ----------------------------
def delete_markers():
    for i in [0, 1]:
        m = Marker()
        m.header.frame_id = "os_sensor"
        m.header.stamp = rospy.Time.now()
        m.ns = "mirror_frame" if i == 0 else "mirror_plane"
        m.id = i
        m.action = Marker.DELETE
        pub_marker.publish(m)


# ----------------------------
# 검출 및 퍼블리시 메인 함수
# ----------------------------
def detect_and_publish():
    global dual_points1, dual_points2, pub_label

    frame_pts = dual_points1
    plane_pts = dual_points2

    # 최소 포인트 수 체크
    if frame_pts.shape[0] < MIN_FRAME_PTS or plane_pts.shape[0] < MIN_PLANE_PTS:
        pub_label.publish(Bool(data=False))
        delete_markers()
        return

    # 평면 피팅
    plane_center, plane_normal = fit_plane_pca(plane_pts)
    if plane_center is None or plane_normal is None:
        pub_label.publish(Bool(data=False))
        delete_markers()
        return

    u, v, n = make_plane_basis(plane_normal)

    # 프레임 점 2D 투영 및 사각형 근사
    frame_2d = project_to_plane(frame_pts, plane_center, u, v)
    hull_2d = get_hull_2d(frame_2d)
    if hull_2d is None:
        pub_label.publish(Bool(data=False))
        delete_markers()
        return

    rect_2d = approximate_rectangle_from_hull(hull_2d)
    if rect_2d is None:
        pub_label.publish(Bool(data=False))
        delete_markers()
        return

    # 면 2D 투영 및 면적 계산
    plane_2d = project_to_plane(plane_pts, plane_center, u, v)
    min_uv = plane_2d.min(axis=0)
    max_uv = plane_2d.max(axis=0)
    size_u = max_uv[0] - min_uv[0]
    size_v = max_uv[1] - min_uv[1]

    if size_u * size_v < MIN_AREA:
        pub_label.publish(Bool(data=False))
        delete_markers()
        return

    # 2D 사각형을 3D 좌표로 변환
    frame_rect3 = plane_to_3d(rect_2d, plane_center, u, v)

    # 마커 퍼블리시
    publish_frame_marker(frame_rect3)
    publish_plane_marker(plane_center, plane_normal, size_u, size_v)

    pub_label.publish(Bool(data=True))


# ----------------------------
# ROS 콜백 함수
# ----------------------------
def points1_callback(msg):
    global dual_points1
    dual_points1 = cloud_to_numpy(msg)


def points2_callback(msg):
    global dual_points2
    dual_points2 = cloud_to_numpy(msg)


# ----------------------------
# 메인 함수
# ----------------------------
def main():
    global pub_label, pub_marker

    rospy.init_node('mirror_frame_plane_node')

    rospy.Subscriber('/processed_points1', PointCloud2, points1_callback)
    rospy.Subscriber('/processed_points2', PointCloud2, points2_callback)

    pub_label = rospy.Publisher('/mirror_label', Bool, queue_size=1)
    pub_marker = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        detect_and_publish()
        rate.sleep()


if __name__ == '__main__':
    main()
