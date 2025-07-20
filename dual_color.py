#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from std_msgs.msg import Header

pub_1 = None
pub_2 = None

def color_points(msg, color):
    points_list = []
    for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        x, y, z = p
        points_list.append([x, y, z])
    points = np.array(points_list, dtype=np.float32)
    if points.shape[0] == 0:
        return None

    colors = np.tile(color, (points.shape[0], 1))

    points_rgb = []
    for i in range(points.shape[0]):
        x, y, z = points[i, :3]
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
    rospy.loginfo("Subscribed to /points and /points2, publishing colored point clouds")
    rospy.spin()

if __name__ == '__main__':
    listener()
