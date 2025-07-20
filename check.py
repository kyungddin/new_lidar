import rospy
from sensor_msgs.msg import PointCloud2

def print_fields(msg):
    print("fields:")
    for field in msg.fields:
        print(f"  name: {field.name}, offset: {field.offset}, datatype: {field.datatype}, count: {field.count}")

rospy.init_node('print_pointcloud_fields', anonymous=True)
msg = rospy.wait_for_message('/ouster/points', PointCloud2)
print_fields(msg)
