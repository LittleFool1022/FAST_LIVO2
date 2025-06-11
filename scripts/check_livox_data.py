#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu, PointCloud2

latest_lidar_stamp = None

def lidar_callback(msg):
    global latest_lidar_stamp
    latest_lidar_stamp = msg.header.stamp.to_sec()

def imu_callback(msg):
    if latest_lidar_stamp is None:
        return
    imu_stamp = msg.header.stamp.to_sec()
    delta = imu_stamp - latest_lidar_stamp
    print("[Δt imu - lidar]: {:.6f} sec".format(delta))

if __name__ == "__main__":
    rospy.init_node("imu_lidar_sync_checker")
    rospy.Subscriber("/livox/lidar", PointCloud2, lidar_callback)
    rospy.Subscriber("/livox/imu", Imu, imu_callback)
    rospy.spin()
