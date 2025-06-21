#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
地图管理器 - 统一管理2D和3D地图的生成、保存和传输
"""

import rospy
import os
import numpy as np
import json
import gzip
import base64
import threading
from datetime import datetime
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf.transformations import euler_from_quaternion

class MapManager:
    def __init__(self):
        rospy.init_node('map_manager', anonymous=True)
        
        # 参数配置
        self.maps_dir = os.path.expanduser("~/catkin_ws/maps")
        self.session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = os.path.join(self.maps_dir, f"session_{self.session_id}")
        os.makedirs(self.session_dir, exist_ok=True)
        
        # 压缩和传输参数
        self.max_points_per_chunk = 10000  # 每包最大点数
        self.compression_level = 6  # gzip压缩级别
        self.enable_compression = True
        
        # 数据存储
        self.current_2d_map = None
        self.current_3d_cloud = None
        self.accumulated_3d_points = []
        self.robot_pose = None
        
        # 状态控制
        self.is_collecting = False
        self.collection_start_time = None
        
        # 线程锁
        self.data_lock = threading.Lock()
        
        # TF相关
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 发布器 - 用于与QT上位机通信
        self.map_data_pub = rospy.Publisher('/map_data_for_qt', String, queue_size=1)
        self.status_pub = rospy.Publisher('/map_manager_status', String, queue_size=1)
        
        # 订阅器
        self.cloud_sub = rospy.Subscriber('/cloud_registered', PointCloud2, 
                                        self.cloud_callback, queue_size=1)
        self.map_sub = rospy.Subscriber('/enhanced_2d_map', OccupancyGrid, 
                                      self.map_callback, queue_size=1)
        
        # 服务
        self.start_collection_srv = rospy.Service('start_map_collection', Trigger, self.start_collection)
        self.stop_collection_srv = rospy.Service('stop_map_collection', Trigger, self.stop_collection)
        self.save_maps_srv = rospy.Service('save_maps', Trigger, self.save_maps)
        self.send_2d_map_srv = rospy.Service('send_2d_map', Trigger, self.send_2d_map)
        self.send_3d_map_srv = rospy.Service('send_3d_map', Trigger, self.send_3d_map)
        
        # 定时器
        self.status_timer = rospy.Timer(rospy.Duration(5.0), self.publish_status)
        
        # 统计信息
        self.stats = {
            'session_id': self.session_id,
            'total_3d_points': 0,
            'map_updates': 0,
            'data_sent': 0,
            'collection_time': 0
        }
        
        rospy.loginfo("Map Manager initialized")
        rospy.loginfo(f"Session ID: {self.session_id}")
        rospy.loginfo(f"Maps directory: {self.session_dir}")

    def get_robot_pose(self):
        """获取机器人位姿"""
        try:
            transform = self.tf_buffer.lookup_transform('camera_init', 'aft_mapped', 
                                                      rospy.Time(0), rospy.Duration(0.1))
            
            pos = transform.transform.translation
            rot = transform.transform.rotation
            _, _, yaw = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
            
            self.robot_pose = {
                'x': pos.x,
                'y': pos.y,
                'z': pos.z,
                'yaw': yaw,
                'timestamp': rospy.Time.now().to_sec()
            }
            
        except Exception as e:
            rospy.logwarn_throttle(10.0, f"Failed to get robot pose: {e}")

    def cloud_callback(self, cloud_msg):
        """3D点云回调"""
        if not self.is_collecting:
            return
            
        with self.data_lock:
            try:
                # 获取机器人位姿
                self.get_robot_pose()
                
                # 转换点云数据
                points = list(pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True))
                
                if len(points) < 10:
                    return
                
                # 过滤和处理点云
                filtered_points = []
                for point in points:
                    x, y, z = point
                    
                    # 基本过滤
                    distance = np.sqrt(x*x + y*y)
                    if distance <= 25.0 and -2.0 <= z <= 5.0:
                        filtered_points.append([x, y, z])
                
                if filtered_points:
                    self.accumulated_3d_points.extend(filtered_points)
                    self.stats['total_3d_points'] += len(filtered_points)
                
                # 限制内存使用
                if len(self.accumulated_3d_points) > 1000000:  # 100万点
                    # 体素降采样
                    self.accumulated_3d_points = self.voxel_downsample(self.accumulated_3d_points, 0.05)
                
                rospy.loginfo_throttle(10.0, 
                    f"3D Points collected: {len(self.accumulated_3d_points)}")
                
            except Exception as e:
                rospy.logerr(f"Error in cloud callback: {e}")

    def map_callback(self, map_msg):
        """2D地图回调"""
        with self.data_lock:
            self.current_2d_map = map_msg
            self.stats['map_updates'] += 1

    def voxel_downsample(self, points, voxel_size):
        """体素降采样"""
        if not points:
            return points
        
        voxel_dict = {}
        for point in points:
            vx = int(point[0] / voxel_size)
            vy = int(point[1] / voxel_size)
            vz = int(point[2] / voxel_size)
            
            voxel_key = (vx, vy, vz)
            if voxel_key not in voxel_dict:
                voxel_dict[voxel_key] = []
            voxel_dict[voxel_key].append(point)
        
        # 每个体素取中心点
        downsampled_points = []
        for voxel_points in voxel_dict.values():
            if voxel_points:
                center = np.mean(voxel_points, axis=0)
                downsampled_points.append(center.tolist())
        
        return downsampled_points

    def start_collection(self, req):
        """开始收集地图数据"""
        try:
            if self.is_collecting:
                return TriggerResponse(success=False, message="Already collecting")
            
            self.is_collecting = True
            self.collection_start_time = rospy.Time.now()
            self.accumulated_3d_points = []
            self.stats['total_3d_points'] = 0
            
            rospy.loginfo("Started map collection")
            return TriggerResponse(success=True, message=f"Collection started. Session: {self.session_id}")
            
        except Exception as e:
            return TriggerResponse(success=False, message=str(e))

    def stop_collection(self, req):
        """停止收集地图数据"""
        try:
            if not self.is_collecting:
                return TriggerResponse(success=False, message="Not currently collecting")
            
            self.is_collecting = False
            
            if self.collection_start_time:
                self.stats['collection_time'] = (rospy.Time.now() - self.collection_start_time).to_sec()
            
            rospy.loginfo("Stopped map collection")
            return TriggerResponse(success=True, 
                                 message=f"Collection stopped. Points: {len(self.accumulated_3d_points)}")
            
        except Exception as e:
            return TriggerResponse(success=False, message=str(e))

    def save_maps(self, req):
        """保存地图到文件"""
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            # 保存2D地图
            if self.current_2d_map:
                map_file = os.path.join(self.session_dir, f"2d_map_{timestamp}")
                self.save_2d_map_to_file(self.current_2d_map, map_file)
                rospy.loginfo(f"2D map saved: {map_file}")
            
            # 保存3D点云
            if self.accumulated_3d_points:
                pcd_file = os.path.join(self.session_dir, f"3d_map_{timestamp}.pcd")
                self.save_pcd_file(self.accumulated_3d_points, pcd_file)
                rospy.loginfo(f"3D map saved: {pcd_file}")
            
            # 保存统计信息
            stats_file = os.path.join(self.session_dir, f"stats_{timestamp}.json")
            with open(stats_file, 'w') as f:
                json.dump(self.stats, f, indent=2)
            
            return TriggerResponse(success=True, message=f"Maps saved to: {self.session_dir}")
            
        except Exception as e:
            return TriggerResponse(success=False, message=str(e))

    def save_2d_map_to_file(self, map_msg, filename_base):
        """保存2D地图为PGM和YAML格式"""
        try:
            yaml_file = filename_base + ".yaml"
            pgm_file = filename_base + ".pgm"
            
            # 保存YAML文件
            with open(yaml_file, 'w') as f:
                f.write(f"image: {os.path.basename(pgm_file)}\n")
                f.write(f"resolution: {map_msg.info.resolution}\n")
                f.write(f"origin: [{map_msg.info.origin.position.x}, {map_msg.info.origin.position.y}, 0.0]\n")
                f.write("negate: 0\n")
                f.write("occupied_thresh: 0.65\n")
                f.write("free_thresh: 0.196\n")
            
            # 保存PGM文件
            with open(pgm_file, 'wb') as f:
                f.write(b"P5\n")
                f.write(f"{map_msg.info.width} {map_msg.info.height}\n".encode())
                f.write(b"255\n")
                
                for y in range(map_msg.info.height):
                    for x in range(map_msg.info.width):
                        i = x + (map_msg.info.height - y - 1) * map_msg.info.width
                        if map_msg.data[i] == -1:
                            f.write(bytes([205]))  # 未知
                        elif map_msg.data[i] == 0:
                            f.write(bytes([254]))  # 空闲
                        else:
                            f.write(bytes([0]))    # 占用
            
        except Exception as e:
            rospy.logerr(f"Error saving 2D map: {e}")

    def save_pcd_file(self, points, filepath):
        """保存PCD文件"""
        try:
            with open(filepath, 'w') as f:
                f.write("# .PCD v0.7 - Point Cloud Data file format\n")
                f.write("VERSION 0.7\n")
                f.write("FIELDS x y z\n")
                f.write("SIZE 4 4 4\n")
                f.write("TYPE F F F\n")
                f.write("COUNT 1 1 1\n")
                f.write(f"WIDTH {len(points)}\n")
                f.write("HEIGHT 1\n")
                f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                f.write(f"POINTS {len(points)}\n")
                f.write("DATA ascii\n")
                
                for point in points:
                    f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")
                    
        except Exception as e:
            rospy.logerr(f"Error saving PCD file: {e}")

    def send_2d_map(self, req):
        """发送2D地图到QT上位机"""
        try:
            if not self.current_2d_map:
                return TriggerResponse(success=False, message="No 2D map available")
            
            # 准备2D地图数据
            map_data = {
                'type': '2d_map',
                'timestamp': rospy.Time.now().to_sec(),
                'session_id': self.session_id,
                'width': self.current_2d_map.info.width,
                'height': self.current_2d_map.info.height,
                'resolution': self.current_2d_map.info.resolution,
                'origin': {
                    'x': self.current_2d_map.info.origin.position.x,
                    'y': self.current_2d_map.info.origin.position.y,
                    'z': self.current_2d_map.info.origin.position.z
                },
                'data': self.current_2d_map.data
            }
            
            # 压缩数据
            if self.enable_compression:
                json_str = json.dumps(map_data)
                compressed_data = gzip.compress(json_str.encode('utf-8'), compresslevel=self.compression_level)
                encoded_data = base64.b64encode(compressed_data).decode('utf-8')
                
                final_data = {
                    'compressed': True,
                    'data': encoded_data
                }
            else:
                final_data = map_data
            
            # 发布数据
            self.map_data_pub.publish(String(data=json.dumps(final_data)))
            self.stats['data_sent'] += 1
            
            rospy.loginfo("2D map sent to QT client")
            return TriggerResponse(success=True, message="2D map sent successfully")
            
        except Exception as e:
            return TriggerResponse(success=False, message=str(e))

    def send_3d_map(self, req):
        """发送3D地图到QT上位机（分包传输）"""
        try:
            if not self.accumulated_3d_points:
                return TriggerResponse(success=False, message="No 3D map available")
            
            # 分包发送3D点云
            total_points = len(self.accumulated_3d_points)
            total_chunks = (total_points + self.max_points_per_chunk - 1) // self.max_points_per_chunk
            
            for chunk_id in range(total_chunks):
                start_idx = chunk_id * self.max_points_per_chunk
                end_idx = min((chunk_id + 1) * self.max_points_per_chunk, total_points)
                chunk_points = self.accumulated_3d_points[start_idx:end_idx]
                
                # 准备分包数据
                chunk_data = {
                    'type': '3d_map_chunk',
                    'timestamp': rospy.Time.now().to_sec(),
                    'session_id': self.session_id,
                    'chunk_id': chunk_id,
                    'total_chunks': total_chunks,
                    'points_in_chunk': len(chunk_points),
                    'total_points': total_points,
                    'points': chunk_points
                }
                
                # 压缩分包数据
                if self.enable_compression:
                    json_str = json.dumps(chunk_data)
                    compressed_data = gzip.compress(json_str.encode('utf-8'), compresslevel=self.compression_level)
                    encoded_data = base64.b64encode(compressed_data).decode('utf-8')
                    
                    final_data = {
                        'compressed': True,
                        'data': encoded_data
                    }
                else:
                    final_data = chunk_data
                
                # 发布分包数据
                self.map_data_pub.publish(String(data=json.dumps(final_data)))
                self.stats['data_sent'] += 1
                
                # 短暂延迟避免网络拥塞
                rospy.sleep(0.1)
                
                rospy.loginfo(f"3D map chunk {chunk_id+1}/{total_chunks} sent")
            
            rospy.loginfo(f"3D map sent in {total_chunks} chunks")
            return TriggerResponse(success=True, 
                                 message=f"3D map sent in {total_chunks} chunks")
            
        except Exception as e:
            return TriggerResponse(success=False, message=str(e))

    def publish_status(self, event):
        """发布状态信息"""
        status = {
            'session_id': self.session_id,
            'is_collecting': self.is_collecting,
            'total_3d_points': len(self.accumulated_3d_points),
            'has_2d_map': self.current_2d_map is not None,
            'robot_pose': self.robot_pose,
            'stats': self.stats
        }
        
        self.status_pub.publish(String(data=json.dumps(status)))

    def run(self):
        """主循环"""
        rospy.loginfo("Map Manager ready")
        rospy.loginfo("Services available:")
        rospy.loginfo("  - /start_map_collection")
        rospy.loginfo("  - /stop_map_collection")
        rospy.loginfo("  - /save_maps")
        rospy.loginfo("  - /send_2d_map")
        rospy.loginfo("  - /send_3d_map")
        rospy.spin()

if __name__ == '__main__':
    try:
        manager = MapManager()
        manager.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Map Manager shutdown")
