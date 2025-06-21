#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS WebSocket Map Server
为安卓客户端提供实时地图数据传输服务
服务器IP: 192.168.200.216:8000
"""

import rospy
import json
import gzip
import base64
import threading
import time
from datetime import datetime
import numpy as np

# WebSocket相关
import asyncio
import websockets
from websockets.server import serve

# ROS消息类型
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class WebSocketMapServer:
    def __init__(self):
        rospy.init_node('websocket_map_server', anonymous=True)
        
        # 服务器配置
        self.host = "192.168.200.216"
        self.port = 8000
        
        # 数据存储
        self.current_2d_map = None
        self.current_3d_points = []
        self.robot_pose = None
        self.connected_clients = set()
        
        # 数据压缩配置
        self.enable_compression = True
        self.compression_level = 6
        self.max_3d_points_per_chunk = 5000
        
        # 数据采样配置
        self.map_update_interval = 2.0  # 2D地图更新间隔(秒)
        self.pointcloud_update_interval = 1.0  # 3D点云更新间隔(秒)
        self.max_3d_points_buffer = 50000  # 最大3D点云缓存
        
        # 线程锁
        self.data_lock = threading.Lock()
        
        # ROS订阅器
        self.map_sub = rospy.Subscriber('/enhanced_2d_map', OccupancyGrid, self.map_callback)
        self.cloud_sub = rospy.Subscriber('/cloud_registered', PointCloud2, self.cloud_callback)
        self.pose_sub = rospy.Subscriber('/aft_mapped_to_init', PoseStamped, self.pose_callback)
        
        # 统计信息
        self.stats = {
            'clients_connected': 0,
            'maps_sent': 0,
            'pointclouds_sent': 0,
            'data_compressed': 0,
            'start_time': time.time()
        }
        
        rospy.loginfo("WebSocket Map Server initialized")
        rospy.loginfo(f"Server will start at ws://{self.host}:{self.port}")
        
    def map_callback(self, msg):
        """2D地图回调"""
        with self.data_lock:
            self.current_2d_map = msg
            rospy.logdebug("2D map updated")
    
    def cloud_callback(self, msg):
        """3D点云回调"""
        try:
            # 采样点云数据
            points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            
            # 降采样以减少数据量
            if len(points) > 1000:
                step = len(points) // 1000
                points = points[::step]
            
            with self.data_lock:
                # 添加新点云数据
                self.current_3d_points.extend([[p[0], p[1], p[2]] for p in points])
                
                # 限制缓存大小
                if len(self.current_3d_points) > self.max_3d_points_buffer:
                    # 保留最新的点
                    self.current_3d_points = self.current_3d_points[-self.max_3d_points_buffer:]
                
                rospy.logdebug(f"3D points updated: {len(self.current_3d_points)} total")
                
        except Exception as e:
            rospy.logerr(f"Error processing point cloud: {e}")
    
    def pose_callback(self, msg):
        """机器人位姿回调"""
        with self.data_lock:
            self.robot_pose = {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z,
                'qx': msg.pose.orientation.x,
                'qy': msg.pose.orientation.y,
                'qz': msg.pose.orientation.z,
                'qw': msg.pose.orientation.w,
                'timestamp': msg.header.stamp.to_sec()
            }
    
    def compress_data(self, data):
        """压缩数据"""
        if not self.enable_compression:
            return data, False
        
        try:
            json_str = json.dumps(data, separators=(',', ':'))
            compressed = gzip.compress(json_str.encode('utf-8'), compresslevel=self.compression_level)
            encoded = base64.b64encode(compressed).decode('utf-8')
            self.stats['data_compressed'] += 1
            return encoded, True
        except Exception as e:
            rospy.logerr(f"Compression error: {e}")
            return data, False
    
    def prepare_2d_map_data(self):
        """准备2D地图数据"""
        with self.data_lock:
            if self.current_2d_map is None:
                return None
            
            map_data = {
                'type': '2d_map',
                'timestamp': time.time(),
                'width': self.current_2d_map.info.width,
                'height': self.current_2d_map.info.height,
                'resolution': self.current_2d_map.info.resolution,
                'origin': {
                    'x': self.current_2d_map.info.origin.position.x,
                    'y': self.current_2d_map.info.origin.position.y,
                    'theta': 0.0  # 简化处理
                },
                'data': list(self.current_2d_map.data),
                'robot_pose': self.robot_pose
            }
            
            return map_data
    
    def prepare_3d_pointcloud_chunks(self):
        """准备3D点云数据块"""
        with self.data_lock:
            if not self.current_3d_points:
                return []
            
            points_copy = self.current_3d_points.copy()
        
        # 分块处理
        chunks = []
        total_points = len(points_copy)
        total_chunks = (total_points + self.max_3d_points_per_chunk - 1) // self.max_3d_points_per_chunk
        
        for chunk_id in range(total_chunks):
            start_idx = chunk_id * self.max_3d_points_per_chunk
            end_idx = min((chunk_id + 1) * self.max_3d_points_per_chunk, total_points)
            chunk_points = points_copy[start_idx:end_idx]
            
            chunk_data = {
                'type': '3d_pointcloud_chunk',
                'timestamp': time.time(),
                'chunk_id': chunk_id,
                'total_chunks': total_chunks,
                'points_in_chunk': len(chunk_points),
                'total_points': total_points,
                'points': chunk_points,
                'robot_pose': self.robot_pose
            }
            
            chunks.append(chunk_data)
        
        return chunks
    
    async def handle_client_message(self, websocket, message):
        """处理客户端消息"""
        try:
            request = json.loads(message)
            request_type = request.get('type', '')
            
            rospy.loginfo(f"Received request: {request_type} from {websocket.remote_address}")
            
            if request_type == 'request_2d_map':
                # 发送2D地图
                map_data = self.prepare_2d_map_data()
                if map_data:
                    compressed_data, is_compressed = self.compress_data(map_data)
                    
                    response = {
                        'type': 'request_2d_map',
                        'compressed': is_compressed,
                        'data': compressed_data if is_compressed else map_data
                    }
                    
                    await websocket.send(json.dumps(response))
                    self.stats['maps_sent'] += 1
                    rospy.loginfo("2D map sent to client")
                else:
                    await websocket.send(json.dumps({
                        'type': 'error',
                        'message': 'No 2D map available'
                    }))
            
            elif request_type == 'request_3d_pointcloud':
                # 发送3D点云
                chunks = self.prepare_3d_pointcloud_chunks()
                if chunks:
                    for chunk in chunks:
                        compressed_data, is_compressed = self.compress_data(chunk)
                        
                        response = {
                            'type': 'response_3d_pointcloud_chunk',
                            'compressed': is_compressed,
                            'data': compressed_data if is_compressed else chunk
                        }
                        
                        await websocket.send(json.dumps(response))
                        await asyncio.sleep(0.01)  # 小延迟避免网络拥塞
                    
                    self.stats['pointclouds_sent'] += 1
                    rospy.loginfo(f"3D pointcloud sent in {len(chunks)} chunks")
                else:
                    await websocket.send(json.dumps({
                        'type': 'error',
                        'message': 'No 3D pointcloud available'
                    }))
            
            elif request_type == 'request_robot_pose':
                # 发送机器人位姿
                response = {
                    'type': 'response_robot_pose',
                    'data': self.robot_pose,
                    'timestamp': time.time()
                }
                await websocket.send(json.dumps(response))
            
            elif request_type == 'request_server_stats':
                # 发送服务器统计信息
                current_stats = self.stats.copy()
                current_stats['clients_connected'] = len(self.connected_clients)
                current_stats['uptime'] = time.time() - current_stats['start_time']
                
                response = {
                    'type': 'response_server_stats',
                    'data': current_stats,
                    'timestamp': time.time()
                }
                await websocket.send(json.dumps(response))
            
            else:
                await websocket.send(json.dumps({
                    'type': 'error',
                    'message': f'Unknown request type: {request_type}'
                }))
                
        except json.JSONDecodeError:
            await websocket.send(json.dumps({
                'type': 'error',
                'message': 'Invalid JSON format'
            }))
        except Exception as e:
            rospy.logerr(f"Error handling client message: {e}")
            await websocket.send(json.dumps({
                'type': 'error',
                'message': str(e)
            }))
    
    async def handle_client(self, websocket, path):
        """处理客户端连接"""
        client_addr = websocket.remote_address
        rospy.loginfo(f"New client connected: {client_addr}")
        
        self.connected_clients.add(websocket)
        
        try:
            # 发送欢迎消息
            welcome_msg = {
                'type': 'welcome',
                'message': 'Connected to ROS WebSocket Map Server',
                'server_time': time.time(),
                'available_requests': [
                    'request_2d_map',
                    'request_3d_pointcloud', 
                    'request_robot_pose',
                    'request_server_stats'
                ]
            }
            await websocket.send(json.dumps(welcome_msg))
            
            # 处理客户端消息
            async for message in websocket:
                await self.handle_client_message(websocket, message)
                
        except websockets.exceptions.ConnectionClosed:
            rospy.loginfo(f"Client disconnected: {client_addr}")
        except Exception as e:
            rospy.logerr(f"Error with client {client_addr}: {e}")
        finally:
            self.connected_clients.discard(websocket)
    
    async def start_server(self):
        """启动WebSocket服务器"""
        rospy.loginfo(f"Starting WebSocket server on {self.host}:{self.port}")
        
        async with serve(self.handle_client, self.host, self.port):
            rospy.loginfo("WebSocket server started successfully")
            
            # 保持服务器运行
            while not rospy.is_shutdown():
                await asyncio.sleep(1)
    
    def run(self):
        """运行服务器"""
        try:
            # 在新线程中运行asyncio事件循环
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self.start_server())
        except Exception as e:
            rospy.logerr(f"Server error: {e}")
        finally:
            rospy.loginfo("WebSocket server stopped")

def main():
    try:
        server = WebSocketMapServer()
        server.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("WebSocket Map Server shutdown")
    except KeyboardInterrupt:
        rospy.loginfo("WebSocket Map Server interrupted")

if __name__ == '__main__':
    main()
