#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
直接WebSocket发送器 - 强制发送数据到QT上位机
"""

import rospy
import json
import gzip
import base64
import socket
import struct
import hashlib
import time
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger, TriggerResponse
import sensor_msgs.point_cloud2 as pc2

class DirectWebSocketSender:
    def __init__(self):
        rospy.init_node('direct_websocket_sender', anonymous=True)
        
        # QT上位机参数
        self.qt_host = rospy.get_param('~qt_host', '192.168.200.164')
        self.qt_port = rospy.get_param('~qt_port', 8001)
        self.enable_compression = rospy.get_param('~enable_compression', True)
        
        # 数据缓存
        self.current_2d_map = None
        
        # 订阅器
        self.map_sub = rospy.Subscriber('/enhanced_2d_map', OccupancyGrid, 
                                      self.map_callback, queue_size=1)
        
        # 发布器
        self.status_pub = rospy.Publisher('/websocket_sender_status', String, queue_size=1)
        
        # 服务
        self.send_2d_ws_srv = rospy.Service('send_2d_websocket', Trigger, self.send_2d_websocket)
        self.send_3d_ws_srv = rospy.Service('send_3d_websocket', Trigger, self.send_3d_websocket)
        self.test_raw_send_srv = rospy.Service('test_raw_send', Trigger, self.test_raw_send)
        
        rospy.loginfo("Direct WebSocket Sender initialized")
        rospy.loginfo(f"Target QT server: {self.qt_host}:{self.qt_port}")

    def map_callback(self, map_msg):
        """2D地图回调"""
        self.current_2d_map = map_msg

    def create_websocket_key(self):
        """创建WebSocket密钥"""
        import base64
        import os
        key = base64.b64encode(os.urandom(16)).decode('utf-8')
        return key

    def create_websocket_accept(self, key):
        """创建WebSocket接受密钥"""
        WEBSOCKET_MAGIC_STRING = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
        accept = base64.b64encode(
            hashlib.sha1((key + WEBSOCKET_MAGIC_STRING).encode()).digest()
        ).decode('utf-8')
        return accept

    def send_raw_websocket_frame(self, data, opcode=1):
        """发送原始WebSocket帧"""
        try:
            # 创建TCP连接
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(10.0)
            sock.connect((self.qt_host, self.qt_port))
            
            # 发送WebSocket握手
            key = self.create_websocket_key()
            handshake = (
                f"GET / HTTP/1.1\r\n"
                f"Host: {self.qt_host}:{self.qt_port}\r\n"
                f"Upgrade: websocket\r\n"
                f"Connection: Upgrade\r\n"
                f"Sec-WebSocket-Key: {key}\r\n"
                f"Sec-WebSocket-Version: 13\r\n"
                f"Origin: http://{self.qt_host}:{self.qt_port}\r\n"
                f"User-Agent: FAST-LIVO-Direct-Sender/1.0\r\n"
                f"\r\n"
            )
            
            sock.send(handshake.encode())
            rospy.loginfo("WebSocket handshake sent")
            
            # 等待响应（短时间）
            try:
                response = sock.recv(1024).decode()
                rospy.loginfo(f"Received response: {response[:200]}...")
            except:
                rospy.logwarn("No handshake response, sending data anyway")
            
            # 直接发送WebSocket数据帧
            if isinstance(data, str):
                payload = data.encode('utf-8')
            else:
                payload = data
            
            # 构造WebSocket帧
            frame = self.create_websocket_frame(payload, opcode)
            sock.send(frame)
            
            rospy.loginfo(f"WebSocket frame sent: {len(payload)} bytes")
            
            # 尝试接收响应
            try:
                response = sock.recv(1024)
                rospy.loginfo(f"Data response received: {len(response)} bytes")
            except:
                rospy.loginfo("No data response (normal for one-way send)")
            
            sock.close()
            return True
            
        except Exception as e:
            rospy.logerr(f"Error sending WebSocket frame: {e}")
            return False

    def create_websocket_frame(self, payload, opcode=1):
        """创建WebSocket数据帧"""
        # WebSocket帧格式
        frame = bytearray()
        
        # 第一个字节：FIN=1, RSV=000, opcode
        frame.append(0x80 | opcode)
        
        # 负载长度
        payload_len = len(payload)
        if payload_len < 126:
            frame.append(0x80 | payload_len)  # MASK=1
        elif payload_len < 65536:
            frame.append(0x80 | 126)  # MASK=1
            frame.extend(struct.pack('>H', payload_len))
        else:
            frame.append(0x80 | 127)  # MASK=1
            frame.extend(struct.pack('>Q', payload_len))
        
        # 掩码密钥（4字节）
        import os
        mask_key = os.urandom(4)
        frame.extend(mask_key)
        
        # 掩码负载
        masked_payload = bytearray()
        for i, byte in enumerate(payload):
            masked_payload.append(byte ^ mask_key[i % 4])
        
        frame.extend(masked_payload)
        return bytes(frame)

    def compress_data(self, data):
        """压缩数据"""
        if not self.enable_compression:
            return data
        
        try:
            json_str = json.dumps(data)
            compressed_data = gzip.compress(json_str.encode('utf-8'), compresslevel=6)
            encoded_data = base64.b64encode(compressed_data).decode('utf-8')
            
            return {
                'compressed': True,
                'original_size': len(json_str),
                'compressed_size': len(encoded_data),
                'compression_ratio': f"{len(encoded_data)/len(json_str)*100:.1f}%",
                'data': encoded_data
            }
        except Exception as e:
            rospy.logerr(f"Compression error: {e}")
            return data

    def send_2d_websocket(self, req):
        """通过WebSocket发送2D地图"""
        try:
            if not self.current_2d_map:
                return TriggerResponse(success=False, message="No 2D map available")
            
            # 准备2D地图数据
            map_data = {
                'type': '2d_map',
                'timestamp': time.time(),
                'source': 'FAST-LIVO-Direct',
                'map_info': {
                    'width': self.current_2d_map.info.width,
                    'height': self.current_2d_map.info.height,
                    'resolution': self.current_2d_map.info.resolution,
                    'origin': {
                        'x': self.current_2d_map.info.origin.position.x,
                        'y': self.current_2d_map.info.origin.position.y,
                        'z': self.current_2d_map.info.origin.position.z
                    }
                },
                'data': self.current_2d_map.data
            }
            
            # 压缩数据
            final_data = self.compress_data(map_data)
            
            # 发送WebSocket帧
            json_data = json.dumps(final_data)
            
            if self.send_raw_websocket_frame(json_data):
                rospy.loginfo("✅ 2D map sent via direct WebSocket")
                return TriggerResponse(success=True, message="2D map sent via direct WebSocket")
            else:
                return TriggerResponse(success=False, message="Failed to send 2D map")
            
        except Exception as e:
            rospy.logerr(f"Error sending 2D map: {e}")
            return TriggerResponse(success=False, message=str(e))

    def send_3d_websocket(self, req):
        """通过WebSocket发送3D地图"""
        try:
            # 收集当前的点云数据
            rospy.loginfo("Collecting 3D point cloud data...")
            cloud_data = rospy.wait_for_message('/cloud_registered', PointCloud2, timeout=5.0)
            points = list(pc2.read_points(cloud_data, field_names=("x", "y", "z"), skip_nans=True))
            
            if len(points) < 10:
                return TriggerResponse(success=False, message="Insufficient 3D data")
            
            # 过滤点云
            filtered_points = []
            for point in points:
                x, y, z = point
                distance = np.sqrt(x*x + y*y)
                if distance <= 25.0 and -2.0 <= z <= 5.0:
                    filtered_points.append([float(x), float(y), float(z)])
            
            if not filtered_points:
                return TriggerResponse(success=False, message="No valid 3D points after filtering")
            
            # 限制点数以避免数据过大
            if len(filtered_points) > 50000:
                # 随机采样
                import random
                filtered_points = random.sample(filtered_points, 50000)
            
            # 准备3D地图数据
            map_3d_data = {
                'type': '3d_map',
                'timestamp': time.time(),
                'source': 'FAST-LIVO-Direct',
                'point_count': len(filtered_points),
                'points': filtered_points
            }
            
            # 压缩数据
            final_data = self.compress_data(map_3d_data)
            
            # 发送WebSocket帧
            json_data = json.dumps(final_data)
            
            if self.send_raw_websocket_frame(json_data):
                rospy.loginfo(f"✅ 3D map sent via direct WebSocket ({len(filtered_points)} points)")
                return TriggerResponse(success=True, 
                                     message=f"3D map sent via direct WebSocket ({len(filtered_points)} points)")
            else:
                return TriggerResponse(success=False, message="Failed to send 3D map")
            
        except Exception as e:
            rospy.logerr(f"Error sending 3D map: {e}")
            return TriggerResponse(success=False, message=str(e))

    def test_raw_send(self, req):
        """测试原始数据发送"""
        try:
            test_data = {
                'type': 'test_message',
                'timestamp': time.time(),
                'source': 'FAST-LIVO-Test',
                'message': 'Hello QT from ROS!',
                'data': [1, 2, 3, 4, 5]
            }
            
            json_data = json.dumps(test_data)
            
            if self.send_raw_websocket_frame(json_data):
                rospy.loginfo("✅ Test message sent successfully")
                return TriggerResponse(success=True, message="Test message sent successfully")
            else:
                return TriggerResponse(success=False, message="Failed to send test message")
            
        except Exception as e:
            return TriggerResponse(success=False, message=str(e))

    def run(self):
        """主循环"""
        rospy.loginfo("Direct WebSocket Sender ready")
        rospy.loginfo("Services available:")
        rospy.loginfo("  - /send_2d_websocket")
        rospy.loginfo("  - /send_3d_websocket")
        rospy.loginfo("  - /test_raw_send")
        rospy.spin()

if __name__ == '__main__':
    try:
        sender = DirectWebSocketSender()
        sender.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Direct WebSocket Sender shutdown")
