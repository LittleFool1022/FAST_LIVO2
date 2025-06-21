#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
QT上位机通信桥接器
处理与QT上位机的WebSocket通信，包括地图数据传输
"""

import rospy
import json
import gzip
import base64
import threading
import websocket
import time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped

class QTCommunicationBridge:
    def __init__(self):
        rospy.init_node('qt_communication_bridge', anonymous=True)
        
        # 参数配置
        self.qt_server_ip = rospy.get_param('~qt_server_ip', '192.168.200.164')
        self.qt_server_port = rospy.get_param('~qt_server_port', 8001)
        self.reconnect_interval = rospy.get_param('~reconnect_interval', 5.0)
        self.heartbeat_interval = rospy.get_param('~heartbeat_interval', 30.0)
        
        # WebSocket连接
        self.ws = None
        self.is_connected = False
        self.connection_thread = None
        self.heartbeat_thread = None
        
        # 数据缓存
        self.received_3d_chunks = {}
        self.current_session_id = None
        
        # 线程锁
        self.connection_lock = threading.Lock()
        
        # ROS订阅器
        self.map_data_sub = rospy.Subscriber('/map_data_for_qt', String, 
                                           self.map_data_callback, queue_size=10)
        self.status_sub = rospy.Subscriber('/map_manager_status', String, 
                                         self.status_callback, queue_size=1)
        
        # ROS发布器
        self.qt_command_pub = rospy.Publisher('/qt_commands', String, queue_size=1)
        self.connection_status_pub = rospy.Publisher('/qt_connection_status', String, queue_size=1)
        
        # ROS服务代理
        self.start_collection_srv = None
        self.stop_collection_srv = None
        self.save_maps_srv = None
        self.send_2d_map_srv = None
        self.send_3d_map_srv = None
        
        # 定时器
        self.status_timer = rospy.Timer(rospy.Duration(5.0), self.publish_connection_status)
        
        rospy.loginfo("QT Communication Bridge initialized")
        rospy.loginfo(f"Target QT server: {self.qt_server_ip}:{self.qt_server_port}")
        
        # 等待服务可用
        self.wait_for_services()
        
        # 启动连接
        self.start_connection()

    def wait_for_services(self):
        """等待地图管理器服务可用"""
        services = [
            'start_map_collection',
            'stop_map_collection',
            'save_maps',
            'send_2d_map',
            'send_3d_map'
        ]
        
        rospy.loginfo("Waiting for map manager services...")
        for service in services:
            try:
                rospy.wait_for_service(service, timeout=10.0)
                rospy.loginfo(f"✅ {service} service available")
            except rospy.ROSException:
                rospy.logwarn(f"⚠️ {service} service not available")
        
        # 创建服务代理
        try:
            self.start_collection_srv = rospy.ServiceProxy('start_map_collection', Trigger)
            self.stop_collection_srv = rospy.ServiceProxy('stop_map_collection', Trigger)
            self.save_maps_srv = rospy.ServiceProxy('save_maps', Trigger)
            self.send_2d_map_srv = rospy.ServiceProxy('send_2d_map', Trigger)
            self.send_3d_map_srv = rospy.ServiceProxy('send_3d_map', Trigger)
            rospy.loginfo("Service proxies created")
        except Exception as e:
            rospy.logerr(f"Failed to create service proxies: {e}")

    def start_connection(self):
        """启动WebSocket连接"""
        if self.connection_thread and self.connection_thread.is_alive():
            return
        
        self.connection_thread = threading.Thread(target=self.connection_worker)
        self.connection_thread.daemon = True
        self.connection_thread.start()

    def connection_worker(self):
        """连接工作线程"""
        while not rospy.is_shutdown():
            try:
                if not self.is_connected:
                    self.connect_to_qt()
                time.sleep(self.reconnect_interval)
            except Exception as e:
                rospy.logerr(f"Connection worker error: {e}")
                time.sleep(self.reconnect_interval)

    def connect_to_qt(self):
        """连接到QT上位机"""
        try:
            with self.connection_lock:
                if self.is_connected:
                    return
                
                ws_url = f"ws://{self.qt_server_ip}:{self.qt_server_port}"
                rospy.loginfo(f"Connecting to QT server: {ws_url}")
                
                self.ws = websocket.WebSocketApp(
                    ws_url,
                    on_open=self.on_open,
                    on_message=self.on_message,
                    on_error=self.on_error,
                    on_close=self.on_close
                )
                
                self.ws.run_forever()
                
        except Exception as e:
            rospy.logerr(f"Failed to connect to QT server: {e}")

    def on_open(self, ws):
        """WebSocket连接打开"""
        rospy.loginfo("Connected to QT server")
        self.is_connected = True
        
        # 发送连接确认
        welcome_msg = {
            'type': 'connection_established',
            'timestamp': rospy.Time.now().to_sec(),
            'ros_node': 'qt_communication_bridge'
        }
        self.send_to_qt(welcome_msg)
        
        # 启动心跳
        self.start_heartbeat()

    def on_message(self, ws, message):
        """接收QT消息"""
        try:
            data = json.loads(message)
            self.handle_qt_command(data)
        except Exception as e:
            rospy.logerr(f"Error processing QT message: {e}")

    def on_error(self, ws, error):
        """WebSocket错误"""
        rospy.logerr(f"WebSocket error: {error}")

    def on_close(self, ws, close_status_code, close_msg):
        """WebSocket连接关闭"""
        rospy.logwarn("Disconnected from QT server")
        self.is_connected = False
        self.stop_heartbeat()

    def handle_qt_command(self, data):
        """处理QT命令"""
        try:
            command_type = data.get('type', '')
            
            if command_type == 'start_mapping':
                response = self.start_collection_srv()
                self.send_response_to_qt('start_mapping', response.success, response.message)
                
            elif command_type == 'stop_mapping':
                response = self.stop_collection_srv()
                self.send_response_to_qt('stop_mapping', response.success, response.message)
                
            elif command_type == 'save_maps':
                response = self.save_maps_srv()
                self.send_response_to_qt('save_maps', response.success, response.message)
                
            elif command_type == 'request_2d_map':
                response = self.send_2d_map_srv()
                self.send_response_to_qt('request_2d_map', response.success, response.message)
                
            elif command_type == 'request_3d_map':
                response = self.send_3d_map_srv()
                self.send_response_to_qt('request_3d_map', response.success, response.message)
                
            elif command_type == 'heartbeat':
                self.send_heartbeat_response()
                
            else:
                rospy.logwarn(f"Unknown command type: {command_type}")
                
        except Exception as e:
            rospy.logerr(f"Error handling QT command: {e}")

    def send_response_to_qt(self, command_type, success, message):
        """发送响应到QT"""
        response = {
            'type': f'{command_type}_response',
            'success': success,
            'message': message,
            'timestamp': rospy.Time.now().to_sec()
        }
        self.send_to_qt(response)

    def map_data_callback(self, msg):
        """地图数据回调 - 转发到QT"""
        try:
            if self.is_connected:
                # 直接转发地图数据到QT
                self.ws.send(msg.data)
                rospy.loginfo("Map data forwarded to QT")
            else:
                rospy.logwarn("Not connected to QT, map data not sent")
                
        except Exception as e:
            rospy.logerr(f"Error forwarding map data: {e}")

    def status_callback(self, msg):
        """状态回调"""
        try:
            if self.is_connected:
                # 转发状态信息到QT
                status_data = json.loads(msg.data)
                qt_status = {
                    'type': 'ros_status',
                    'data': status_data,
                    'timestamp': rospy.Time.now().to_sec()
                }
                self.send_to_qt(qt_status)
                
        except Exception as e:
            rospy.logerr(f"Error forwarding status: {e}")

    def send_to_qt(self, data):
        """发送数据到QT"""
        try:
            if self.is_connected and self.ws:
                message = json.dumps(data)
                self.ws.send(message)
            else:
                rospy.logwarn("Cannot send to QT: not connected")
                
        except Exception as e:
            rospy.logerr(f"Error sending to QT: {e}")

    def start_heartbeat(self):
        """启动心跳"""
        if self.heartbeat_thread and self.heartbeat_thread.is_alive():
            return
        
        self.heartbeat_thread = threading.Thread(target=self.heartbeat_worker)
        self.heartbeat_thread.daemon = True
        self.heartbeat_thread.start()

    def heartbeat_worker(self):
        """心跳工作线程"""
        while self.is_connected and not rospy.is_shutdown():
            try:
                heartbeat = {
                    'type': 'heartbeat',
                    'timestamp': rospy.Time.now().to_sec()
                }
                self.send_to_qt(heartbeat)
                time.sleep(self.heartbeat_interval)
            except Exception as e:
                rospy.logerr(f"Heartbeat error: {e}")
                break

    def send_heartbeat_response(self):
        """发送心跳响应"""
        response = {
            'type': 'heartbeat_response',
            'timestamp': rospy.Time.now().to_sec()
        }
        self.send_to_qt(response)

    def stop_heartbeat(self):
        """停止心跳"""
        if self.heartbeat_thread:
            self.heartbeat_thread = None

    def publish_connection_status(self, event):
        """发布连接状态"""
        status = {
            'connected': self.is_connected,
            'qt_server': f"{self.qt_server_ip}:{self.qt_server_port}",
            'timestamp': rospy.Time.now().to_sec()
        }
        self.connection_status_pub.publish(String(data=json.dumps(status)))

    def shutdown(self):
        """关闭连接"""
        self.is_connected = False
        if self.ws:
            self.ws.close()

    def run(self):
        """主循环"""
        rospy.loginfo("QT Communication Bridge running")
        try:
            rospy.spin()
        finally:
            self.shutdown()

if __name__ == '__main__':
    try:
        bridge = QTCommunicationBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("QT Communication Bridge shutdown")
