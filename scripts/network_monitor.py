#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Network Monitor for WebSocket Map Server
监控网络状态和服务器性能
"""

import rospy
import psutil
import socket
import time
import json
from std_msgs.msg import String

class NetworkMonitor:
    def __init__(self):
        rospy.init_node('network_monitor', anonymous=True)
        
        # 配置参数
        self.monitor_interval = rospy.get_param('~monitor_interval', 5.0)
        self.log_network_stats = rospy.get_param('~log_network_stats', True)
        self.server_host = rospy.get_param('~server_host', '192.168.200.216')
        self.server_port = rospy.get_param('~server_port', 8000)
        
        # 发布器
        self.status_pub = rospy.Publisher('/network_monitor/status', String, queue_size=1)
        
        # 统计数据
        self.last_net_io = psutil.net_io_counters()
        self.last_time = time.time()
        
        rospy.loginfo("Network Monitor initialized")
        rospy.loginfo(f"Monitoring server at {self.server_host}:{self.server_port}")
        
    def check_port_status(self):
        """检查端口状态"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(1)
            result = sock.connect_ex((self.server_host, self.server_port))
            sock.close()
            return result == 0
        except Exception:
            return False
    
    def get_network_stats(self):
        """获取网络统计信息"""
        try:
            current_net_io = psutil.net_io_counters()
            current_time = time.time()
            
            time_delta = current_time - self.last_time
            
            if time_delta > 0:
                bytes_sent_per_sec = (current_net_io.bytes_sent - self.last_net_io.bytes_sent) / time_delta
                bytes_recv_per_sec = (current_net_io.bytes_recv - self.last_net_io.bytes_recv) / time_delta
            else:
                bytes_sent_per_sec = 0
                bytes_recv_per_sec = 0
            
            self.last_net_io = current_net_io
            self.last_time = current_time
            
            return {
                'bytes_sent_total': current_net_io.bytes_sent,
                'bytes_recv_total': current_net_io.bytes_recv,
                'bytes_sent_per_sec': bytes_sent_per_sec,
                'bytes_recv_per_sec': bytes_recv_per_sec,
                'packets_sent': current_net_io.packets_sent,
                'packets_recv': current_net_io.packets_recv,
                'errors_in': current_net_io.errin,
                'errors_out': current_net_io.errout,
                'drops_in': current_net_io.dropin,
                'drops_out': current_net_io.dropout
            }
        except Exception as e:
            rospy.logerr(f"Error getting network stats: {e}")
            return {}
    
    def get_system_stats(self):
        """获取系统统计信息"""
        try:
            return {
                'cpu_percent': psutil.cpu_percent(interval=0.1),
                'memory_percent': psutil.virtual_memory().percent,
                'memory_available_mb': psutil.virtual_memory().available / 1024 / 1024,
                'disk_usage_percent': psutil.disk_usage('/').percent,
                'load_average': psutil.getloadavg() if hasattr(psutil, 'getloadavg') else [0, 0, 0]
            }
        except Exception as e:
            rospy.logerr(f"Error getting system stats: {e}")
            return {}
    
    def get_network_interfaces(self):
        """获取网络接口信息"""
        try:
            interfaces = {}
            for interface, addrs in psutil.net_if_addrs().items():
                interface_info = {
                    'addresses': [],
                    'is_up': False
                }
                
                for addr in addrs:
                    if addr.family == socket.AF_INET:  # IPv4
                        interface_info['addresses'].append({
                            'ip': addr.address,
                            'netmask': addr.netmask,
                            'broadcast': addr.broadcast
                        })
                
                # 检查接口状态
                if interface in psutil.net_if_stats():
                    interface_info['is_up'] = psutil.net_if_stats()[interface].isup
                
                interfaces[interface] = interface_info
            
            return interfaces
        except Exception as e:
            rospy.logerr(f"Error getting network interfaces: {e}")
            return {}
    
    def monitor_loop(self):
        """监控主循环"""
        rate = rospy.Rate(1.0 / self.monitor_interval)
        
        while not rospy.is_shutdown():
            try:
                # 收集监控数据
                port_status = self.check_port_status()
                network_stats = self.get_network_stats()
                system_stats = self.get_system_stats()
                network_interfaces = self.get_network_interfaces()
                
                # 构建状态消息
                status_data = {
                    'timestamp': time.time(),
                    'server': {
                        'host': self.server_host,
                        'port': self.server_port,
                        'port_open': port_status
                    },
                    'network': network_stats,
                    'system': system_stats,
                    'interfaces': network_interfaces
                }
                
                # 发布状态
                status_msg = String()
                status_msg.data = json.dumps(status_data, indent=2)
                self.status_pub.publish(status_msg)
                
                # 日志输出
                if self.log_network_stats:
                    rospy.loginfo(f"Port {self.server_port} {'OPEN' if port_status else 'CLOSED'}")
                    rospy.loginfo(f"Network: ↑{network_stats.get('bytes_sent_per_sec', 0)/1024:.1f}KB/s "
                                f"↓{network_stats.get('bytes_recv_per_sec', 0)/1024:.1f}KB/s")
                    rospy.loginfo(f"System: CPU {system_stats.get('cpu_percent', 0):.1f}% "
                                f"MEM {system_stats.get('memory_percent', 0):.1f}%")
                
            except Exception as e:
                rospy.logerr(f"Monitor loop error: {e}")
            
            rate.sleep()

def main():
    try:
        monitor = NetworkMonitor()
        monitor.monitor_loop()
    except rospy.ROSInterruptException:
        rospy.loginfo("Network Monitor shutdown")
    except KeyboardInterrupt:
        rospy.loginfo("Network Monitor interrupted")

if __name__ == '__main__':
    main()
