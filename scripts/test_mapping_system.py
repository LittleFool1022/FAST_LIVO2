#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
建图系统测试脚本
验证激光雷达驱动、FAST-LIVO算法和2D地图生成器的完整工作流程
"""

import rospy
import subprocess
import time
import signal
import sys
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

class MappingSystemTester:
    def __init__(self):
        rospy.init_node('mapping_system_tester', anonymous=True)
        
        self.cloud_received = False
        self.map_received = False
        self.pose_received = False
        
        # 订阅关键话题
        self.cloud_sub = rospy.Subscriber('/cloud_registered', PointCloud2, self.cloud_callback)
        self.map_sub = rospy.Subscriber('/projected_map', OccupancyGrid, self.map_callback)
        self.pose_sub = rospy.Subscriber('/Odometry', PoseStamped, self.pose_callback)
        
        rospy.loginfo("[测试器] 建图系统测试器初始化完成")

    def cloud_callback(self, msg):
        if not self.cloud_received:
            rospy.loginfo("[测试器] ✅ 接收到点云数据: /cloud_registered")
            rospy.loginfo(f"[测试器] 点云信息: {msg.width}x{msg.height} 点")
            self.cloud_received = True

    def map_callback(self, msg):
        if not self.map_received:
            rospy.loginfo("[测试器] ✅ 接收到2D地图数据: /projected_map")
            rospy.loginfo(f"[测试器] 地图信息: {msg.info.width}x{msg.info.height}, 分辨率: {msg.info.resolution}m")
            self.map_received = True
            
        # 分析地图质量
        self.analyze_map_quality(msg)

    def pose_callback(self, msg):
        if not self.pose_received:
            rospy.loginfo("[测试器] ✅ 接收到位姿数据: /Odometry")
            self.pose_received = True

    def analyze_map_quality(self, map_msg):
        """分析地图质量"""
        data = map_msg.data
        total_cells = len(data)
        
        obstacles = sum(1 for x in data if x == 100)
        free_space = sum(1 for x in data if x == 0)
        unknown = sum(1 for x in data if x == -1)
        
        obstacle_ratio = obstacles / total_cells * 100
        free_ratio = free_space / total_cells * 100
        unknown_ratio = unknown / total_cells * 100
        
        rospy.loginfo_throttle(10.0, 
            f"[测试器] 地图质量分析:\n"
            f"  - 障碍物: {obstacles} 个 ({obstacle_ratio:.1f}%)\n"
            f"  - 空闲区域: {free_space} 个 ({free_ratio:.1f}%)\n"
            f"  - 未知区域: {unknown} 个 ({unknown_ratio:.1f}%)")
        
        # 质量评估
        if free_ratio > 30 and obstacle_ratio > 5 and obstacle_ratio < 25:
            rospy.loginfo_throttle(15.0, "[测试器] 🎯 地图质量良好！")
        elif free_ratio < 10:
            rospy.logwarn_throttle(15.0, "[测试器] ⚠️ 空闲区域过少，可能需要调整参数")
        elif obstacle_ratio > 40:
            rospy.logwarn_throttle(15.0, "[测试器] ⚠️ 障碍物过多，可能存在噪声")

    def run_system_test(self):
        """运行系统测试"""
        rospy.loginfo("[测试器] 🚀 开始建图系统测试...")
        
        # 等待数据
        rospy.loginfo("[测试器] 等待系统数据...")
        
        start_time = time.time()
        timeout = 30  # 30秒超时
        
        while not rospy.is_shutdown() and (time.time() - start_time) < timeout:
            if self.cloud_received and self.map_received and self.pose_received:
                rospy.loginfo("[测试器] 🎉 所有系统组件正常工作！")
                break
            time.sleep(1)
        
        if (time.time() - start_time) >= timeout:
            rospy.logwarn("[测试器] ⏰ 测试超时，检查系统状态：")
            if not self.cloud_received:
                rospy.logwarn("[测试器] ❌ 未接收到点云数据 - 检查FAST-LIVO")
            if not self.map_received:
                rospy.logwarn("[测试器] ❌ 未接收到地图数据 - 检查2D地图生成器")
            if not self.pose_received:
                rospy.logwarn("[测试器] ❌ 未接收到位姿数据 - 检查里程计")
        
        # 持续监控
        rospy.loginfo("[测试器] 进入持续监控模式...")
        rospy.spin()

def signal_handler(sig, frame):
    rospy.loginfo("[测试器] 接收到退出信号，正在关闭...")
    sys.exit(0)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        tester = MappingSystemTester()
        tester.run_system_test()
    except rospy.ROSInterruptException:
        rospy.loginfo("[测试器] 测试正常结束")
