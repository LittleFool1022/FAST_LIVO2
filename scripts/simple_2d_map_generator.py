#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简化稳定的2D地图生成器
专注于稳定地图生成和清晰轮廓展示
"""

import rospy
import tf2_ros
import numpy as np
import math
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
import sensor_msgs.point_cloud2 as pc2
from threading import Lock

class Simple2DMapGenerator:
    def __init__(self):
        rospy.init_node('simple_2d_map_generator', anonymous=True)

        # 基础参数配置 - 提高分辨率
        self.map_resolution = rospy.get_param('~map_resolution', 0.05)  # 改为5cm分辨率
        self.max_range = rospy.get_param('~max_range', 15.0)
        self.min_range = rospy.get_param('~min_range', 0.1)
        self.min_height = rospy.get_param('~min_height', -0.4)
        self.max_height = rospy.get_param('~max_height', 3.0)

        # 障碍物检测参数
        self.obstacle_threshold = rospy.get_param('~obstacle_threshold', 0.02)  # 2cm以上为障碍物

        # 空闲区域检测参数
        self.free_space_radius = rospy.get_param('~free_space_radius', 0.3)  # 机器人周围0.3m标记为空闲

        # 动态地图边界跟踪
        self.min_x = float('inf')
        self.max_x = float('-inf')
        self.min_y = float('inf')
        self.max_y = float('-inf')
        self.has_data = False

        # 点云数据缓存 - 改为累积模式
        self.obstacle_points = []
        self.free_points = []

        # 累积数据字典 - 用于稳定地图
        self.obstacle_hits = {}  # 记录每个位置被标记为障碍物的次数
        self.free_hits = {}      # 记录每个位置被标记为空闲的次数
        self.scanned_area = {}   # 记录已扫描区域
        self.max_history = 10    # 最大历史记录数

        # 机器人位置
        self.robot_x = 0.0
        self.robot_y = 0.0

        # TF和线程
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.data_lock = Lock()

        # ROS发布器和订阅器
        self.map_pub = rospy.Publisher('/enhanced_2d_map', OccupancyGrid, queue_size=1, latch=False)  # 关闭latch
        self.cloud_sub = rospy.Subscriber('/cloud_registered', PointCloud2, self.cloud_callback, queue_size=1)

        # 定时器 - 提高发布频率
        self.map_timer = rospy.Timer(rospy.Duration(0.5), self.publish_map)  # 改为0.5秒

        # 添加变化检测
        self.last_map_hash = None
        self.map_changed = True

        rospy.loginfo("简化稳定2D地图生成器已初始化")
        rospy.loginfo(f"分辨率: {self.map_resolution}m, 最大范围: {self.max_range}m")
        rospy.loginfo(f"障碍物阈值: {self.obstacle_threshold}m")

    def get_robot_pose(self):
        """获取机器人位姿"""
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(0.1))
            self.robot_x = transform.transform.translation.x
            self.robot_y = transform.transform.translation.y
            return True
        except:
            # 使用默认位置
            return True

    def cloud_callback(self, cloud_msg):
        """改进的点云处理 - 添加累积机制"""
        with self.data_lock:
            try:
                self.get_robot_pose()

                # 当前帧的临时数据
                current_obstacles = []
                current_free = []

                # 读取点云
                points = list(pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True))
                if len(points) < 10:
                    return

                # 处理点云
                for x, y, z in points:
                    # 基础过滤
                    if (z < self.min_height or z > self.max_height or
                        math.sqrt(x*x + y*y) > self.max_range or
                        math.sqrt(x*x + y*y) < self.min_range):
                        continue

                    # 转换到全局坐标
                    global_x = x + self.robot_x
                    global_y = y + self.robot_y

                    # 更新边界
                    self.update_bounds(global_x, global_y)

                    # 障碍物检测并累积
                    grid_key = self.get_grid_key(global_x, global_y)
                    if self.is_obstacle(z):
                        current_obstacles.append((global_x, global_y))
                        self.update_obstacle_hits(grid_key)
                    else:
                        current_free.append((global_x, global_y))
                        self.update_free_hits(grid_key)

                # 标记机器人周围为已扫描区域
                self.mark_scanned_area()

                # 添加光线追踪的空闲点
                self.add_ray_traced_free_points_with_accumulation(current_obstacles)

                # 添加全方位空闲区域检测
                self.add_omnidirectional_free_space(points)

                # 更新稳定的点云数据
                self.update_stable_points()

                # 标记地图已变化
                self.map_changed = True

                rospy.loginfo_throttle(5.0, f"累积数据: 障碍物={len(self.obstacle_hits)}, 空闲={len(self.free_hits)}, 已扫描={len(self.scanned_area)}")
                rospy.loginfo_throttle(2.0, f"🤖 机器人位置: ({self.robot_x:.2f}, {self.robot_y:.2f})")

            except Exception as e:
                rospy.logerr(f"点云处理错误: {e}")

    def get_grid_key(self, x, y):
        """获取网格键值，用于累积数据"""
        grid_x = int(x / self.map_resolution)
        grid_y = int(y / self.map_resolution)
        return (grid_x, grid_y)

    def update_obstacle_hits(self, grid_key):
        """更新障碍物命中次数"""
        if grid_key not in self.obstacle_hits:
            self.obstacle_hits[grid_key] = 0
        self.obstacle_hits[grid_key] = min(self.obstacle_hits[grid_key] + 1, self.max_history)

        # 如果该位置也被标记为空闲，减少空闲计数
        if grid_key in self.free_hits:
            self.free_hits[grid_key] = max(0, self.free_hits[grid_key] - 1)
            if self.free_hits[grid_key] == 0:
                del self.free_hits[grid_key]

    def update_free_hits(self, grid_key):
        """更新空闲命中次数"""
        if grid_key not in self.free_hits:
            self.free_hits[grid_key] = 0
        self.free_hits[grid_key] = min(self.free_hits[grid_key] + 1, self.max_history)

        # 如果该位置也被标记为障碍物，减少障碍物计数
        if grid_key in self.obstacle_hits:
            self.obstacle_hits[grid_key] = max(0, self.obstacle_hits[grid_key] - 1)
            if self.obstacle_hits[grid_key] == 0:
                del self.obstacle_hits[grid_key]

    def mark_scanned_area(self):
        """标记机器人周围区域为已扫描"""
        # 在机器人周围一定半径内标记为已扫描
        steps = int(self.free_space_radius / self.map_resolution)
        for dx in range(-steps, steps + 1):
            for dy in range(-steps, steps + 1):
                distance = math.sqrt(dx*dx + dy*dy) * self.map_resolution
                if distance <= self.free_space_radius:
                    scan_x = self.robot_x + dx * self.map_resolution
                    scan_y = self.robot_y + dy * self.map_resolution
                    grid_key = self.get_grid_key(scan_x, scan_y)

                    # 标记为已扫描
                    if grid_key not in self.scanned_area:
                        self.scanned_area[grid_key] = 0
                    self.scanned_area[grid_key] = min(self.scanned_area[grid_key] + 1, self.max_history)

                    # 如果没有障碍物，标记为空闲
                    if grid_key not in self.obstacle_hits or self.obstacle_hits[grid_key] < 2:
                        self.update_free_hits(grid_key)

    def update_bounds(self, x, y):
        """更新数据边界"""
        self.min_x = min(self.min_x, x)
        self.max_x = max(self.max_x, x)
        self.min_y = min(self.min_y, y)
        self.max_y = max(self.max_y, y)
        self.has_data = True

    def is_obstacle(self, z):
        """改进的障碍物判断 - 更稳定的检测"""
        # 激光雷达高度0.25m，地面在-0.25m
        ground_level = -0.25

        # 地面容差范围
        ground_tolerance = 0.05

        # 如果在地面容差范围内，认为是地面
        if abs(z - ground_level) <= ground_tolerance:
            return False

        # 如果明显高于地面，认为是障碍物
        if z > ground_level + self.obstacle_threshold:
            return True

        # 如果明显低于地面，可能是坑洞，也算障碍物
        if z < ground_level - ground_tolerance * 2:
            return True

        return False

    def add_ray_traced_free_points_with_accumulation(self, current_obstacles):
        """添加光线追踪的空闲点并累积"""
        robot_global_x = self.robot_x
        robot_global_y = self.robot_y

        # 对当前帧的障碍物点进行光线追踪
        for obs_x, obs_y in current_obstacles[:50]:  # 限制数量提高性能
            dx = obs_x - robot_global_x
            dy = obs_y - robot_global_y
            distance = math.sqrt(dx*dx + dy*dy)

            if distance > 0.1:  # 避免除零
                # 沿光线添加空闲点
                steps = int(distance / self.map_resolution)
                for i in range(1, steps):  # 不包括起点和终点
                    ratio = i / steps
                    free_x = robot_global_x + dx * ratio
                    free_y = robot_global_y + dy * ratio

                    # 累积空闲点
                    grid_key = self.get_grid_key(free_x, free_y)
                    self.update_free_hits(grid_key)

    def add_omnidirectional_free_space(self, points):
        """添加全方位空闲区域检测 - 基于激光雷达扫描角度"""
        robot_global_x = self.robot_x
        robot_global_y = self.robot_y

        # 分析点云的角度分布
        angle_ranges = {}
        for x, y, z in points:
            # 基础过滤
            if (z < self.min_height or z > self.max_height or
                math.sqrt(x*x + y*y) > self.max_range or
                math.sqrt(x*x + y*y) < self.min_range):
                continue

            # 计算角度（相对于机器人）
            angle = math.atan2(y, x)
            distance = math.sqrt(x*x + y*y)

            # 将角度分组（每5度一组）
            angle_group = int(angle * 180 / math.pi / 5) * 5

            if angle_group not in angle_ranges:
                angle_ranges[angle_group] = []
            angle_ranges[angle_group].append(distance)

        # 对每个角度方向，标记到最远点的路径为空闲
        for angle_deg, distances in angle_ranges.items():
            if len(distances) > 0:
                max_distance = max(distances)
                angle_rad = angle_deg * math.pi / 180

                # 沿该方向标记空闲点
                steps = int(max_distance / self.map_resolution)
                for i in range(1, steps):
                    distance = i * self.map_resolution
                    free_x = robot_global_x + distance * math.cos(angle_rad)
                    free_y = robot_global_y + distance * math.sin(angle_rad)

                    grid_key = self.get_grid_key(free_x, free_y)
                    self.update_free_hits(grid_key)

    def update_stable_points(self):
        """更新稳定的点云数据用于地图生成 - 更快响应变化"""
        self.obstacle_points.clear()
        self.free_points.clear()

        # 降低障碍物确认阈值（命中次数 >= 2）
        for (grid_x, grid_y), hits in self.obstacle_hits.items():
            if hits >= 2:
                world_x = grid_x * self.map_resolution
                world_y = grid_y * self.map_resolution
                self.obstacle_points.append((world_x, world_y))

        # 更宽松的空闲点确认（命中次数 >= 1）
        for (grid_x, grid_y), hits in self.free_hits.items():
            if hits >= 1:
                world_x = grid_x * self.map_resolution
                world_y = grid_y * self.map_resolution
                self.free_points.append((world_x, world_y))

        # 添加已扫描但没有障碍物的区域作为空闲点
        for (grid_x, grid_y), scan_hits in self.scanned_area.items():
            if scan_hits >= 1 and (grid_x, grid_y) not in self.obstacle_hits:
                world_x = grid_x * self.map_resolution
                world_y = grid_y * self.map_resolution
                # 避免重复添加
                if (world_x, world_y) not in self.free_points:
                    self.free_points.append((world_x, world_y))

        # 添加已扫描但没有障碍物的区域作为空闲点
        for (grid_x, grid_y), scan_hits in self.scanned_area.items():
            if scan_hits >= 2 and (grid_x, grid_y) not in self.obstacle_hits:
                world_x = grid_x * self.map_resolution
                world_y = grid_y * self.map_resolution
                # 避免重复添加
                if (world_x, world_y) not in self.free_points:
                    self.free_points.append((world_x, world_y))

    def generate_map(self):
        """生成动态尺寸地图"""
        if not self.has_data:
            # 返回默认小地图
            return np.full((10, 10), -1, dtype=np.int8), 0.0, 0.0, 10, 10

        # 计算地图边界（添加缓冲区）
        buffer = 1.0  # 1米缓冲区
        map_min_x = self.min_x - buffer
        map_max_x = self.max_x + buffer
        map_min_y = self.min_y - buffer
        map_max_y = self.max_y + buffer

        # 计算地图尺寸
        map_width = int((map_max_x - map_min_x) / self.map_resolution) + 1
        map_height = int((map_max_y - map_min_y) / self.map_resolution) + 1

        # 限制地图尺寸
        map_width = max(10, min(map_width, 300))
        map_height = max(10, min(map_height, 300))

        # 创建地图
        occupancy_map = np.full((map_height, map_width), -1, dtype=np.int8)

        # 标记障碍物
        for x, y in self.obstacle_points:
            map_x = int((x - map_min_x) / self.map_resolution)
            map_y = int((y - map_min_y) / self.map_resolution)
            if 0 <= map_x < map_width and 0 <= map_y < map_height:
                occupancy_map[map_y, map_x] = 100

        # 标记空闲区域
        for x, y in self.free_points:
            map_x = int((x - map_min_x) / self.map_resolution)
            map_y = int((y - map_min_y) / self.map_resolution)
            if 0 <= map_x < map_width and 0 <= map_y < map_height:
                if occupancy_map[map_y, map_x] != 100:  # 不覆盖障碍物
                    occupancy_map[map_y, map_x] = 0

        return occupancy_map, map_min_x, map_min_y, map_width, map_height

    def publish_map(self, event):
        """发布动态地图 - 改进实时更新"""
        try:
            with self.data_lock:
                # 检查是否有变化
                if not self.map_changed and self.last_map_hash is not None:
                    return

                # 生成地图
                occupancy_map, origin_x, origin_y, width, height = self.generate_map()

                # 计算地图哈希值检测变化
                import hashlib
                map_hash = hashlib.md5(occupancy_map.tobytes()).hexdigest()

                # 如果地图没有实际变化，跳过发布
                if map_hash == self.last_map_hash:
                    return

                self.last_map_hash = map_hash
                self.map_changed = False

                # 创建ROS消息
                grid_msg = OccupancyGrid()
                grid_msg.header.stamp = rospy.Time.now()
                grid_msg.header.frame_id = "map"

                grid_msg.info.resolution = self.map_resolution
                grid_msg.info.width = width
                grid_msg.info.height = height
                grid_msg.info.origin.position.x = origin_x
                grid_msg.info.origin.position.y = origin_y
                grid_msg.info.origin.position.z = 0.0
                grid_msg.info.origin.orientation.w = 1.0

                grid_msg.data = occupancy_map.flatten().tolist()
                self.map_pub.publish(grid_msg)

                # 更详细的实时日志
                rospy.loginfo_throttle(3.0,
                    f"� 地图更新: {width}x{height}, 障碍物={len(self.obstacle_points)}, 空闲={len(self.free_points)}")

                rospy.loginfo_throttle(1.0,
                    f"📍 机器人: ({self.robot_x:.2f}, {self.robot_y:.2f}), 地图原点: ({origin_x:.2f}, {origin_y:.2f})")

                if self.has_data:
                    rospy.loginfo_throttle(30.0,
                        f"🗺️ 地图边界: X=[{self.min_x:.1f}, {self.max_x:.1f}], Y=[{self.min_y:.1f}, {self.max_y:.1f}]")

        except Exception as e:
            rospy.logerr(f"发布地图错误: {e}")

    def run(self):
        """运行节点"""
        rospy.loginfo("简化2D地图生成器运行中...")
        rospy.spin()

if __name__ == '__main__':
    try:
        generator = Simple2DMapGenerator()
        generator.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点正常退出")


