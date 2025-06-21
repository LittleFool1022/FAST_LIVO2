#!/bin/bash

# FAST-LIVO地图生成和传输系统 启动脚本
# 支持2D/3D地图、WebSocket通信、地图保存与QT交互

# -------------------- 工具函数 --------------------
check_node_exists() {
    local node_name="$1"
    if rosnode list | grep -q "$node_name"; then
        echo "✅ 节点 $node_name 存在"
    else
        echo "❌ 节点 $node_name 不存在"
    fi
}

safe_kill() {
    local pid="$1"
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
        kill "$pid"
        echo "🔻 已终止 PID: $pid"
    fi
}

echo "=========================================="
echo "FAST-LIVO地图生成和传输系统 v2.0"
echo "=========================================="
echo "功能："
echo "- 2D占用栅格地图生成"
echo "- 3D点云地图收集和处理"
echo "- 地图文件保存(PGM/YAML/PCD)"
echo "- QT上位机WebSocket通信"
echo "- 数据压缩和传输优化"
echo ""

# 检查ROS环境
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS环境未设置，请先source ROS"
    exit 1
fi

# 检查工作空间
if [ ! -f "devel/setup.bash" ]; then
    echo "❌ 请在catkin工作空间根目录运行此脚本"
    exit 1
fi

# Source工作空间
source devel/setup.bash

echo "✅ ROS环境检查完成"
echo ""

# 参数配置
QT_SERVER_IP=${QT_SERVER_IP:-"192.168.200.164"}
QT_SERVER_PORT=${QT_SERVER_PORT:-8001}
ENABLE_QT_COMMUNICATION=${ENABLE_QT_COMMUNICATION:-true}
ENABLE_COMPRESSION=${ENABLE_COMPRESSION:-true}
MAX_POINTS_PER_CHUNK=${MAX_POINTS_PER_CHUNK:-50000}
USE_DIRECT_WEBSOCKET=${USE_DIRECT_WEBSOCKET:-true}
ENABLE_MAP_MANAGER=${ENABLE_MAP_MANAGER:-true}

echo "配置参数："
echo "QT服务器IP: $QT_SERVER_IP"
echo "QT服务器端口: $QT_SERVER_PORT"
echo "启用QT通信: $ENABLE_QT_COMMUNICATION"
echo "启用数据压缩: $ENABLE_COMPRESSION"
echo "每包最大点数: $MAX_POINTS_PER_CHUNK"
echo "使用直接WebSocket: $USE_DIRECT_WEBSOCKET"
echo "启用地图管理器: $ENABLE_MAP_MANAGER"
echo ""

# 检查必要的包
echo "检查依赖包..."
REQUIRED_PACKAGES=("rosbridge_server" "tf" "sensor_msgs" "nav_msgs")
for pkg in "${REQUIRED_PACKAGES[@]}"; do
    if rospack find $pkg >/dev/null 2>&1; then
        echo "✅ $pkg"
    else
        echo "❌ $pkg 未安装"
        echo "请安装: sudo apt-get install ros-$ROS_DISTRO-$pkg"
        exit 1
    fi
done

# 检查Python依赖
echo "检查Python依赖..."
python3 -c "import websocket, gzip, base64" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✅ Python依赖"
else
    echo "❌ Python依赖缺失"
    echo "请安装: pip3 install websocket-client"
    exit 1
fi

echo ""

# 检查FAST-LIO状态
echo "检查FAST-LIO状态..."
if rosnode ping /fast_lio -c 1 >/dev/null 2>&1; then
    echo "✅ FAST-LIO节点运行正常"
else
    echo "⚠️ FAST-LIO节点未运行"
    echo "请先启动FAST-LIO："
    echo "  roslaunch fast_livo run_mid360.launch"
    echo ""
    read -p "是否继续启动地图系统？(y/n): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# 检查点云话题
echo "检查点云话题..."
if rostopic list | grep -q "/cloud_registered"; then
    echo "✅ 点云话题正常"
else
    echo "❌ 未发现/cloud_registered话题"
    echo "请确保FAST-LIO正常运行"
    exit 1
fi

echo ""

# 创建必要的目录
echo "创建目录结构..."
mkdir -p ~/catkin_ws/maps
mkdir -p logs/mapping_system
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_DIR="logs/mapping_system"

echo "日志目录: $LOG_DIR"
echo "时间戳: $TIMESTAMP"
echo ""

# 启动系统组件
echo "启动地图生成和传输系统..."
echo "=========================================="

# 1. 启动2D地图生成器
echo "1. 启动2D地图生成器..."
python3 src/fast_livo/scripts/simple_2d_map_generator.py \
    2>&1 | tee $LOG_DIR/2d_map_generator_$TIMESTAMP.log &
MAP_2D_PID=$!
sleep 2

# 检查2D地图生成器
if ! rosnode ping /simple_2d_map_generator* -c 1 >/dev/null 2>&1; then
    echo "❌ 2D地图生成器启动失败"
    exit 1
fi
echo "✅ 2D地图生成器启动成功"

# 2. 启动地图管理器（可选）
if [ "$ENABLE_MAP_MANAGER" = "true" ]; then
    echo "2. 启动地图管理器..."
    python3 src/fast_livo/scripts/map_manager.py \
        _enable_compression:=$ENABLE_COMPRESSION \
        _max_points_per_chunk:=$MAX_POINTS_PER_CHUNK \
        2>&1 | tee $LOG_DIR/map_manager_$TIMESTAMP.log &
    MAP_MANAGER_PID=$!
    sleep 3

    # 检查地图管理器
    if ! rosnode ping /map_manager* -c 1 >/dev/null 2>&1; then
        echo "❌ 地图管理器启动失败"
        kill $MAP_2D_PID 2>/dev/null
        exit 1
    fi
    echo "✅ 地图管理器启动成功"
else
    echo "2. 地图管理器已禁用"
    MAP_MANAGER_PID=""
fi

# 3. 启动ROSBridge服务器（如果启用QT通信）
if [ "$ENABLE_QT_COMMUNICATION" = "true" ]; then
    echo "3. 启动ROSBridge服务器..."
    
    # 检查端口是否被占用
    if netstat -tuln | grep -q ":$QT_SERVER_PORT "; then
        echo "⚠️ 端口 $QT_SERVER_PORT 已被占用"
        read -p "是否继续？(y/n): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            kill $MAP_2D_PID $MAP_MANAGER_PID 2>/dev/null
            exit 1
        fi
    fi
    
    roslaunch rosbridge_server rosbridge_websocket.launch \
        port:=$QT_SERVER_PORT \
        address:=$QT_SERVER_IP \
        2>&1 | tee $LOG_DIR/rosbridge_$TIMESTAMP.log &
    ROSBRIDGE_PID=$!
    sleep 3
    
    # 检查ROSBridge
    if ! rosnode ping /rosbridge_websocket -c 1 >/dev/null 2>&1; then
        echo "❌ ROSBridge服务器启动失败"
        kill $MAP_2D_PID $MAP_MANAGER_PID 2>/dev/null
        exit 1
    fi
    echo "✅ ROSBridge服务器启动成功"
    
    # 4. 启动WebSocket发送器
    if [ "$USE_DIRECT_WEBSOCKET" = "true" ]; then
        echo "4. 启动直接WebSocket发送器..."
        python3 src/fast_livo/scripts/direct_websocket_sender.py \
            _qt_host:=$QT_SERVER_IP \
            _qt_port:=$QT_SERVER_PORT \
            _enable_compression:=$ENABLE_COMPRESSION \
            2>&1 | tee $LOG_DIR/websocket_sender_$TIMESTAMP.log &
        WEBSOCKET_SENDER_PID=$!
        sleep 2

        echo "✅ 直接WebSocket发送器启动成功"
    else
        echo "4. 启动QT通信桥接器..."
        python3 scripts/qt_communication_bridge.py \
            _qt_server_ip:=$QT_SERVER_IP \
            _qt_server_port:=$QT_SERVER_PORT \
            2>&1 | tee $LOG_DIR/qt_bridge_$TIMESTAMP.log &
        QT_BRIDGE_PID=$!
        sleep 2

        echo "✅ QT通信桥接器启动成功"
        WEBSOCKET_SENDER_PID=""
    fi
else
    echo "3. QT通信功能已禁用"
    ROSBRIDGE_PID=""
    QT_BRIDGE_PID=""
fi

echo ""
echo "=========================================="
echo "✅ 系统启动完成"
echo "=========================================="
echo ""

# 显示系统状态
echo "系统组件状态："
echo "- 2D地图生成器: PID $MAP_2D_PID"
if [ "$ENABLE_MAP_MANAGER" = "true" ]; then
    echo "- 地图管理器: PID $MAP_MANAGER_PID"
fi
if [ "$ENABLE_QT_COMMUNICATION" = "true" ]; then
    echo "- ROSBridge服务器: PID $ROSBRIDGE_PID"
    if [ "$USE_DIRECT_WEBSOCKET" = "true" ]; then
        echo "- 直接WebSocket发送器: PID $WEBSOCKET_SENDER_PID"
    else
        echo "- QT通信桥接器: PID $QT_BRIDGE_PID"
    fi
fi
echo ""

# 显示可用服务
echo "可用服务："
if [ "$ENABLE_MAP_MANAGER" = "true" ]; then
    echo "地图管理器服务："
    echo "- 开始地图收集: rosservice call /start_map_collection"
    echo "- 停止地图收集: rosservice call /stop_map_collection"
    echo "- 保存地图: rosservice call /save_maps"
    echo "- 发送2D地图: rosservice call /send_2d_map"
    echo "- 发送3D地图: rosservice call /send_3d_map"
fi

if [ "$ENABLE_QT_COMMUNICATION" = "true" ] && [ "$USE_DIRECT_WEBSOCKET" = "true" ]; then
    echo "WebSocket发送器服务："
    echo "- 发送2D地图: rosservice call /send_2d_websocket"
    echo "- 发送3D地图: rosservice call /send_3d_websocket"
    echo "- 测试发送: rosservice call /test_raw_send"
fi
echo ""

# 显示话题
echo "重要话题："
echo "- 2D地图: /enhanced_2d_map"
echo "- 地图数据传输: /map_data_for_qt"
echo "- 系统状态: /map_manager_status"
if [ "$ENABLE_QT_COMMUNICATION" = "true" ]; then
    echo "- QT连接状态: /qt_connection_status"
fi
echo ""

# 显示QT连接信息
if [ "$ENABLE_QT_COMMUNICATION" = "true" ]; then
    echo "QT上位机连接信息："
    echo "- WebSocket地址: ws://$QT_SERVER_IP:$QT_SERVER_PORT"
    echo "- 数据压缩: $ENABLE_COMPRESSION"
    echo "- 分包大小: $MAX_POINTS_PER_CHUNK 点/包"
    echo ""
fi

# 使用说明
echo "使用说明："
echo "1. 开始建图: rosservice call /start_map_collection"
echo "2. 移动机器人进行建图"
echo "3. 停止建图: rosservice call /stop_map_collection"
echo "4. 保存地图: rosservice call /save_maps"
echo "5. 发送到QT: rosservice call /send_2d_map 或 /send_3d_map"
echo ""

# 询问是否启动监控
read -p "是否启动实时监控？(y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "启动实时监控..."
    echo "按Ctrl+C停止监控和系统"
    
    # 监控循环
    while true; do
        echo "=========================================="
        echo "系统状态 - $(date)"
        echo "=========================================="
        
        # 检查节点状态
        echo "节点状态："
        for node in "simple_2d_map_generator" "map_manager"; do
            if rosnode ping /$node* -c 1 >/dev/null 2>&1; then
                echo "✅ $node"
            else
                echo "❌ $node"
            fi
        done
        
        if [ "$ENABLE_QT_COMMUNICATION" = "true" ]; then
            for node in "rosbridge_websocket" "qt_communication_bridge"; do
                if rosnode ping /$node* -c 1 >/dev/null 2>&1; then
                    echo "✅ $node"
                else
                    echo "❌ $node"
                fi
            done
        fi
        
        # 显示话题频率
        echo ""
        echo "话题频率："
        timeout 3s rostopic hz /enhanced_2d_map 2>/dev/null | head -1 || echo "/enhanced_2d_map: 无数据"
        timeout 3s rostopic hz /cloud_registered 2>/dev/null | head -1 || echo "/cloud_registered: 无数据"
        
        echo ""
        sleep 10
    done
else
    echo "系统在后台运行"
    echo "查看日志: tail -f $LOG_DIR/*_$TIMESTAMP.log"
    echo "停止系统: kill $MAP_2D_PID $MAP_MANAGER_PID $ROSBRIDGE_PID $QT_BRIDGE_PID"
fi

# 信号处理
cleanup() {
    echo ""
    echo "正在停止系统..."
    [ ! -z "$MAP_2D_PID" ] && kill $MAP_2D_PID 2>/dev/null
    [ ! -z "$MAP_MANAGER_PID" ] && kill $MAP_MANAGER_PID 2>/dev/null
    [ ! -z "$ROSBRIDGE_PID" ] && kill $ROSBRIDGE_PID 2>/dev/null
    [ ! -z "$QT_BRIDGE_PID" ] && kill $QT_BRIDGE_PID 2>/dev/null
    
    echo "系统已停止"
    exit 0
}

trap cleanup SIGINT SIGTERM

# 等待用户中断
wait
