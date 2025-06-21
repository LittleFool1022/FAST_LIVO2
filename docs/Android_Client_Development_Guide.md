# Android WebSocket地图客户端开发指南

## 1. 服务器连接信息

- **服务器地址**: `ws://192.168.200.216:8000`
- **协议**: WebSocket
- **数据格式**: JSON
- **压缩**: 支持gzip压缩 + base64编码

## 2. 客户端请求格式

### 2.1 基础请求结构
```json
{
    "type": "请求类型",
    "timestamp": 1703123456.789
}
```

### 2.2 支持的请求类型

#### A. 请求2D地图
```json
{
    "type": "request_2d_map",
    "timestamp": 1703123456.789
}
```

#### B. 请求3D点云
```json
{
    "type": "request_3d_pointcloud", 
    "timestamp": 1703123456.789
}
```

#### C. 请求机器人位姿
```json
{
    "type": "request_robot_pose",
    "timestamp": 1703123456.789
}
```

#### D. 请求服务器统计信息
```json
{
    "type": "request_server_stats",
    "timestamp": 1703123456.789
}
```

## 3. 服务器响应格式

### 3.1 欢迎消息（连接后自动发送）
```json
{
    "type": "welcome",
    "message": "Connected to ROS WebSocket Map Server",
    "server_time": 1703123456.789,
    "available_requests": [
        "request_2d_map",
        "request_3d_pointcloud",
        "request_robot_pose", 
        "request_server_stats"
    ]
}
```

### 3.2 2D地图响应
```json
{
    "type": "response_2d_map",
    "compressed": false,
    "data": {
        "type": "2d_map",
        "timestamp": 1703123456.789,
        "width": 1000,
        "height": 1000,
        "resolution": 0.05,
        "origin": {
            "x": -25.0,
            "y": -25.0,
            "theta": 0.0
        },
        "data": [
            -1, -1, -1, 0, 0, 0, 100, 100, 0, 0,
            -1, -1, 0, 0, 0, 0, 0, 100, 100, 0,
            "... 更多地图数据 ..."
        ],
        "robot_pose": {
            "x": 1.234,
            "y": 2.567,
            "z": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.123,
            "qw": 0.992,
            "timestamp": 1703123456.789
        }
    }
}
```

**地图数据说明**:
- `data`数组: 占用栅格数据
  - `-1`: 未知区域
  - `0`: 空闲区域  
  - `100`: 占用区域（障碍物）
- 数组索引计算: `index = y * width + x`

### 3.3 3D点云响应（分块传输）
```json
{
    "type": "response_3d_pointcloud_chunk",
    "compressed": false,
    "data": {
        "type": "3d_pointcloud_chunk",
        "timestamp": 1703123456.789,
        "chunk_id": 0,
        "total_chunks": 3,
        "points_in_chunk": 5000,
        "total_points": 12500,
        "points": [
            [1.234, 2.567, 0.123],
            [1.235, 2.568, 0.124],
            [1.236, 2.569, 0.125],
            "... 更多点云数据 ..."
        ],
        "robot_pose": {
            "x": 1.234,
            "y": 2.567,
            "z": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.123,
            "qw": 0.992,
            "timestamp": 1703123456.789
        }
    }
}
```

**点云数据说明**:
- `points`: 三维点数组，每个点格式为`[x, y, z]`
- 坐标单位: 米
- 分块传输: 大点云会分多个块发送

### 3.4 机器人位姿响应
```json
{
    "type": "response_robot_pose",
    "data": {
        "x": 1.234,
        "y": 2.567,
        "z": 0.0,
        "qx": 0.0,
        "qy": 0.0,
        "qz": 0.123,
        "qw": 0.992,
        "timestamp": 1703123456.789
    },
    "timestamp": 1703123456.789
}
```

### 3.5 服务器统计响应
```json
{
    "type": "response_server_stats",
    "data": {
        "clients_connected": 2,
        "maps_sent": 15,
        "pointclouds_sent": 8,
        "data_compressed": 12,
        "uptime": 3600.5,
        "start_time": 1703119856.789
    },
    "timestamp": 1703123456.789
}
```

### 3.6 错误响应
```json
{
    "type": "error",
    "message": "错误描述信息"
}
```

## 4. 数据压缩处理

### 4.1 压缩响应格式
```json
{
    "type": "response_2d_map",
    "compressed": true,
    "data": "H4sIAAAAAAAAA+compressed_base64_data_here=="
}
```

### 4.2 Android解压代码示例
```java
import java.util.Base64;
import java.util.zip.GZIPInputStream;
import java.io.ByteArrayInputStream;
import java.io.BufferedReader;
import java.io.InputStreamReader;

public class DataDecompressor {
    public static String decompressData(String compressedData) {
        try {
            // Base64解码
            byte[] compressed = Base64.getDecoder().decode(compressedData);
            
            // GZIP解压
            GZIPInputStream gzipStream = new GZIPInputStream(
                new ByteArrayInputStream(compressed)
            );
            
            BufferedReader reader = new BufferedReader(
                new InputStreamReader(gzipStream, "UTF-8")
            );
            
            StringBuilder result = new StringBuilder();
            String line;
            while ((line = reader.readLine()) != null) {
                result.append(line);
            }
            
            reader.close();
            return result.toString();
            
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }
}
```

## 5. Android WebSocket客户端实现

### 5.1 依赖添加（build.gradle）
```gradle
implementation 'org.java-websocket:Java-WebSocket:1.5.3'
implementation 'com.google.code.gson:gson:2.8.9'
```

### 5.2 WebSocket客户端类
```java
import org.java_websocket.client.WebSocketClient;
import org.java_websocket.handshake.ServerHandshake;
import com.google.gson.Gson;
import com.google.gson.JsonObject;

public class MapWebSocketClient extends WebSocketClient {
    private Gson gson = new Gson();
    private MapDataListener listener;
    
    public interface MapDataListener {
        void onMapReceived(MapData mapData);
        void onPointCloudReceived(PointCloudData pointCloud);
        void onRobotPoseReceived(RobotPose pose);
        void onError(String error);
    }
    
    public MapWebSocketClient(URI serverUri, MapDataListener listener) {
        super(serverUri);
        this.listener = listener;
    }
    
    @Override
    public void onOpen(ServerHandshake handshake) {
        Log.d("WebSocket", "Connected to server");
    }
    
    @Override
    public void onMessage(String message) {
        try {
            JsonObject response = gson.fromJson(message, JsonObject.class);
            String type = response.get("type").getAsString();
            
            switch (type) {
                case "welcome":
                    Log.d("WebSocket", "Welcome: " + 
                        response.get("message").getAsString());
                    break;
                    
                case "response_2d_map":
                    handleMapResponse(response);
                    break;
                    
                case "response_3d_pointcloud_chunk":
                    handlePointCloudResponse(response);
                    break;
                    
                case "response_robot_pose":
                    handlePoseResponse(response);
                    break;
                    
                case "error":
                    listener.onError(response.get("message").getAsString());
                    break;
            }
        } catch (Exception e) {
            listener.onError("Parse error: " + e.getMessage());
        }
    }
    
    public void requestMap() {
        JsonObject request = new JsonObject();
        request.addProperty("type", "request_2d_map");
        request.addProperty("timestamp", System.currentTimeMillis() / 1000.0);
        send(gson.toJson(request));
    }
    
    public void requestPointCloud() {
        JsonObject request = new JsonObject();
        request.addProperty("type", "request_3d_pointcloud");
        request.addProperty("timestamp", System.currentTimeMillis() / 1000.0);
        send(gson.toJson(request));
    }
    
    // ... 其他方法实现
}
```

## 6. 地图渲染建议

### 6.1 2D地图渲染
- 使用`Canvas`或`OpenGL ES`绘制栅格地图
- 颜色方案:
  - 未知区域: 灰色 `#808080`
  - 空闲区域: 白色 `#FFFFFF`
  - 占用区域: 黑色 `#000000`
- 支持缩放和平移操作

### 6.2 3D点云渲染
- 推荐使用`OpenGL ES`进行3D渲染
- 点云着色可根据高度值进行颜色映射
- 支持旋转、缩放、平移操作

### 6.3 性能优化
- 大地图数据使用分块加载
- 点云数据使用LOD（细节层次）技术
- 实现数据缓存机制
- 使用后台线程处理网络数据

## 7. 使用流程

1. **建立连接**: 连接到`ws://192.168.200.216:8000`
2. **接收欢迎**: 等待服务器发送欢迎消息
3. **发送请求**: 根据需要发送地图或点云请求
4. **处理响应**: 解析JSON数据，如有压缩则先解压
5. **渲染显示**: 将地图数据渲染到界面上
6. **实时更新**: 定期请求最新数据保持同步

## 8. 错误处理

- 网络连接失败: 实现重连机制
- 数据解析错误: 记录日志并提示用户
- 服务器错误: 显示错误信息给用户
- 内存不足: 实现数据清理机制
