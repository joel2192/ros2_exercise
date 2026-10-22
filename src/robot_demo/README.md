# ROS 2 Jazzy C++ Practice

使用 **ROS 2 Jazzy、C++ 與 Docker** 建立的 ROS 2 練習專案，透過實際範例熟悉 ROS 2 的通訊機制、節點管理、參數設定、感測器資料處理，以及單元測試。

目前以純模擬環境為主，不依賴實體機器人硬體。

## Features

-  Publisher / Subscriber
-  Service / Client
-  Action Server / Client
-  ROS 2 Parameters
-  Launch System
-  YAML Parameter Configuration
-  Lifecycle Node
-  Component Node
-  LiDAR Simulation
-  QoS Configuration
-  Obstacle Detection
-  C++ Unit Tests with `ament_cmake_gtest`

## Learning Topics

| 範例 | 課題 |
|---|---|
| Publisher / Subscriber | 使用 Topic 進行節點間的非同步通訊 |
| Service / Client | 使用 Request / Response 執行請求與回應 |
| Action Server / Client | 處理長時間任務、Feedback、Result 與 Cancel |
| Parameters | 宣告、讀取及動態修改 Node Parameters |
| Launch | 透過 Python Launch 同時啟動多個 Node |
| YAML | 將參數設定與程式碼分離 |
| Lifecycle | 管理 Node 的 Configure、Activate、Deactivate 與 Cleanup 狀態 |
| Components | 將 Node 編譯成可動態載入的 Component |
| LiDAR | 使用 `sensor_msgs/msg/LaserScan` 模擬感測器資料 |
| QoS | 理解 Reliability、History 與 Sensor Data QoS 的使用情境 |
| Obstacle Detection | 從 LaserScan 過濾有效距離，計算最近障礙物並發布警告 |
| Unit Testing | 使用 GoogleTest 驗證獨立的 C++ 邏輯 |

## Environment

| 項目 | 設定 |
|---|---|
| OS | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| Language | C++ |
| Build System | CMake / colcon |
| Test Framework | GoogleTest / `ament_cmake_gtest` |
| Development Environment | Docker on macOS Apple Silicon |


## Examples

### 1. Publisher / Subscriber

**目的：** 熟悉 ROS 2 Topic 通訊。

- `demo_node.cpp`：持續發布 `std_msgs/msg/String` 訊息。
- `listener_node.cpp`：訂閱訊息並輸出接收到的內容。

**Terminal 1 — Publisher**

```bash
ros2 run robot_demo demo_node
```

**Terminal 2 — Subscriber**

```bash
ros2 run robot_demo listener_node
```

**檢查 Topic**

```bash
ros2 topic list
ros2 topic info /chatter
ros2 topic echo /chatter
ros2 topic hz /chatter
```

### 2. Service / Client

**目的：** 熟悉同步請求與回應的通訊模式。

使用 `example_interfaces/srv/AddTwoInts`，由 Client 傳入兩個整數，Server 計算總和並回傳結果。

**Terminal 1 — Server**

```bash
ros2 run robot_demo add_server
```

**Terminal 2 — Client**

```bash
ros2 run robot_demo add_client
```

**檢查 Service**

```bash
ros2 service list
ros2 service type /add_two_ints
ros2 interface show example_interfaces/srv/AddTwoInts
```

### 3. Action Server / Client

**目的：** 熟悉長時間任務的執行、進度回報與取消機制。

使用自訂的 `CountUntil.action`：

```text
int32 target
---
int32 result
---
int32 progress
```

- Goal：指定計數目標。
- Result：回傳最終計數結果。
- Feedback：回報目前的計數進度。
- Cancel：允許 Client 取消執行中的任務。

**Terminal 1 — Action Server**

```bash
ros2 run robot_demo count_server
```

**Terminal 2 — Action Client**

```bash
ros2 run robot_demo count_client
```

**檢查 Action**

```bash
ros2 action list
ros2 action info /count_until
```

### 4. Parameters

**目的：** 熟悉 Node 參數的宣告、讀取與動態修改。

`count_server.cpp` 使用 `count_delay` 控制計數間隔，並可透過 YAML 載入不同設定。

**啟動 Server**

```bash
ros2 run robot_demo count_server
```

**查看及修改參數**

```bash
ros2 param list /count_server
ros2 param get /count_server count_delay
ros2 param set /count_server count_delay 0.2
```

### 5. Launch System

**目的：** 使用 Launch 檔案管理多個 Node 的啟動流程。

`count_demo.launch.py` 示範：

- 同時啟動多個 Node。
- 使用 Namespace 隔離不同實例。
- 載入 YAML 參數。
- 透過 Remapping 修改通訊名稱。

**執行**

```bash
ros2 launch robot_demo count_demo.launch.py
```

**檢查執行狀態**

```bash
ros2 node list
ros2 action list
ros2 action info /robot1/count_until
ros2 action info /robot2/count_until
```

### 6. Lifecycle Node

**目的：** 熟悉 Managed Node 的生命週期管理。

`lifecycle_demo.cpp` 使用 `rclcpp_lifecycle::LifecycleNode`，透過 Lifecycle Transition 控制節點狀態，並使用 Lifecycle Publisher 發布訊息。

**啟動**

```bash
ros2 run robot_demo lifecycle_demo
```

**查看狀態**

```bash
ros2 lifecycle nodes
ros2 lifecycle get /lifecycle_demo
```

**執行生命週期轉換**

```bash
ros2 lifecycle set /lifecycle_demo configure
ros2 lifecycle set /lifecycle_demo activate
ros2 lifecycle set /lifecycle_demo deactivate
ros2 lifecycle set /lifecycle_demo cleanup
```

**檢查發布內容**

```bash
ros2 topic echo /lifecycle_chatter
```

### 7. Component Node

**目的：** 熟悉將 Node 編譯為 Component，並動態載入至 Component Container。

`hello_component.cpp` 定義 `robot_demo::HelloComponent`，發布訊息至 `/component_chatter`。

**Terminal 1 — 啟動 Container**

```bash
ros2 run rclcpp_components component_container_mt
```

**Terminal 2 — 載入 Component**

```bash
ros2 component list
ros2 component load /ComponentManager robot_demo robot_demo::HelloComponent
```

**檢查結果**

```bash
ros2 node list
ros2 topic echo /component_chatter
```

> 注意：Component Manager 的名稱可能依 Container 設定而不同。請先使用 `ros2 component list` 確認實際名稱。

### 8. LiDAR Simulation

**目的：** 模擬感測器發布資料，熟悉 `sensor_msgs/msg/LaserScan` 與感測器常用 QoS。

`lidar_sim.cpp` 以固定頻率發布模擬的 LiDAR 資料至 `/scan`；`lidar_sub.cpp` 負責訂閱並檢查資料。

**Terminal 1 — Simulator**

```bash
ros2 run robot_demo lidar_sim
```

**Terminal 2 — Subscriber**

```bash
ros2 run robot_demo lidar_sub
```

**檢查資料**

```bash
ros2 topic info /scan --verbose
ros2 topic echo /scan
ros2 topic hz /scan
```

### 9. Obstacle Detection

**目的：** 建立完整的感測器資料處理流程，從 LiDAR 資料判斷最近障礙物並發布警告。

`obstacle_detector.cpp`：

1. 訂閱 `/scan`。
2. 過濾無效的距離資料。
3. 計算最近的有效距離。
4. 發布最近障礙物距離至 `/obstacle_distance`。
5. 根據 `warning_distance` 判斷是否發布警告至 `/obstacle_alert`。

相關檔案：

- `lidar_sim.cpp`：模擬感測器。
- `obstacle_detector.cpp`：障礙物偵測。
- `obstacle_demo.launch.py`：啟動整個範例。
- `obstacle_params.yaml`：設定警告距離。
- `obstacle_utils.hpp`：可獨立測試的距離計算邏輯。

**啟動完整範例**

```bash
ros2 launch robot_demo obstacle_demo.launch.py
```

**檢查輸出**

開啟另一個 Terminal，載入 ROS 2 與 Workspace 環境後執行：

```bash
ros2 topic echo /obstacle_distance
ros2 topic echo /obstacle_alert
```

**查看參數**

```bash
ros2 param get /obstacle_detector warning_distance
ros2 param set /obstacle_detector warning_distance 1.5
```

`warning_distance` 預設值由 `obstacle_params.yaml` 設定。

## Testing

### Unit Tests

使用 `ament_cmake_gtest` 與 GoogleTest 測試 `obstacle_utils.hpp` 中的獨立 C++ 邏輯。

目前涵蓋以下情況：

- 正常有效資料：找出最小距離。
- 包含無效資料：過濾 NaN、無限值及超出有效範圍的資料。
- 所有資料皆無效：正確回傳失敗結果。

**執行測試**

```bash
cd ~/ros2_ws

colcon test --packages-select robot_demo
```

**查看測試結果**

```bash
colcon test-result --verbose
```

目前 `test_obstacle_utils.cpp` 的 3 個測試案例已確認通過。這不代表整個 Workspace 的所有 lint 或其他檢查均已通過。
