# MIMI LIVE MAP - PROJECT STRUCTURE

Folder: mimi_live_map/

Mục đích:
Web viewer nhẹ để xem SLAM map realtime trên Raspberry Pi.
Cho phép:

* hiển thị map
* hiển thị robot pose
* hiển thị path planner
* chọn goal navigation
* save/load map
* xem system status

---

## main.py

Entry point của chương trình.

Chức năng:

* khởi động HTTP web server
* khởi động ROS2 node LiveMapWeb
* chạy rclpy.spin()

Flow:

start_web_server()  -> chạy web
LiveMapWeb()        -> node ROS
spin()              -> vòng lặp ROS

---

## ros_node.py

File quan trọng nhất.

Chứa class:

LiveMapWeb(Node)

Chức năng chính:

1. Subscribe
   /map (OccupancyGrid)

2. TF
   đọc transform:
   map -> base_link

3. Planner
   gọi A* planner:
   plan_path()

4. Publish
   /a_star_path
   /a_star_goal

5. Render map
   OccupancyGrid -> PNG

6. xử lý request từ web

   * goal request
   * clear path
   * save map

Timers:

fast_timer (0.15s)
- process_goal_request
- clear path
- update robot pose

slow_timer (1s)
- render map PNG

---

## web_server.py

HTTP server đơn giản dùng:

http.server.HTTPServer

Chạy port:

8080

API endpoints:

GET /

```
trả về web UI
```

GET /state

```
trả về trạng thái hệ thống JSON
```

GET /map.png

```
trả ảnh map render
```

GET /set_goal_pose

```
gửi goal navigation
```

GET /clear_path

```
clear path
```

GET /save_map

```
yêu cầu ROS save map
```

GET /maps/<file>

```
download map file
```

POST /upload_map

```
upload map file
```

---

## templates.py

Chứa HTML giao diện web.

Hàm chính:

build_index_html()

HTML bao gồm:

UI Modes:
VIEW
NAV

Controls:
save map
load map
reset view
clear path

checkbox:
robot
path
grid

Canvas overlay:
vẽ robot
vẽ path
vẽ grid

---

## shared_state.py

Shared memory giữa:

ROS thread
WEB thread

Dùng lock:

threading.Lock()

Biến chính:

SHARED_STATE

chứa:

map_version
map_info
render_info
pose
goal
paths
status

Ngoài ra có các request flag:

GOAL_REQUEST
CLEAR_REQUEST
SAVE_REQUEST_NAME

---

## utils.py

Các hàm tiện ích:

now_sec()

```
lấy timestamp
```

quat_to_yaw()

```
quaternion -> yaw
```

yaw_to_quat()

```
yaw -> quaternion
```

---

## config.py

Chứa cấu hình đường dẫn.

BASE_DIR

MAP_PNG_PATH

```
file map render
```

MAP_SAVE_DIR

```
folder chứa map đã save
```

---

## saved_maps/

Folder lưu map.

Khi save map sẽ tạo:

map_name.yaml
map_name.pgm

---

## File có thể thiếu

path_planner.py

Chứa thuật toán A*

Hàm:

plan_path()

Được ros_node.py sử dụng để tính đường đi.

---

## Data Flow tổng thể

SLAM (/map)
↓
ROS Node
↓
render map.png
↓
Web UI load map.png
↓
user click goal
↓
HTTP /set_goal_pose
↓
ROS Node nhận request
↓
planner A*
↓
publish path
↓
Web UI vẽ path

---

## Debug nhanh

Nếu map không hiện:

check topic:

ros2 topic echo /map

Nếu robot không hiện:

check TF:

ros2 run tf2_tools view_frames

Nếu web không chạy:

check port:

http://<raspberry_ip>:8080

---

## End of document
