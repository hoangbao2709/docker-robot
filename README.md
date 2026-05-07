# Docker Robot SLAM/Nav Service

`docker-robot` la service chay trong moi truong ROS2/container de hien thi ban
do SLAM, quan ly saved maps, named points, navigation goal va metrics. Service
nay thuong duoc `dogzilla_server` khoi dong/dung thong qua Docker container
`yahboom_humble`.

## Luu y ve Git

Thu muc nay co `.git` rieng, doc lap voi Git repo o workspace goc va
`dogzilla_server`. Khi commit thay doi trong thu muc nay:

```bash
cd docker-robot
git status
git add README.md
git commit -m "docs: explain docker robot flow"
```

Commit o workspace goc se khong gom thay doi cua repo nay.

## Cach chay

Live SLAM/map mode:

```bash
python3 main.py
```

Navigation/static-map mode:

```bash
python3 main_nav.py
```

Ca hai entrypoint deu:

1. Start HTTP server tren thread rieng.
2. `rclpy.init()`.
3. Tao ROS2 node `LiveMapWeb`.
4. `rclpy.spin(node)` de xu ly topic, TF, action, timer.

## Cau truc file

```text
main.py                   Entry point live SLAM web server
main_nav.py               Entry point navigation/static-map web server
web_server.py             HTTP API/UI cho live map
web_server_nav.py         HTTP API/UI cho navigation mode
ros_node.py               ROS2 node doc map/scan/TF, gui Nav2 goal, render map
shared_state.py           Shared state thread-safe giua ROS thread va HTTP thread
path_planner.py           A* planner tren OccupancyGrid
points_store.py           Luu named points vao named_points.json
metrics_store.py          Ghi metrics ve mission, distance, planner, pose
cartographer_manager.py   Restart/read/write Cartographer config
templates.py              HTML/CSS/JS UI render map
config.py                 Duong dan va ten topic/frame mac dinh
saved_maps/               Thu muc luu map bundle/pbstream/yaml/pgm
```

## Luong hoat dong tong quat

```text
ROS2 /map, /scan, TF map->base_link
  -> ros_node.LiveMapWeb
    -> cap nhat shared_state
    -> render map.png
    -> HTTP server doc shared_state
      -> frontend/backend/dogzilla_server goi API cong 8080
```

Khi user hoac backend yeu cau goal:

```text
HTTP /set_goal_pose hoac /go_to_point
  -> web_server set request/shared state
    -> ros_node xu ly request trong timer
      -> gui Nav2 NavigateToPose action
      -> cap nhat path/status/metrics
```

## ROS2 node

Class chinh:

```text
ros_node.LiveMapWeb
```

Node nay lam cac viec:

- Subscribe `/map` de nhan `OccupancyGrid`.
- Subscribe scan/local/global plan neu topic co san.
- Doc TF `map -> base_link` de lay pose robot.
- Publish path/goal de debug/visualize.
- Goi Nav2 action de dieu huong robot den goal.
- Render `map.png` tu OccupancyGrid.
- Cap nhat `shared_state` cho web server doc.
- Ghi metrics ve path, mission, pose va distance.

Topic/frame mac dinh nam trong `config.py`:

```text
MAP_FRAME=map
ODOM_FRAME=odom
BASE_FRAME=base_link
SCAN_TOPIC=/scan
ODOM_TOPIC=/odom
CMD_VEL_TOPIC=/cmd_vel
```

## HTTP API cong 8080

Server duoc start boi `web_server.py` hoac `web_server_nav.py`.

API doc state/map:

```text
GET /
GET /state
GET /state_light
GET /map.png
GET /metrics
GET /distance
GET /slam_status
```

API navigation:

```text
GET /set_goal_pose?x=<x>&y=<y>&yaw=<yaw>
GET /clear_path
GET /set_initial_pose?x=<x>&y=<y>&yaw=<yaw>
POST /go_to_point
POST /manual_goal
```

API named points:

```text
GET /points
POST /points
POST /delete_point
POST /point_from_obstacle
```

API saved maps:

```text
GET /save_map?name=<name>
GET /maps/<file>
POST /upload_map
GET /use_live_map
```

API metrics:

```text
GET /metrics
GET /metrics/reset
GET /distance
```

## Shared state

`shared_state.py` la cau noi giua ROS thread va HTTP thread. State duoc bao ve
bang lock va gom:

- map version, map info, render info
- robot pose
- goal hien tai
- paths: Nav2 path, A* path, local plan
- scan points
- status: slam/tf/planner/nav2
- request flags: goal, clear, save map, map override

HTTP server khong thao tac truc tiep ROS object; no set request vao shared
state, sau do `LiveMapWeb` xu ly trong timer.

## Saved maps va named points

Named points nam trong:

```text
named_points.json
```

Saved maps nam trong:

```text
saved_maps/
```

Khi load map bundle, web server co the set map override de UI hien thi map da
load trong khi ROS/Cartographer restart voi pbstream tuong ung.

## Quan he voi dogzilla_server va backend

- `dogzilla_server/routes/control.py` bat/tat Docker/ROS stack va goi service
  nay qua cong 8080.
- Django backend goi service nay gian tiep qua `ROSClient` de lay state, map,
  points, manual goal va patrol.
- Frontend khong thuong goi truc tiep cong 8080; no goi backend Django, backend
  proxy sang service nay.

## Debug nhanh

Kiem tra web:

```text
http://<robot-ip>:8080/
http://<robot-ip>:8080/state
http://<robot-ip>:8080/map.png
```

Kiem tra ROS topic:

```bash
ros2 topic list
ros2 topic echo /map
ros2 topic echo /scan
```

Kiem tra TF:

```bash
ros2 run tf2_tools view_frames
```

Neu map khong hien:

- Kiem tra `/map` co du lieu khong.
- Kiem tra `map.png` co duoc ghi moi khong.
- Kiem tra `MAP_FRAME`, `BASE_FRAME` va TF.

Neu navigation khong chay:

- Kiem tra Nav2 action server.
- Kiem tra goal co nam trong map va khong roi vao obstacle.
- Kiem tra `/state` truong `status.planner_msg`.
