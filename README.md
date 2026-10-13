# IntroSSE-Group4

ROS 2 **Jazzy** workspace for robot_4 (iRobot Create 3 + lidar). This repo **is** the workspace:
clone it, then `colcon build` in the repo root.

| Package | Owner | Job |
|---|---|---|
| `bug2_create3` | freek | Round 1: Bug2 start → goal → start |
| `group4_slam` | shah | slam_toolbox mapping (runs during both rounds) + button to start it |
| `group4_navigation` | mina | Round 2: Nav2 on the live SLAM map (NavFn + Regulated Pure Pursuit + velocity smoother + collision monitor) + goal sender |

**Convention:** every node runs in the namespace `robot_4` (set in the launch files) and uses
relative topic names. So everything is `/robot_4/...`: `/robot_4/scan`, `/robot_4/map`,
`/robot_4/navigate_to_pose`, `/robot_4/tf`, and so on.

## Build
```bash
colcon build --symlink-install
source install/setup.bash
```

## One run
```bash
# 1. undock + reset pose
ros2 action send_goal /robot_4/undock irobot_create_msgs/action/Undock "{}"
ros2 service call /robot_4/reset_pose irobot_create_msgs/srv/ResetPose "{}"

# 2. SLAM (keep running until the end), then press button 1 on the robot
ros2 launch group4_slam slam_launch.xml

# 3. round 1: Bug2 (only AFTER SLAM is active: it drives on the SLAM pose, map frame).
#    It logs "Nav2 goal for round 2 (frame map): x=.. y=.." and writes it to
#    ~/bug2_logs/last_goal.json. Ctrl+C when DONE.
#    Without SLAM (testing): add pose_source:=odom
ros2 launch bug2_create3 bug2.launch.py goal_forward:=3.0 goal_left:=0.0

# 4. round 2: Nav2, wait for "Managed nodes are active"
ros2 launch group4_navigation navigation.launch.py

# 5. send Bug2's goal: waits until Nav2 is active, logs the progress, ends with
#    "GOAL REACHED" (exit 0) or the failure. Ctrl+C cancels the goal (robot stops).
ros2 launch group4_navigation send_goal.launch.py
#    other goal (map frame): goal:=3.0,0.0   or with heading: goal:=3.0,0.0,90
```

**Goal at a wall:** Bug2 accepts it "as close as possible" (within 0.30 m). When leg 1 ends,
Bug2 replaces the planned goal on `bug2/goal` and in `last_goal.json` with the spot it actually
**reached** (`"source": "reached"`), so round 2 always gets a free, reachable goal. If round 1
is stopped before the goal, the file still holds the planned goal (`"source": "planned"`).

Goal of `send_goal`, first one found: `goal:=` → `/robot_4/bug2/goal` (only while Bug2 still
runs) → `~/bug2_logs/last_goal.json` (it warns if that file is over an hour old).
By hand, without the script:
```bash
ros2 action send_goal --feedback /robot_4/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 0.0}, orientation: {w: 1.0}}}}"
```
Or click the goal in Foxglove (see below): it publishes to `/robot_4/goal_pose`, which
`bt_navigator` turns into a `navigate_to_pose` goal.

Save the map: `ros2 run nav2_map_server map_saver_cli -f ~/maps/arena -t /robot_4/map`
TF command-line tools need the TF remap, e.g.
`ros2 run tf2_tools view_frames --ros-args -r /tf:=/robot_4/tf -r /tf_static:=/robot_4/tf_static`

## Foxglove
Everything runs on the Pi, and `foxglove_bridge` runs there as a service. In Foxglove:
**Open connection → Foxglove WebSocket → `ws://<pi-ip>:8765`**, then
**Layout → Import from file → `foxglove/group4_layout.json`**. Every panel/topic in it can also be
added by hand (the layout was written without a live robot; fix a panel if it looks off).

| Panel | Shows |
|---|---|
| 3D (display frame `map`) | `/robot_4/map`, `/robot_4/scan`, TF, Bug2 (`bug2/markers`: m-line blue, goal green, hit point red + `HIT_LEFT`/`HIT_RETURN` circles, state text, orange robot outline r = 0.17 m with a line to the front, locked to `base_footprint` so it moves with the robot; `bug2/path`; `bug2/goal`), Nav2 `plan`, RPP `lookahead_point` + `lookahead_collision_arc`, footprint. Costmaps are in the list but hidden (big over wifi). |
| State Transitions | `bug2/state` (GO_TO_GOAL / WALL_FOLLOW / DONE), `bug2/rule` (drive, hit, follow, ...) and `collision_monitor_state.action_type` (0 = free, 3 = approach: the collision monitor is slowing/stopping Nav2) |
| Plot | `cmd_vel` vs `odom` speed (v and w): what Bug2/Nav2 asks vs what the robot does. For Nav2 also add `/robot_4/cmd_vel_nav` (RPP before the velocity smoother). |
| Gauge / Log | battery, `/rosout` |

`local_costmap/published_footprint` only exists while Nav2 runs: during round 1 Foxglove shows
its last (old) message, standing still. Use Bug2's orange outline instead (it stays in Foxglove,
following the TF, as long as Foxglove stays connected).

**Nav2 goal by click:** 3D panel → publish tool (top right) → **Publish pose**, then click + drag
on the map. Goes to `/robot_4/goal_pose` in frame `map` (needs client publishing enabled in the
bridge, which is the default).

**Wifi:** Foxglove only receives the topics that a panel shows, all over the Pi's wifi. Show
costmaps only when you need them. For replay, record on the Pi and open the `.mcap` in Foxglove:
`ros2 bag record -s mcap -o ~/bags/run1 /robot_4/scan /robot_4/odom /robot_4/tf /robot_4/tf_static /robot_4/map /robot_4/cmd_vel /robot_4/bug2/markers /robot_4/bug2/path /robot_4/bug2/state /rosout`

**Checking the lidar TF:** Bug2 logs `LIDAR TF base_footprint -> <frame>: x=.. y=..` at the start
and measures everything from the robot centre (`use_lidar_tf:=false` = scan as it is). Check the
TF itself in Foxglove: display frame `odom`, scan decay time ~10 s, spin the robot in place.
Walls must stay sharp (smeared = wrong x/y offset, rotating walls = wrong yaw).

## Known pitfalls (fixed 2026-10-13, after the first real run)
* **Nav2 got no map:** slam_toolbox publishes `/map` (absolute), not `/robot_4/map`.
  `slam_launch.xml` now remaps `/map → map` (verified on the Pi). Check:
  `ros2 topic info -v /map` must say `Publisher count: 0`. A plain `/map` may still be listed
  when a Foxglove panel subscribes to it (`foxglove_bridge` as the only subscriber): harmless.
* **Nav2 got no odometry:** the Create 3 publishes `/robot_4/odom` best-effort, and Nav2 subscribes
  reliable ("offering incompatible QoS"). `navigation.launch.py` starts `odom_relay`, which
  republishes it as `/robot_4/odom_reliable`, and Nav2 reads that one.
* **Bug2 never reached a goal ~5 cm from a wall** (it hit the wall *behind* the goal again and
  again, never started leg 2, and ended UNREACHABLE). Fixed in `bug2.goal_reachable` /
  `bug2.goal_close_enough`.
* Harmless: `collision_monitor ... Failed to get parameters: 'FootprintApproach.max_points' is not
  initialized` every ~15 s. Something (probably Foxglove's parameter polling) reads a deprecated
  parameter that Nav2 declares without a value.
