# Final Project — Safety Filter

**[Due 9:00 AM Tuesday, May 5 — Demo Day]**

You will build a ROS 2 safety filter: a node that sits between a human driver
(PS4 joystick) and the vehicle and intervenes when the human's command would
cause a collision or send the car far off the planned route.

There are **2 tasks**. You will develop and test your solution in the
**simulator** on your laptop, but the **live demo on May 5 is on the real
truck only** — no simulator during demo day.

---

## 1. Pull the starter code

If git asks how to reconcile divergent branches (one-time):
```bash
git config pull.rebase false
```

Then pull the upstream changes and rebuild the container:
```bash
git pull upstream SP2026
./start.sh build
```

If you get a **merge conflict** on a file you modified (e.g. your Lab 1
`traj_planner.py`), keep your version:
```bash
git checkout --ours <path-to-conflicting-file>
git add      <path-to-conflicting-file>
git commit
```

Start the container and rebuild the workspace:
```bash
./start.sh down
./start.sh build
./start.sh 
# Inside the container:
cd /ros2_ws && colcon build --symlink-install && source install/setup.bash
```

---

## 2. Set up the PS4 controller

You can connect the controller either by **USB** or **Bluetooth**. USB is
simpler; Bluetooth is required if you want to drive the truck from a distance.

### Option A — USB (easy)

Plug the controller into your laptop with a micro-USB cable. Nothing else to
configure.

### Option B — Bluetooth (`bluetoothctl`)

Inside the container:
```bash
bluetoothctl
# Inside the prompt:
power on
agent on
scan on          # wait ~5 s
# Put the controller into pairing mode: hold SHARE + PS until the light flashes fast.
# You should see a line like: "Wireless Controller" with a MAC address.
pair  <MAC>
trust <MAC>
connect <MAC>
exit
```

**Full tutorial / troubleshooting**:
<https://wiki.archlinux.org/title/Gamepad#Sony_DualShock_controllers>

### Verify the controller is visible

```bash
ls /dev/input/js0
# expect: /dev/input/js0
```

If that file doesn't exist, the controller is not paired/connected — go back
to Option A or redo Option B. You can also confirm the sticks work:

```bash
jstest /dev/input/js0
# move sticks / press buttons → values change
```

Once `/dev/input/js0` exists, the ROS `joy_node` will pick it up automatically
when you launch the project.

---

## 3. Launch the project in simulation
Note: TRUCK_IP here doesn't matter. The purpose of this command is to set DOMAIN_ID

```bash
source setup_cyclone.sh <Truck_IP> <DOMAIN_ID>
ros2 launch racecar_ece346 final_project_simulation_launch.py
```

This brings up, in one window:

- `simulator_node` — the Bicycle4D vehicle simulator
- `traffic_simulation_node` — the track and routing services
- `visualization_node` + RViz — 3D view
- `joy_node` + `joy_to_ackermann_node` — your controller → `/teleop`
- `safety_filter_node` — **your node** 
- `drive_to_servo_node` — sim bridge, translates `/drive` → simulator
- `static_obstacle_publisher_node` — RViz click-to-place obstacles

Until you finish Task 1 the car won't move — the safety filter node doesn't publish to
`/drive`.

---

## 4. Task 1 — Wire the node (subscribers, publisher, timer)

Open [`scripts/safety_filter_node.py`](scripts/safety_filter_node.py) and look
for the lines tagged `TODO(Task 1)`

Fill in `__init__` and the small callbacks so that the node:

1. Declares ROS parameters for each topic name and the publish rate. The
   launch yaml is already written to pass these values in; match the names:
   - `teleop_topic`, `drive_topic`, `odom_topic`, `static_obs_topic`,
     `publish_rate`.
2. Creates three subscribers, each with a one-line callback that stores the
   latest message in an instance attribute:
   - `/teleop` — `ackermann_msgs/AckermannDriveStamped` (human input)
   - `<odom_topic>` — `nav_msgs/Odometry` (vehicle state)
   - `/Obstacles/Static` — `visualization_msgs/MarkerArray` (obstacles)
3. Creates one publisher on `/drive` — `AckermannDriveStamped`.
4. Creates a timer at `publish_rate` Hz whose callback invokes
   `self.safety_filter(...)` and publishes the returned command on `/drive`.

Hints:
- `self.declare_parameter('name', default)` for parameters.
- `self.create_subscription(MsgType, topic, callback, queue_size)` for subs.
- `self.create_publisher(MsgType, topic, queue_size)` for the publisher.
- `self.create_timer(period_sec, callback)` for the timer.
- Look at any other node in this repo (e.g. `joy_to_ackermann_node.py`) if
  you need to see the pattern.

Leave `safety_filter()` as the default **passthrough** for Task 1 — it just
returns the human's teleop command.

### How to verify Task 1

Relaunch the simulation. With no code changes to anything else, pushing the
joystick should now move the car. Confirm the full pipeline:
```bash
ros2 topic hz /teleop   # ~20 Hz while you move sticks
ros2 topic hz /drive    # ~30 Hz once Task 1 works
```

---

## 5. Task 2 — Implement the safety filter

Fill in the body of `safety_filter(teleop, odom, obstacles)`. The goal: return
a command that is *as close as possible to the human's command* while keeping
the car safe. You have three inputs and one output:

- `teleop`  — `AckermannDriveStamped` — what the human wants (`speed`, `steering_angle`)
- `odom`    — `Odometry` — where the car is and how fast it's going
- `obstacles` — `MarkerArray` — static obstacles the car should avoid
- Return   — `AckermannDriveStamped` published on `/drive`

The full argument field reference is documented in the docstring inside
`safety_filter_node.py` (every useful field listed with units).


You may add new files (e.g. an `ilqr/` subfolder with your solver), edit
**any** yaml under `config/`, and declare additional ROS parameters on the
node. You should NOT need to modify the launch files, the other
nodes (`joy_to_ackermann`, `drive_to_servo`, etc.), or the top-level
`CMakeLists.txt` — if you think you do, ask a TA first.

---

## 6. Placing obstacles with "Publish Point" in RViz

In simulation you can drop obstacles at runtime using RViz's **"Publish
Point"** tool.

1. In the RViz toolbar (top of the 3D view), click the button labeled
   **Publish Point** (last tool in the row). The cursor becomes a crosshair
   when hovering over the map.
2. Click anywhere on the map — a blue cube appears at that location and is
   added to `/Obstacles/Static` (the same topic your filter subscribes to).
3. The launch terminal logs `Added obstacle #N at (x, y)`.

You can pre-seed obstacles by editing
[`config/static_obstacles.yaml`](config/static_obstacles.yaml):

```yaml
obstacles:
  - [2.5, 1.0]
  - [4.0, 2.3]
  - [1.2, 3.1]
```

---

## 7. Where are the lane boundaries?

The routing node publishes the planned route on `/Routing/Path`
(`nav_msgs/Path`). Each waypoint encodes **centerline + lane widths +
speed limit** packed into the message (the `orientation` fields are
reused for metadata — see `racecar_routing/routing/routing.py:190-195`):

| Field of `path.poses[i].pose` | Meaning | Units |
|---|---|---|
| `position.x`, `position.y` | centerline point in map frame | m |
| `orientation.x` | lane width to the LEFT of centerline | m |
| `orientation.y` | lane width to the RIGHT of centerline | m |
| `orientation.z` | speed limit for this lanelet | m/s |

Concretely, for a waypoint `wp`:
```python
x  = wp.pose.position.x
y  = wp.pose.position.y
wL = wp.pose.orientation.x   # half-width on the left
wR = wp.pose.orientation.y   # half-width on the right
v_limit = wp.pose.orientation.z
```

This lets you compute, for any car position `(px, py)`:
- the closest waypoint index (scan the path),
- your signed lateral deviation from the centerline, and
- how close you are to either lane edge (`wL - deviation` on the left,
  `wR + deviation` on the right).

If you want the raw lanelet2 map itself (with all lanelets, not just the
current route), look at `racecar_routing/routing/lanelet_wrapper.py` — it
exposes helpers like `get_shortest_path`, `get_lanelet_speed_limit`, etc.

### Seeing the current route

Set a goal in RViz: click the **"2D Goal Pose"** tool, then click on the
track. The routing node plans a route and publishes it on `/Routing/Path`;
you'll see the route drawn as a red line in RViz.

---

## 8. Running on the real truck

On demo day you will deploy your filter on the physical truck. This requires
**three terminals**: **two on the truck via SSH**, and **one on your host
laptop** inside the container. Bring up the truck-side pieces *first*, then
your host-side launch.

### Prerequisites

- Truck is powered on, battery connected, Jetson booted.
- Your laptop is on the **same WiFi network** as the truck.
- Know your host IP (`hostname -I`) and the truck IP (`192.168.1.2XX`, where
  `XX` is your Jetson ID).
- Know your group's **`<DOMAIN_ID>`** (a number between 0–101 assigned by
  the TAs).

### One-time: pair the PS4 controller to the truck

The PS4 controller pairs to the **Jetson** (not your laptop). The pairing
persists across reboots — you only need to do this once per truck/controller
pair.

SSH into the truck and start `bluetoothctl`:
```bash
ssh nvidia@192.168.1.2XX
bluetoothctl
```

Plug the controller into the Jetson over USB. You'll be prompted to authorize
the HID service — type `yes`:
```
[agent] Authorize service 00001124-0000-1000-8000-00805f9b34fb (yes/no): yes
```

Unplug the controller, then press the **PS** button. The LED bar blinks while
connecting and stays solid once paired. Type `exit` to leave `bluetoothctl`.

**Troubleshooting** — if the LED keeps blinking and never connects, from
inside `bluetoothctl`:
```
scan on
```

If `scan on` returns `Failed to start discovery: org.bluez.Error.NotReady`,
exit and unblock the radio, then retry:
```bash
exit
sudo rfkill unblock bluetooth
bluetoothctl
# press PS button
```

### Terminal 1 (truck) — start SLAM

```bash
ssh nvidia@192.168.1.2XX        # password: nvidia
cd ~/ece346_truck
./start_slam.sh
```

Wait until SLAM reports it's publishing pose. Leave this running.

### Terminal 2 (truck) — start F1Tenth stack + ZMQ bridge

```bash
ssh nvidia@192.168.1.2XX
cd ~/ece346_truck
source setup_cyclone.sh <HOST_IP> <DOMAIN_ID>
ros2 launch f1tenth_stack bringup_launch.py
```

The first command configures CycloneDDS so the truck and your host can see
each other's topics. The second actually launches the joystick driver, VESC
motor controller, ackermann mux, `control_gate`, and the ROS 2 ZMQ bridge
that republishes SLAM pose + AprilTag detections as ROS 2 topics on your
network. Leave this terminal running.

**Quick test**: with the PS4 controller paired to the truck, you should be
able to teleop by holding **L2** and using the sticks. If the truck doesn't
respond, don't start the host side — fix the truck first.

### Terminal 3 (host laptop) — start your safety filter

In the container:

```bash
cd /ros2_ws
source /ros2_ws/setup_cyclone.sh 192.168.1.2XX <DOMAIN_ID>
```

Use the **same `<DOMAIN_ID>`** you used on the truck. Verify truck topics
are reachable:

```bash
ros2 topic list
# expect to see: /SLAM/Pose, /SLAM/Tag_Detections_Dynamic, /joy, /drive, ...
```

If those topics don't appear, CycloneDDS isn't talking between the two
machines — re-check `<DOMAIN_ID>` and `<HOST_IP>`.

When topics are visible, launch the truck variant of the final project:

```bash
ros2 launch racecar_ece346 final_project_truck_launch.py
```

This brings up the same node layout as sim **minus** the simulator and the
sim-only bridge — your `safety_filter_node.py` code and config yamls are
identical between the two environments. `obstacle_detection_node` reads AprilTag detections instead
of click-to-place obstacles, and f1tenth_stack's `control_gate` consumes
your `/drive` directly. Your filter code and config are identical to sim.

### Controller gating on the truck

The truck's `control_gate` uses the PS4 for a deadman switch:

- **L2 (held)** — manual teleop (bypass your filter entirely).
- **R1 (held)** — autonomous mode (your filter's `/drive` is forwarded).
- **Neither held** — the truck does not move (safe default).

### Demo day expectations

Be prepared to show:
- A clean drive where the filter passes through your commands.
- The filter stopping / correcting when you aim at an AprilTag obstacle.
- An explanation of your approach and the threshold values you chose.

---