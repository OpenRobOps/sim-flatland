# Flatland Nav2 Docker Environment

Docker environment for running the [avidbots/flatland](https://github.com/avidbots/flatland) 2D simulator with the full Nav2 navigation stack on ROS2 Jazzy.

## What's Included

- **Flatland Server** - 2D physics simulator (built from source with Jazzy compatibility patches)
- **Nav2** - Full navigation stack (planner, controller, BT navigator, AMCL, recoveries)
- **Map Server** - Serves occupancy grid maps
- **Rviz2** - Visualization with preconfigured layout
- **Battery Simulation** - Velocity-based battery drain plugin with `sensor_msgs/BatteryState` output
- **ROS2 Agent** - Connects the simulated robot to OpenRobOps (runs in a sidecar container, optional)
- **ROS Diagnostics** - Health monitoring of sensors, battery, TF, and Nav2 lifecycle published on `/diagnostics` and grouped on `/diagnostics_agg`
- **Sample world** - 20x20m multi-room office with a differential-drive robot (laser, odometry, battery)

![Demo of Flatland Nav2](demo.gif)

## Prerequisites

- Docker and Docker Compose
- For rviz: X11 or Wayland display server on the host (rviz uses XWayland on Wayland desktops)

## Quick Start

### Build the image

```bash
git submodule update --init        # first time only: fetch upstream flatland
docker compose build
```

### Run with rviz (default)

```bash
# Allow X11 access for GUI (required on both X11 and Wayland desktops)
xhost +local:docker

# Start the simulation with rviz
docker compose up
```

### Run headless (no GUI)

```bash
docker compose run --rm flatland-nav2 --no-rviz
```

### Run without the InOrbit agent

An InOrbit agent sidecar is enabled by default (via `COMPOSE_PROFILES=agent` in `.env`). 
The InOrbit agent is compatible with OpenRobOps (an option to run the OpenRobOps agent will replace this in the future).

To skip it:

```bash
COMPOSE_PROFILES= docker compose up
```

`docker compose run --rm flatland-nav2 ...` already skips the agent, since `run` only starts the named service and its dependencies.

### Run rviz separately

Start the simulation headless in one terminal, then connect rviz from another container:

```bash
# Terminal 1: headless simulation
docker compose run --rm flatland-nav2 --no-rviz

# Terminal 2: rviz only (connects to running simulation)
docker compose --profile rviz-separate up rviz
```

### Shell access

```bash
docker compose run --rm flatland-nav2 --shell
```

## Display and GPU

### How it works

Rviz2 uses OGRE for 3D rendering, which requires GLX (an X11 protocol). This means:

- **X11 desktops**: Works directly via X11 socket forwarding.
- **Wayland desktops**: Works via XWayland. Most Wayland compositors (GNOME, KDE, Cosmic) run an XWayland server automatically and set the `DISPLAY` environment variable.

The container mounts `/dev/dri` for GPU-accelerated rendering (Mesa/DRI). The entrypoint auto-detects the display and forces the `xcb` (X11) Qt platform plugin since OGRE requires GLX.

### NVIDIA GPU

By default the container renders with Mesa which is enough for this simulation on Intel and AMD GPUs. 
To render on an NVIDIA GPU, whether it is the discrete card in hybrid graphics system or the primary GPU in the machine, install the
[NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
and apply the `docker-compose.nvidia.yml` overlay:

```bash
xhost +local:docker
docker compose -f docker-compose.nvidia.yml up
```
The overlay sets `runtime: nvidia` plus the PRIME render-offload
variables on both rviz services.

The same `-f docker-compose.nvidia.yml` flag works with the other commands
above too, such as the separate-rviz workflow:

```bash
docker compose -f docker-compose.nvidia.yml --profile rviz-separate up rviz
```

## Using Navigation

1. Start the simulation with rviz (`docker compose up`)
2. AMCL automatically sets the initial pose at the robot's spawn position - no manual step needed
3. Use **Nav2 Goal** in rviz to send navigation goals - the robot will plan and navigate autonomously

To re-localize the robot manually, use rviz's **2D Pose Estimate** tool.

## Entrypoint Modes

| Command | Description |
|---------|-------------|
| `--with-rviz` | Launch simulation + Nav2 + rviz2 (default) |
| `--no-rviz` | Launch simulation + Nav2 headless |
| `--rviz-only` | Launch only rviz2 (connect to existing simulation) |
| `--shell` | Drop into a bash shell |

## Customization

### Custom maps and worlds

Map and world files are bind-mounted from the host. Edit them in place:

- `maps/` - Occupancy grid maps (`.yaml` + `.pgm`)
- `worlds/` - Flatland world and model definitions
- `config/` - Nav2 parameters and rviz layout

Override at launch time:

```bash
docker compose run --rm flatland-nav2 --with-rviz \
  world_path:=/ros2_ws/worlds/my_world.yaml \
  map_path:=/ros2_ws/maps/my_map.yaml
```

### Nav2 parameters

Edit `config/nav2_params.yaml` to tune navigation behavior (controller, planner, costmaps, AMCL, etc.). Changes take effect on next container start since the config directory is bind-mounted.

## Battery Simulation

The robot includes a battery plugin that simulates finite energy. The battery drains at a base idle rate and faster when the robot moves. When the battery reaches 0%, the robot stops. The battery recharges automatically when the robot enters a charging zone, or manually via ROS2 services.

Battery state is visible in rviz as a floating text marker above the robot (green when healthy, orange below 20%, cyan when charging, red when depleted). Charging zones appear as blue circles on the map.

### Configuration

Parameters in `worlds/turtlebot.model.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `capacity_ah` | 5.0 | Battery capacity in Amp-hours |
| `voltage_full` | 12.6 | Voltage at 100% charge |
| `voltage_empty` | 10.0 | Voltage at 0% charge |
| `base_current` | 0.5 | Idle current draw (Amps) |
| `linear_current_coeff` | 2.0 | Additional Amps per m/s of linear speed |
| `angular_current_coeff` | 0.5 | Additional Amps per rad/s of angular speed |
| `charge_current` | 2.0 | Charging current (Amps) |
| `initial_charge` | 1.0 | Starting charge fraction (0.0 - 1.0) |
| `pub_rate` | 1.0 | Publish rate in Hz |

### Charging Zones

Charging zones are circular regions defined in the robot model YAML. When the robot enters a zone, charging starts automatically:

```yaml
charging_zones:
  - name: "Charger B (Lab)"
    x: 15.0
    y: 15.0
    radius: 0.6
  - name: "Charger D (Lobby)"
    x: 17.0
    y: 3.0
    radius: 0.6
```

The sample world includes two chargers: one in the Lab (Room B) and one in the Lobby (Room D). Navigate the robot to either location to recharge.

### ROS2 Services

| Service | Type | Description |
|---------|------|-------------|
| `/set_charging` | `std_srvs/SetBool` | Manually enable/disable charging (overrides zone detection) |
| `/reset_battery` | `std_srvs/Trigger` | Instantly reset battery to 100% |

Examples:

```bash
# Monitor battery state
ros2 topic echo /battery_state

# Manually enable charging
ros2 service call /set_charging std_srvs/srv/SetBool '{data: true}'
ros2 topic pub --once /inorbit/custom_command std_msgs/msg/String '{data: "charge"}'
ros2 topic pub --once /inorbit/custom_command std_msgs/msg/String '{data: "discharge"}'

# Reset battery to full
ros2 service call /reset_battery std_srvs/srv/Trigger '{}'
# or
ros2 topic pub --once /inorbit/custom_command std_msgs/msg/String '{data: "reset"}'

# Dock at the nearest charging zone
ros2 topic pub --once /inorbit/custom_command std_msgs/msg/String '{data: "dock"}'

# Dock at a specific charger by id (letter from the zone name, case-insensitive)
ros2 topic pub --once /inorbit/custom_command std_msgs/msg/String '{data: "dock=A"}'
```

## Camera Simulation

The robot includes a synthetic forward-facing camera that renders a Wolfenstein
3D-style image of the 2D world using one Box2D raycast per image column.
The image is depth-shaded, with a solid sky and floor split at the horizon.

### Viewing the image

The camera publishes to ROS topics regardless of whether anyone is watching;
to see the rendered frame in a window, run `rqt_image_view` from inside the
container. As with rviz, the host must first authorize X access:

```bash
# On the host (once per login session — same step rviz needs)
xhost +local:docker

# In another terminal, with `docker compose up` already running
docker compose exec flatland-nav2 bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  source /ros2_ws/install/setup.bash &&
  ros2 run rqt_image_view rqt_image_view /image_raw'
```

An `rviz_default_plugins/Image` display is **not** added to the default rviz
layout: that display's render-to-texture path crashes rviz on some OpenGL /
OGRE setups commonly seen in containerized environments. To try it in rviz
anyway, click *Add* → *By topic* → `/image_raw` → *Image* in a running rviz
session; if rviz survives, save the layout.

### Configuration

Parameters in `worlds/turtlebot.model.yaml` (all optional — defaults shown):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `width` | 320 | Image width in pixels |
| `height` | 240 | Image height in pixels |
| `fov_deg` | 90.0 | Horizontal field of view (degrees) |
| `range` | 8.0 | Max ray distance in meters |
| `update_rate` | 10.0 | Publish rate in Hz |
| `origin` | `[0, 0, 0]` | Camera mount offset `[x, y, theta]` relative to body |
| `layers` | `["all"]` | Box2D collision layers the camera sees |
| `ignore_self` | true | Skip the camera's own model when raycasting |
| `wall_height` | 1.0 | Virtual wall height (m), affects column projection |
| `eye_height` | 0.5 | Camera eye height (m); shifts horizon |
| `shade_min` / `shade_max` | 0.15 / 1.0 | Brightness at far / near distance |
| `directional_shading` | 0.85 | Multiplier for grazing-angle hits (1.0 disables) |
| `sky_color`, `floor_color`, `fog_color` | gray-ish defaults | RGB `[0-255]` triples |
| `broadcast_tf` | false | Publish `base_link → camera_link` transform |
| `publish_camera_info` | true | Publish synthetic `sensor_msgs/CameraInfo` |
| `publish_compressed` | true | Publish JPEG-compressed image |
| `jpeg_quality` | 75 | JPEG quality (1-100) when compressed publishing is on |

### Topics

| Topic | Type | Notes |
|-------|------|-------|
| `/image_raw` | `sensor_msgs/Image` | `rgb8` encoding |
| `/image_raw/camera_info` | `sensor_msgs/CameraInfo` | Static intrinsics (plumb_bob, zero distortion) |
| `/image_raw/compressed` | `sensor_msgs/CompressedImage` | JPEG, on if `publish_compressed: true` |

### Verifying the camera

```bash
# Topics exist and publish at the configured rate
docker compose exec flatland-nav2 ros2 topic hz /image_raw            # ~10 Hz
docker compose exec flatland-nav2 ros2 topic hz /image_raw/compressed
docker compose exec flatland-nav2 ros2 topic echo /image_raw/camera_info --once

# Image fields match config
docker compose exec flatland-nav2 ros2 topic echo /image_raw --field width    --once  # 320
docker compose exec flatland-nav2 ros2 topic echo /image_raw --field height   --once  # 240
docker compose exec flatland-nav2 ros2 topic echo /image_raw --field encoding --once  # rgb8

# Open rqt_image_view (see "Viewing the image" above) and drive the robot
# toward a wall — walls grow and brighten as you approach.
```

## InOrbit Agent

An [InOrbit ROS2 agent](https://www.inorbit.ai/) runs as a sidecar container (`inorbitai/agent:ros-jazzy-4.33.0`) alongside the simulation, connecting the simulated robot to your OpenRobOps or InOrbit instance. A second lightweight `busybox` container tails the agent log into the main `docker compose` output for easier debugging. Both services are part of the `agent` Compose profile, enabled by default via `.env`.

### Configuration

Copy the example env file and fill in your InOrbit credentials:

```bash
cp local/agent.env.sh.example local/agent.env.sh
# edit local/agent.env.sh and set INORBIT_KEY to match your one of your robotApiKeys. 
# Optionally change INORBIT_ID; the ID of the robot.
# Other variables such as INORBIT_URL do not need to be changed.
```

The agent reads this file on startup. The entire `local/` directory is bind-mounted into the agent container at `/root/.inorbit/local/` and is gitignored, so runtime state (logs, cache) stays on the host.

### Disabling the agent

The agent is opt-out. To skip both the agent and its log tail:

```bash
COMPOSE_PROFILES= docker compose up
```

## Diagnostics

A `diagnostics_watcher` node publishes ROS 2 diagnostics on `/diagnostics`, and a `diagnostic_aggregator` groups them into a tree on `/diagnostics_agg`. Both are launched by default with the rest of the simulation.

### What is monitored

| Check | Trigger |
|-------|---------|
| `scan_freshness` | `/scan` not received for > 2 s (ERROR), or rate < 5 Hz (WARN) |
| `odom_publishers` | No publisher on `/odom` (ERROR) |
| `tf_broadcasters` | No publisher on `/tf` (ERROR) |
| `battery` | SOC ≤ 5% or message stale > 5 s (ERROR); SOC ≤ 20% (WARN); NaN percentage (WARN) |
| `cmd_vel_freshness` | `/cmd_vel` not received for > 5 s (WARN only — the robot is idle when no goal is active) |
| `nav2_lifecycle` | Any of `map_server`, `amcl`, `controller_server`, `planner_server`, `bt_navigator` not in state `active` (ERROR) |

During the first 5 s after the watcher starts, missing-message conditions report `STALE` instead of `ERROR` to avoid noise during bringup.

### Topics

| Topic | Type | Source |
|-------|------|--------|
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `diagnostics_watcher` |
| `/diagnostics_agg` | `diagnostic_msgs/DiagnosticArray` | `diagnostic_aggregator` (grouped: Sensors / Power / Navigation) |
| `/diagnostics_toplevel_state` | `diagnostic_msgs/DiagnosticStatus` | `diagnostic_aggregator` (single overall status) |

### Viewing diagnostics

```bash
# One-shot snapshot of the grouped tree
docker compose exec flatland-nav2 ros2 topic echo /diagnostics_agg --once

# Live GUI tree (requires X11/Wayland forwarding)
docker compose exec flatland-nav2 ros2 run rqt_robot_monitor rqt_robot_monitor
```

### Tuning thresholds

All thresholds and topic names are ROS parameters on the `diagnostics_watcher` node. Override them at launch time, e.g.:

```bash
ros2 param set /diagnostics_watcher scan_stale_sec 5.0
ros2 param set /diagnostics_watcher battery_warn_soc 0.30
```

The full list (`scan_stale_sec`, `cmd_vel_stale_sec`, `battery_stale_sec`, `battery_warn_soc`, `battery_critical_soc`, `nav2_nodes`, `update_rate_hz`, `startup_grace_sec`) lives in `diagnostics_watcher/diagnostics_watcher/watcher_node.py`.

### Disabling

Comment out the `diagnostics_watcher` and `diagnostics_aggregator` `Node` entries (plus their references in the returned `LaunchDescription`) in `launch/flatland_nav2.launch.py`.

## File Structure

```
flatland/
  Dockerfile                        # Multi-layer build: Box2D + flatland + Nav2
  entrypoint.sh                     # Mode-switching entrypoint with display auto-detection
  docker-compose.yml                # X11/Wayland forwarding, GPU, host networking, InOrbit agent
  .env                              # COMPOSE_PROFILES=agent (default profiles for docker compose)
  config/
    nav2_params.yaml                # Nav2 parameters (DWB controller, NavFn planner)
    flatland_rviz.rviz              # Rviz2 layout (map, scan, TF, costmaps, Nav2 panel, battery)
    diagnostics_aggregator.yaml     # Analyzer groups for diagnostic_aggregator
  launch/
    flatland_nav2.launch.py         # Unified launch file
  plugins/
    battery.h                       # Battery simulation plugin header
    battery.cpp                     # Battery simulation plugin implementation
  republisher/                      # Project ROS2 package - InOrbit custom_data bridge
  diagnostics_watcher/              # Project ROS2 package - /diagnostics publisher
  maps/
    sample_map.yaml                 # Map metadata
    sample_map.pgm                  # 20x20m multi-room office occupancy grid
  worlds/
    sample.world.yaml               # Flatland world definition
    turtlebot.model.yaml            # Robot model (DiffDrive + Laser + Battery + TF)
  patches/
    flatland/                       # Unified-diff patches applied to upstream flatland at build time
      0001-*.patch ... 0012-*.patch # One patch per concern; 0011-0012 are project-specific
  third_party/
    flatland/                       # Git submodule -> avidbots/flatland @ pinned SHA
  local/                            # Bind-mounted into agent container (gitignored)
    agent.env.sh                    # InOrbit agent credentials (INORBIT_KEY, INORBIT_URL, ...) - gitignored
    agent.env.sh.example            # Template for agent.env.sh
```

## Updating the flatland patches

Upstream `avidbots/flatland` is tracked as a git submodule at
`third_party/flatland`, pinned to a specific commit. Our edits
for ROS2 Jazzy / Ubuntu 24.04 compatibility (plus battery plugin
registration) live as unified-diff files in `patches/flatland/`,
applied at image build time. `git apply` fails loudly if a patch
no longer applies, so upstream drift cannot silently regress the
build.

The first ten patches are upstream-compat fixes (candidates for a
PR to avidbots); `0011-*` and `0012-*` are project-specific
registration of the battery plugin.

### Editing a patch / adding a new one

```bash
cd third_party/flatland
BASE=$(git rev-parse HEAD)
git checkout -b work "$BASE"
git am --whitespace=nowarn ../../patches/flatland/*.patch
# ...edit code, git add, git commit (with a clear message -- it
# becomes the patch filename and the top of the unified diff)...

# Regenerate the patch stack
rm ../../patches/flatland/*.patch
git format-patch "$BASE"..HEAD -o ../../patches/flatland/
cd ../..
git add patches/flatland
```

### Bumping the upstream pin

```bash
cd third_party/flatland
git fetch origin
git checkout <new-sha>       # or origin/ros2 for the current tip
cd ../..
# Re-apply the stack against the new base and regenerate as above.
# If git am rejects a hunk, resolve manually, commit, re-format-patch.
git add third_party/flatland patches/flatland
```

When upstream merges one of the compat patches, just delete the
corresponding file from `patches/flatland/`.

## ROS2 Topics

Key topics published by the simulation:

| Topic | Type | Source |
|-------|------|--------|
| `/scan` | `sensor_msgs/LaserScan` | Flatland Laser plugin |
| `/odom` | `nav_msgs/Odometry` | Flatland DiffDrive plugin |
| `/map` | `nav_msgs/OccupancyGrid` | Nav2 map_server |
| `/tf` | `tf2_msgs/TFMessage` | DiffDrive (odom->base_link), Laser (base_link->laser_link), AMCL (map->odom) |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 collision_monitor output |
| `/clock` | `rosgraph_msgs/Clock` | Flatland server (sim time) |
| `/plan` | `nav_msgs/Path` | Nav2 planner |
| `/local_plan` | `nav_msgs/Path` | Nav2 controller |
| `/battery_state` | `sensor_msgs/BatteryState` | Battery plugin (charge, voltage, current, percentage) |
| `/battery_marker` | `visualization_msgs/Marker` | Battery plugin (floating text for rviz) |
| `/charging_zones` | `visualization_msgs/MarkerArray` | Battery plugin (zone circles and labels for rviz) |
| `/local_costmap/published_footprint` | `geometry_msgs/PolygonStamped` | Nav2 costmap (robot footprint) |
| `/inorbit/custom_data` | `std_msgs/String` | Republisher node (`battery_percentage`, `battery_voltage`, `estimated_time_remaining` as `key=value`) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `diagnostics_watcher` node |
| `/diagnostics_agg` | `diagnostic_msgs/DiagnosticArray` | `diagnostic_aggregator` (grouped tree) |
| `/diagnostics_toplevel_state` | `diagnostic_msgs/DiagnosticStatus` | `diagnostic_aggregator` (overall status) |

### Sending a navigation goal

Send a single-shot goal via topic — same topic RViz's **2D Goal Pose** tool publishes to:

| Topic | Type | Subscriber |
|-------|------|------------|
| `/goal_pose` | `geometry_msgs/PoseStamped` | Nav2 `bt_navigator` (wraps into a `NavigateToPose` action internally) |

```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 7.0, y: 15.0, z: 0.0}, orientation: {w: 1.0}}}'
```

For full goal lifecycle (feedback, cancellation, result) use the action — what the RViz **Nav2 Goal** button and `ros2 action send_goal` use:

| Action | Type | Server |
|--------|------|--------|
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Nav2 `bt_navigator` |

## Troubleshooting

### Rviz doesn't open

1. Allow X11 access (required even on Wayland, for XWayland):
   ```bash
   xhost +local:docker
   ```

2. Verify `DISPLAY` is set on the host:
   ```bash
   echo $DISPLAY   # Should show something like :0 or :1
   ```

3. Check that `/dev/dri` exists (needed for GPU rendering):
   ```bash
   ls /dev/dri/
   ```

4. If you see MESA/DRI errors, your user may need to be in the `video` and `render` groups:
   ```bash
   sudo usermod -aG video,render $USER
   # Log out and back in for group changes to take effect
   ```

### No map->odom transform

AMCL auto-sets the initial pose from `nav2_params.yaml` (`set_initial_pose: true`). If navigation still isn't working, the localization lifecycle manager may have timed out on startup. Restart the container and wait ~30 seconds for all nodes to initialize.

If you change the robot's spawn position in `worlds/sample.world.yaml`, update the matching `initial_pose` in `config/nav2_params.yaml`.

### DDS discovery issues

The container uses `network_mode: host` for DDS multicast discovery. If running multiple containers or ROS2 nodes on the host, set different `ROS_DOMAIN_ID` values:

```bash
ROS_DOMAIN_ID=42 docker compose up
```

Note: This settings breaks InOrbit agent ROS discoverability (only nav2 and rviz can run in this mode).
