# ROS Diagnostics for the Flatland Simulation — Design

**Date:** 2026-05-12
**Status:** Approved (pre-implementation)

## Goal

Publish a useful ROS 2 `diagnostic_msgs/DiagnosticArray` tree from the simulation so that:

- `rqt_robot_monitor` shows live OK/WARN/ERROR per subsystem.
- Downstream consumers (InOrbit agent, custom dashboards, alerting) can subscribe to a grouped, aggregated view on `/diagnostics_agg`.
- Operators get a single signal that tells them "is the sim healthy and is Nav2 ready to drive?"

## Non-goals

- Instrumenting the upstream Flatland C++ code with `diagnostic_updater` (kept for a follow-up if/when needed).
- Reporting host/container resource diagnostics (CPU, RAM) — out of scope for now.
- Diagnostics-driven recovery behaviors. This spec is observability-only.

## Approach

External watcher node ("approach 2") + `diagnostic_aggregator` ("approach 3"), as chosen during brainstorming. No changes to flatland or the battery plugin in this iteration.

## Architecture

```
flatland (publishers) ──► /scan, /odom, /tf, /battery_state, /cmd_vel
                              │
                              ▼
        diagnostics_watcher_node  ── publishes ──►  /diagnostics
                                                       │
                                              diagnostic_aggregator
                                                       │
                                                       ▼
                                                /diagnostics_agg
                                          (Sensors / Power / Navigation)
```

A new ament_python package `diagnostics_watcher` sits next to `republisher`. It contains one rclpy node that subscribes to the relevant topics, holds a TF buffer, calls Nav2 lifecycle `get_state` services on a slow timer, and uses `diagnostic_updater.Updater` to publish `/diagnostics` at 1 Hz. The stock `diagnostic_aggregator/aggregator_node` consumes `/diagnostics` and republishes a grouped tree on `/diagnostics_agg`, configured by a YAML.

## Files

| Path | Purpose |
|---|---|
| `diagnostics_watcher/package.xml` | ament_python package metadata |
| `diagnostics_watcher/setup.py` | Entry point: `watcher = diagnostics_watcher.watcher_node:main` |
| `diagnostics_watcher/setup.cfg` | Standard ament_python scaffolding |
| `diagnostics_watcher/resource/diagnostics_watcher` | Empty resource marker file |
| `diagnostics_watcher/diagnostics_watcher/__init__.py` | Empty |
| `diagnostics_watcher/diagnostics_watcher/watcher_node.py` | Main rclpy node (see "Watcher node" below) |
| `diagnostics_watcher/diagnostics_watcher/status_logic.py` | Pure helper functions for OK/WARN/ERROR classification (unit-tested) |
| `diagnostics_watcher/test/test_status_logic.py` | pytest cases covering all transitions |
| `config/diagnostics_aggregator.yaml` | Three analyzer groups (see "Aggregator config") |
| `launch/flatland_nav2.launch.py` | Two new `Node` entries (watcher + aggregator) |
| `Dockerfile` | Install `ros-jazzy-diagnostic-updater` and `ros-jazzy-diagnostic-aggregator` apt packages |
| `README.md` | New top-level `## Diagnostics` section + updates to `## What's Included`, `## ROS2 Topics`, and `## File Structure` |

## Watcher node

Six diagnostic tasks registered on a single `diagnostic_updater.Updater`:

| Status name | Inputs | Rules |
|---|---|---|
| `scan_freshness` | `/scan` (BestEffort) | ERROR if no msg in `scan_stale_sec` (default 2.0); WARN if observed rate < 5 Hz over the last 1 s window |
| `odom_freshness` | `/odom` (Reliable) | ERROR if no msg in `odom_stale_sec` (default 1.0) |
| `tf_map_to_base_link` | TF buffer | ERROR if lookup fails or stamp older than `tf_stale_sec` (default 2.0) |
| `battery` | `/battery_state` (Reliable) | ERROR if stale (> `battery_stale_sec`, default 5.0) or `percentage ≤ battery_critical_soc` (default 0.05); WARN if `percentage ≤ battery_warn_soc` (default 0.20); OK otherwise |
| `cmd_vel_freshness` | `/cmd_vel` (Reliable) | WARN if no msg in `cmd_vel_stale_sec` (default 5.0). Never escalates to ERROR — the robot is legitimately idle when not navigating |
| `nav2_lifecycle` | Lifecycle `get_state` services for `map_server`, `amcl`, `controller_server`, `planner_server`, `bt_navigator` (list parameter) | ERROR if any node reports a non-`active` state; WARN if one or more services are unreachable within the grace period or per-call 0.5 s timeout; ERROR if *every* service is unreachable after the grace period; polled every 2 s |

### Startup grace period

For the first `startup_grace_sec` (default 5.0) after the node comes up, missing-message conditions report `STALE` rather than `ERROR`. This avoids a transient red flag while bringup is in flight.

### Parameters

All thresholds and topic names are ROS parameters with defaults equal to the values above. Param names:

- `scan_topic`, `odom_topic`, `cmd_vel_topic`, `battery_topic`
- `scan_stale_sec`, `odom_stale_sec`, `battery_stale_sec`, `cmd_vel_stale_sec`, `tf_stale_sec`
- `battery_warn_soc`, `battery_critical_soc`
- `nav2_nodes` (list of strings)
- `update_rate_hz` (default 1.0)
- `startup_grace_sec` (default 5.0)

### `hardware_id`

All published `DiagnosticStatus` entries use `hardware_id = "flatland_sim"` to make it easy to filter the sim's diagnostics from anything else on the bus.

## Aggregator config — `config/diagnostics_aggregator.yaml`

```yaml
analyzers:
  ros__parameters:
    pub_rate: 1.0
    base_path: ""
    analyzers:
      sensors:
        type: diagnostic_aggregator/GenericAnalyzer
        path: Sensors
        contains: ["scan", "odom", "tf"]
      power:
        type: diagnostic_aggregator/GenericAnalyzer
        path: Power
        contains: ["battery"]
      navigation:
        type: diagnostic_aggregator/GenericAnalyzer
        path: Navigation
        contains: ["cmd_vel", "nav2"]
```

The aggregator node is launched as `name: analyzers` so the `analyzers:` root key in the YAML matches.

## Launch wiring

Two new `Node` entries appended to `launch/flatland_nav2.launch.py` before `rviz_node`:

```python
diagnostics_watcher = Node(
    package='diagnostics_watcher',
    executable='watcher',
    name='diagnostics_watcher',
    output='screen',
    parameters=[{'use_sim_time': use_sim_time}],
)

diagnostics_aggregator = Node(
    package='diagnostic_aggregator',
    executable='aggregator_node',
    name='analyzers',
    output='screen',
    parameters=[
        os.path.join(bringup_dir, 'config', 'diagnostics_aggregator.yaml'),
        {'use_sim_time': use_sim_time},
    ],
)
```

Both are added to the returned `LaunchDescription`.

## Docker

Add `ros-jazzy-diagnostic-updater` and `ros-jazzy-diagnostic-aggregator` to the apt install line in `Dockerfile`. Python bindings for `diagnostic_updater` ship inside `ros-jazzy-diagnostic-updater` on Jazzy.

## Documentation (README)

Add the following changes to `README.md`:

1. **`## What's Included`** — add a bullet:
   > **ROS Diagnostics** — health monitoring of sensors, battery, and Nav2 lifecycle, published as `diagnostic_msgs/DiagnosticArray` on `/diagnostics` and aggregated to `/diagnostics_agg`.

2. **New `## Diagnostics` section** (placed between `## InOrbit Agent` and `## File Structure`), containing:
   - What is monitored (the six checks, in plain prose).
   - Topics: `/diagnostics`, `/diagnostics_agg`, `/diagnostics_toplevel_state`.
   - How to view: `ros2 run rqt_robot_monitor rqt_robot_monitor` and `ros2 topic echo /diagnostics_agg`.
   - How to tune thresholds (pointer to the watcher node's parameters with a one-line example).
   - How to disable (set the lifecycle list empty / comment out the two `Node` entries).

3. **`## ROS2 Topics`** — add `/diagnostics`, `/diagnostics_agg`, `/diagnostics_toplevel_state` to the listing with one-line descriptions.

4. **`## File Structure`** — add the `diagnostics_watcher/` package and `config/diagnostics_aggregator.yaml`.

## Error handling

- Sensor subscriptions use `BestEffort` QoS to match typical publishers; `Reliable` for `cmd_vel`, `odom`, `battery_state`.
- TF lookups wrapped in `try/except tf2_ros.TransformException` and treated as a stale/missing transform.
- Lifecycle `get_state` calls use `wait_for_service(0.0)` (no blocking) plus a 0.5 s future timeout per node; failures contribute `WARN` ("service unreachable") rather than ERROR, except after the grace period when every node fails to respond — that becomes ERROR.
- No `try/except Exception` catch-alls. Anything unexpected propagates so it shows up in logs.

## Testing

### Unit (`pytest`)
Status classification lives in `status_logic.py` as pure functions. Test cases cover every transition:
- `classify_freshness(age, stale_sec, grace, now_since_start)` → OK / WARN / ERROR / STALE
- `classify_battery(percentage, age, stale_sec, warn_soc, critical_soc)` → OK / WARN / ERROR / STALE

`colcon test --packages-select diagnostics_watcher` runs them.

### Manual integration
1. `docker compose up`.
2. `docker compose exec flatland-nav2 ros2 topic echo /diagnostics_agg --once` — should show three top-level groups all OK after the grace period.
3. Kill the flatland process inside the container — `Sensors` flips to ERROR within ~2 s.
4. `ros2 run rqt_robot_monitor rqt_robot_monitor` — verify the tree renders with the three groups.

## Open questions / deferred

- Whether to instrument the C++ battery plugin directly with `diagnostic_updater` (approach 1) — deferred. The watcher's topic-based view is sufficient for now.
- CPU/memory/process diagnostics for the container itself — deferred.

## Acceptance criteria

- `colcon build` succeeds with the new package.
- After `docker compose up`, `/diagnostics_agg` reports OK for `Sensors`, `Power`, `Navigation` once warmed up.
- Killing the flatland process flips `Sensors` to ERROR within `scan_stale_sec` + 1 s.
- README has the four documentation updates listed above.
- `colcon test --packages-select diagnostics_watcher` is green.
