# ROS Diagnostics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a ROS 2 `/diagnostics` watcher node plus `diagnostic_aggregator` to the flatland simulation, producing a grouped `/diagnostics_agg` tree covering sensors, power, and Nav2 health.

**Architecture:** New ament_python package `diagnostics_watcher` containing one rclpy node that monitors `/scan`, `/odom`, `/cmd_vel`, `/battery_state`, the `map→base_link` TF, and Nav2 lifecycle `get_state` services. Pure status-classification logic lives in `status_logic.py` and is unit-tested. The stock `diagnostic_aggregator/aggregator_node` consumes `/diagnostics` and republishes a three-group tree (Sensors / Power / Navigation) on `/diagnostics_agg`.

**Tech Stack:** ROS 2 Jazzy, rclpy, `diagnostic_updater` (Python), `diagnostic_aggregator`, tf2_ros, pytest.

**Spec:** `docs/superpowers/specs/2026-05-12-ros-diagnostics-design.md`

---

## File map

**New:**
- `diagnostics_watcher/package.xml`
- `diagnostics_watcher/setup.py`
- `diagnostics_watcher/setup.cfg`
- `diagnostics_watcher/resource/diagnostics_watcher`
- `diagnostics_watcher/diagnostics_watcher/__init__.py`
- `diagnostics_watcher/diagnostics_watcher/status_logic.py`
- `diagnostics_watcher/diagnostics_watcher/watcher_node.py`
- `diagnostics_watcher/test/test_status_logic.py`
- `config/diagnostics_aggregator.yaml`

**Modify:**
- `launch/flatland_nav2.launch.py` — add watcher + aggregator nodes
- `Dockerfile` — apt-install diagnostic packages, COPY new package, add to colcon build list
- `README.md` — new Diagnostics section + updates to "What's Included", "ROS2 Topics", "File Structure"

---

## Task 1: Scaffold the `diagnostics_watcher` ament_python package

**Files:**
- Create: `diagnostics_watcher/package.xml`
- Create: `diagnostics_watcher/setup.py`
- Create: `diagnostics_watcher/setup.cfg`
- Create: `diagnostics_watcher/resource/diagnostics_watcher`
- Create: `diagnostics_watcher/diagnostics_watcher/__init__.py`

- [ ] **Step 1: Create `package.xml`**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>diagnostics_watcher</name>
  <version>0.1.0</version>
  <description>Publishes ROS 2 diagnostics for sensor freshness, battery health, TF, and Nav2 lifecycle state.</description>
  <maintainer email="noreply@openrobops.local">OpenRobOps</maintainer>
  <license>Apache-2.0</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>diagnostic_updater</exec_depend>
  <exec_depend>diagnostic_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>lifecycle_msgs</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 2: Create `setup.py`**

```python
from setuptools import find_packages, setup

package_name = 'diagnostics_watcher'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenRobOps',
    maintainer_email='noreply@openrobops.local',
    description='Publishes ROS 2 diagnostics for sensor freshness, battery health, TF, and Nav2 lifecycle state.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'watcher = diagnostics_watcher.watcher_node:main',
        ],
    },
)
```

- [ ] **Step 3: Create `setup.cfg`**

```ini
[develop]
script_dir=$base/lib/diagnostics_watcher
[install]
install_scripts=$base/lib/diagnostics_watcher
```

- [ ] **Step 4: Create the empty resource marker**

Write `diagnostics_watcher/resource/diagnostics_watcher` as a zero-byte file. Use:

```bash
mkdir -p diagnostics_watcher/resource && : > diagnostics_watcher/resource/diagnostics_watcher
```

- [ ] **Step 5: Create empty `__init__.py`**

Write `diagnostics_watcher/diagnostics_watcher/__init__.py` as empty.

- [ ] **Step 6: Commit**

```bash
git add diagnostics_watcher/
git commit -m "diagnostics_watcher: scaffold ament_python package"
```

---

## Task 2: TDD pure status-classification logic

**Files:**
- Create: `diagnostics_watcher/diagnostics_watcher/status_logic.py`
- Create: `diagnostics_watcher/test/test_status_logic.py`

The functions below take only primitives so they can be tested without ROS. They return `(level, message)` where `level` is one of the byte constants from `diagnostic_msgs.msg.DiagnosticStatus`: `OK=0`, `WARN=1`, `ERROR=2`, `STALE=3`.

- [ ] **Step 1: Write the failing tests**

Create `diagnostics_watcher/test/test_status_logic.py`:

```python
import pytest
from diagnostic_msgs.msg import DiagnosticStatus

from diagnostics_watcher.status_logic import (
    classify_freshness,
    classify_battery,
)


class TestClassifyFreshness:
    def test_never_received_within_grace_is_stale(self):
        level, _ = classify_freshness(age_sec=None, stale_sec=2.0, grace_active=True)
        assert level == DiagnosticStatus.STALE

    def test_never_received_after_grace_is_error(self):
        level, _ = classify_freshness(age_sec=None, stale_sec=2.0, grace_active=False)
        assert level == DiagnosticStatus.ERROR

    def test_fresh_is_ok(self):
        level, msg = classify_freshness(age_sec=0.5, stale_sec=2.0, grace_active=False)
        assert level == DiagnosticStatus.OK
        assert "0.50" in msg

    def test_stale_within_grace_is_stale(self):
        level, _ = classify_freshness(age_sec=5.0, stale_sec=2.0, grace_active=True)
        assert level == DiagnosticStatus.STALE

    def test_stale_after_grace_is_error(self):
        level, msg = classify_freshness(age_sec=5.0, stale_sec=2.0, grace_active=False)
        assert level == DiagnosticStatus.ERROR
        assert "2.00" in msg


class TestClassifyBattery:
    def test_no_message_within_grace_is_stale(self):
        level, _ = classify_battery(
            percentage=None, age_sec=None,
            stale_sec=5.0, warn_soc=0.2, critical_soc=0.05, grace_active=True,
        )
        assert level == DiagnosticStatus.STALE

    def test_no_message_after_grace_is_error(self):
        level, _ = classify_battery(
            percentage=None, age_sec=None,
            stale_sec=5.0, warn_soc=0.2, critical_soc=0.05, grace_active=False,
        )
        assert level == DiagnosticStatus.ERROR

    def test_stale_message_after_grace_is_error(self):
        level, _ = classify_battery(
            percentage=0.8, age_sec=10.0,
            stale_sec=5.0, warn_soc=0.2, critical_soc=0.05, grace_active=False,
        )
        assert level == DiagnosticStatus.ERROR

    def test_critical_soc_is_error(self):
        level, msg = classify_battery(
            percentage=0.03, age_sec=0.5,
            stale_sec=5.0, warn_soc=0.2, critical_soc=0.05, grace_active=False,
        )
        assert level == DiagnosticStatus.ERROR
        assert "critical" in msg.lower()

    def test_at_critical_threshold_is_error(self):
        level, _ = classify_battery(
            percentage=0.05, age_sec=0.5,
            stale_sec=5.0, warn_soc=0.2, critical_soc=0.05, grace_active=False,
        )
        assert level == DiagnosticStatus.ERROR

    def test_low_soc_is_warn(self):
        level, msg = classify_battery(
            percentage=0.15, age_sec=0.5,
            stale_sec=5.0, warn_soc=0.2, critical_soc=0.05, grace_active=False,
        )
        assert level == DiagnosticStatus.WARN
        assert "low" in msg.lower()

    def test_at_warn_threshold_is_warn(self):
        level, _ = classify_battery(
            percentage=0.20, age_sec=0.5,
            stale_sec=5.0, warn_soc=0.2, critical_soc=0.05, grace_active=False,
        )
        assert level == DiagnosticStatus.WARN

    def test_healthy_is_ok(self):
        level, msg = classify_battery(
            percentage=0.85, age_sec=0.5,
            stale_sec=5.0, warn_soc=0.2, critical_soc=0.05, grace_active=False,
        )
        assert level == DiagnosticStatus.OK
        assert "85" in msg
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd diagnostics_watcher && python -m pytest test/test_status_logic.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'diagnostics_watcher.status_logic'`

- [ ] **Step 3: Implement `status_logic.py`**

Create `diagnostics_watcher/diagnostics_watcher/status_logic.py`:

```python
"""Pure status-classification helpers. No ROS state — easy to unit-test."""

from typing import Optional, Tuple

from diagnostic_msgs.msg import DiagnosticStatus


def classify_freshness(
    age_sec: Optional[float],
    stale_sec: float,
    grace_active: bool,
) -> Tuple[int, str]:
    """Classify how fresh a topic is.

    age_sec: seconds since the last message, or None if never received.
    stale_sec: age threshold beyond which the topic is considered stale.
    grace_active: True during the startup grace window — downgrades
        missing/stale conditions from ERROR to STALE to avoid bringup noise.
    """
    if age_sec is None:
        level = DiagnosticStatus.STALE if grace_active else DiagnosticStatus.ERROR
        return level, "no message received yet"
    if age_sec > stale_sec:
        level = DiagnosticStatus.STALE if grace_active else DiagnosticStatus.ERROR
        return level, f"last message {age_sec:.2f}s ago (threshold {stale_sec:.2f}s)"
    return DiagnosticStatus.OK, f"last message {age_sec:.2f}s ago"


def classify_battery(
    percentage: Optional[float],
    age_sec: Optional[float],
    stale_sec: float,
    warn_soc: float,
    critical_soc: float,
    grace_active: bool,
) -> Tuple[int, str]:
    """Classify battery health from latest BatteryState.percentage (0.0..1.0)."""
    if percentage is None or age_sec is None:
        level = DiagnosticStatus.STALE if grace_active else DiagnosticStatus.ERROR
        return level, "no battery message received"
    if age_sec > stale_sec:
        level = DiagnosticStatus.STALE if grace_active else DiagnosticStatus.ERROR
        return level, f"battery message stale ({age_sec:.2f}s, threshold {stale_sec:.2f}s)"
    if percentage <= critical_soc:
        return DiagnosticStatus.ERROR, f"battery critical: {percentage * 100:.1f}%"
    if percentage <= warn_soc:
        return DiagnosticStatus.WARN, f"battery low: {percentage * 100:.1f}%"
    return DiagnosticStatus.OK, f"battery {percentage * 100:.1f}%"
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd diagnostics_watcher && python -m pytest test/test_status_logic.py -v`
Expected: 11 passed.

Note: requires `diagnostic_msgs` to be importable. If running outside a sourced ROS environment, source it first: `source /opt/ros/jazzy/setup.bash`. Alternatively, run inside the container via `docker compose run --rm flatland-nav2 bash -c "cd /ros2_ws/src/diagnostics_watcher && python -m pytest test/test_status_logic.py -v"`.

- [ ] **Step 5: Commit**

```bash
git add diagnostics_watcher/diagnostics_watcher/status_logic.py \
        diagnostics_watcher/test/test_status_logic.py
git commit -m "diagnostics_watcher: status classification logic with tests"
```

---

## Task 3: Watcher node skeleton with parameters and Updater

**Files:**
- Create: `diagnostics_watcher/diagnostics_watcher/watcher_node.py`

This task lays down the node class with all parameters declared and the `diagnostic_updater.Updater` wired up. No checks are registered yet — those come in Tasks 4-7.

- [ ] **Step 1: Write the skeleton**

Create `diagnostics_watcher/diagnostics_watcher/watcher_node.py`:

```python
"""ROS 2 diagnostics watcher: publishes /diagnostics for sim health."""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from diagnostic_updater import Updater


class DiagnosticsWatcher(Node):
    def __init__(self):
        super().__init__('diagnostics_watcher')

        # --- Parameters -----------------------------------------------------
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('battery_topic', '/battery_state')

        self.declare_parameter('scan_stale_sec', 2.0)
        self.declare_parameter('odom_stale_sec', 1.0)
        self.declare_parameter('cmd_vel_stale_sec', 5.0)
        self.declare_parameter('battery_stale_sec', 5.0)
        self.declare_parameter('tf_stale_sec', 2.0)

        self.declare_parameter('battery_warn_soc', 0.20)
        self.declare_parameter('battery_critical_soc', 0.05)

        self.declare_parameter('nav2_nodes', [
            'map_server', 'amcl',
            'controller_server', 'planner_server', 'bt_navigator',
        ])

        self.declare_parameter('update_rate_hz', 1.0)
        self.declare_parameter('startup_grace_sec', 5.0)

        self._startup_grace = float(self.get_parameter('startup_grace_sec').value)
        self._started_at = self.get_clock().now()

        # --- Diagnostic updater --------------------------------------------
        self._updater = Updater(self)
        self._updater.setHardwareID('flatland_sim')
        period = 1.0 / float(self.get_parameter('update_rate_hz').value)
        self._updater.add('placeholder', self._noop_diag)
        # `period` is enforced via the timer below; Updater's setPeriod is
        # available but we prefer an explicit ROS timer so use_sim_time works.
        self.create_timer(period, self._tick)

    def _tick(self):
        self._updater.force_update()

    def _grace_active(self) -> bool:
        age = self.get_clock().now() - self._started_at
        return age < Duration(seconds=self._startup_grace)

    def _noop_diag(self, stat):
        # Replaced in Task 4 — keeps Updater happy until then.
        stat.summary(0, 'watcher up')
        return stat


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsWatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Verify the file is importable**

The node won't be run in isolation — verification comes via the integration step in Task 12. For now confirm syntax with:

```bash
python -m py_compile diagnostics_watcher/diagnostics_watcher/watcher_node.py
```
Expected: no output.

- [ ] **Step 3: Commit**

```bash
git add diagnostics_watcher/diagnostics_watcher/watcher_node.py
git commit -m "diagnostics_watcher: node skeleton with parameters and Updater"
```

---

## Task 4: Topic freshness diagnostics (scan, odom, cmd_vel)

**Files:**
- Modify: `diagnostics_watcher/diagnostics_watcher/watcher_node.py`

Three near-identical checks share a small helper. `/scan` uses BestEffort QoS to match the typical publisher; the others use default Reliable.

- [ ] **Step 1: Add the topic subscriptions and freshness checks**

Replace the contents of `watcher_node.py` with:

```python
"""ROS 2 diagnostics watcher: publishes /diagnostics for sim health."""

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from diagnostic_updater import Updater

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from diagnostic_msgs.msg import DiagnosticStatus

from .status_logic import classify_freshness


def _best_effort_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
    )


class DiagnosticsWatcher(Node):
    def __init__(self):
        super().__init__('diagnostics_watcher')

        # --- Parameters -----------------------------------------------------
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('battery_topic', '/battery_state')

        self.declare_parameter('scan_stale_sec', 2.0)
        self.declare_parameter('odom_stale_sec', 1.0)
        self.declare_parameter('cmd_vel_stale_sec', 5.0)
        self.declare_parameter('battery_stale_sec', 5.0)
        self.declare_parameter('tf_stale_sec', 2.0)

        self.declare_parameter('battery_warn_soc', 0.20)
        self.declare_parameter('battery_critical_soc', 0.05)

        self.declare_parameter('nav2_nodes', [
            'map_server', 'amcl',
            'controller_server', 'planner_server', 'bt_navigator',
        ])

        self.declare_parameter('update_rate_hz', 1.0)
        self.declare_parameter('startup_grace_sec', 5.0)

        self._startup_grace = float(self.get_parameter('startup_grace_sec').value)
        self._started_at = self.get_clock().now()

        # --- Last-seen timestamps ------------------------------------------
        self._last_scan: Optional = None
        self._last_odom: Optional = None
        self._last_cmd_vel: Optional = None

        # --- Subscriptions --------------------------------------------------
        self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self._on_scan,
            _best_effort_qos(),
        )
        self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self._on_odom,
            10,
        )
        self.create_subscription(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            self._on_cmd_vel,
            10,
        )

        # --- Diagnostic updater --------------------------------------------
        self._updater = Updater(self)
        self._updater.setHardwareID('flatland_sim')
        self._updater.add('scan_freshness', self._diag_scan)
        self._updater.add('odom_freshness', self._diag_odom)
        self._updater.add('cmd_vel_freshness', self._diag_cmd_vel)

        period = 1.0 / float(self.get_parameter('update_rate_hz').value)
        self.create_timer(period, self._tick)

    # --- Subscription callbacks ---------------------------------------------
    def _on_scan(self, _msg):
        self._last_scan = self.get_clock().now()

    def _on_odom(self, _msg):
        self._last_odom = self.get_clock().now()

    def _on_cmd_vel(self, _msg):
        self._last_cmd_vel = self.get_clock().now()

    # --- Helpers ------------------------------------------------------------
    def _tick(self):
        self._updater.force_update()

    def _grace_active(self) -> bool:
        age = self.get_clock().now() - self._started_at
        return age < Duration(seconds=self._startup_grace)

    def _age_sec(self, ts) -> Optional[float]:
        if ts is None:
            return None
        return (self.get_clock().now() - ts).nanoseconds / 1e9

    # --- Diagnostic tasks ---------------------------------------------------
    def _diag_scan(self, stat):
        level, msg = classify_freshness(
            self._age_sec(self._last_scan),
            float(self.get_parameter('scan_stale_sec').value),
            self._grace_active() and self._last_scan is None,
        )
        stat.summary(level, msg)
        return stat

    def _diag_odom(self, stat):
        level, msg = classify_freshness(
            self._age_sec(self._last_odom),
            float(self.get_parameter('odom_stale_sec').value),
            self._grace_active() and self._last_odom is None,
        )
        stat.summary(level, msg)
        return stat

    def _diag_cmd_vel(self, stat):
        level, msg = classify_freshness(
            self._age_sec(self._last_cmd_vel),
            float(self.get_parameter('cmd_vel_stale_sec').value),
            self._grace_active() and self._last_cmd_vel is None,
        )
        # cmd_vel never escalates beyond WARN — the robot is legitimately
        # idle when no goal is active.
        if level == DiagnosticStatus.ERROR:
            level = DiagnosticStatus.WARN
        stat.summary(level, msg)
        return stat


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsWatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Syntax check**

```bash
python -m py_compile diagnostics_watcher/diagnostics_watcher/watcher_node.py
```
Expected: no output.

- [ ] **Step 3: Commit**

```bash
git add diagnostics_watcher/diagnostics_watcher/watcher_node.py
git commit -m "diagnostics_watcher: scan/odom/cmd_vel freshness checks"
```

---

## Task 5: Battery diagnostic

**Files:**
- Modify: `diagnostics_watcher/diagnostics_watcher/watcher_node.py`

- [ ] **Step 1: Add battery subscription and check**

In `watcher_node.py`:

1. Add the import near the other message imports:

```python
from sensor_msgs.msg import BatteryState
```

2. Add the import for the pure helper next to `classify_freshness`:

```python
from .status_logic import classify_freshness, classify_battery
```

3. In `__init__`, after `self._last_cmd_vel = None`, add:

```python
        self._last_battery_ts = None
        self._last_battery_percentage: Optional[float] = None
```

4. After the `cmd_vel` subscription, add:

```python
        self.create_subscription(
            BatteryState,
            self.get_parameter('battery_topic').value,
            self._on_battery,
            10,
        )
```

5. After `self._updater.add('cmd_vel_freshness', self._diag_cmd_vel)`, add:

```python
        self._updater.add('battery', self._diag_battery)
```

6. Add the callback next to `_on_cmd_vel`:

```python
    def _on_battery(self, msg: BatteryState):
        self._last_battery_ts = self.get_clock().now()
        self._last_battery_percentage = float(msg.percentage)
```

7. Add the diagnostic task next to `_diag_cmd_vel`:

```python
    def _diag_battery(self, stat):
        level, msg = classify_battery(
            self._last_battery_percentage,
            self._age_sec(self._last_battery_ts),
            float(self.get_parameter('battery_stale_sec').value),
            float(self.get_parameter('battery_warn_soc').value),
            float(self.get_parameter('battery_critical_soc').value),
            self._grace_active() and self._last_battery_ts is None,
        )
        stat.summary(level, msg)
        return stat
```

- [ ] **Step 2: Syntax check**

```bash
python -m py_compile diagnostics_watcher/diagnostics_watcher/watcher_node.py
```
Expected: no output.

- [ ] **Step 3: Commit**

```bash
git add diagnostics_watcher/diagnostics_watcher/watcher_node.py
git commit -m "diagnostics_watcher: battery health check"
```

---

## Task 6: TF `map → base_link` diagnostic

**Files:**
- Modify: `diagnostics_watcher/diagnostics_watcher/watcher_node.py`

- [ ] **Step 1: Add TF buffer/listener and the check**

In `watcher_node.py`:

1. Add the TF import next to the other top-level imports:

```python
from tf2_ros import Buffer, TransformListener, TransformException
```

(`DiagnosticStatus` is already imported at the top of the file from Task 4.)

2. In `__init__`, after the battery subscription, add:

```python
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
```

3. After `self._updater.add('battery', self._diag_battery)`, add:

```python
        self._updater.add('tf_map_to_base_link', self._diag_tf)
```

4. Add the diagnostic task:

```python
    def _diag_tf(self, stat):
        stale_sec = float(self.get_parameter('tf_stale_sec').value)
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
        except TransformException as ex:
            if self._grace_active():
                stat.summary(DiagnosticStatus.STALE, f'no map->base_link yet: {ex}')
            else:
                stat.summary(DiagnosticStatus.ERROR, f'map->base_link lookup failed: {ex}')
            return stat

        stamp = rclpy.time.Time.from_msg(tf.header.stamp)
        age_sec = (self.get_clock().now() - stamp).nanoseconds / 1e9
        if age_sec > stale_sec:
            stat.summary(
                DiagnosticStatus.ERROR,
                f'map->base_link stale ({age_sec:.2f}s, threshold {stale_sec:.2f}s)',
            )
        else:
            stat.summary(DiagnosticStatus.OK, f'map->base_link fresh ({age_sec:.2f}s)')
        return stat
```

- [ ] **Step 2: Syntax check**

```bash
python -m py_compile diagnostics_watcher/diagnostics_watcher/watcher_node.py
```
Expected: no output.

- [ ] **Step 3: Commit**

```bash
git add diagnostics_watcher/diagnostics_watcher/watcher_node.py
git commit -m "diagnostics_watcher: map->base_link TF freshness check"
```

---

## Task 7: Nav2 lifecycle diagnostic

**Files:**
- Modify: `diagnostics_watcher/diagnostics_watcher/watcher_node.py`

The check polls `<node>/get_state` services on a 2 s timer and caches the most recent state per node. The diagnostic task summarizes the cache. Service calls use a 0.5 s future timeout so a single slow node doesn't block the executor.

- [ ] **Step 1: Add lifecycle polling and the check**

In `watcher_node.py`:

1. Add the import:

```python
from lifecycle_msgs.srv import GetState
```

2. In `__init__`, after the TF buffer setup, add:

```python
        self._nav2_nodes = list(self.get_parameter('nav2_nodes').value)
        self._lifecycle_clients = {
            name: self.create_client(GetState, f'/{name}/get_state')
            for name in self._nav2_nodes
        }
        # name -> (level, label) cached from the most recent poll
        self._lifecycle_state = {
            name: (DiagnosticStatus.STALE, 'not yet polled') for name in self._nav2_nodes
        }
        self.create_timer(2.0, self._poll_lifecycle)
```

3. After `self._updater.add('tf_map_to_base_link', self._diag_tf)`, add:

```python
        self._updater.add('nav2_lifecycle', self._diag_nav2_lifecycle)
```

4. Add the polling method:

```python
    def _poll_lifecycle(self):
        for name, client in self._lifecycle_clients.items():
            if not client.service_is_ready():
                if self._grace_active():
                    self._lifecycle_state[name] = (DiagnosticStatus.WARN, 'service unavailable (grace)')
                else:
                    self._lifecycle_state[name] = (DiagnosticStatus.WARN, 'service unavailable')
                continue
            future = client.call_async(GetState.Request())
            # Don't block the executor; check completion on the next poll tick.
            future.add_done_callback(
                lambda fut, n=name: self._on_lifecycle_response(n, fut))

    def _on_lifecycle_response(self, name, future):
        try:
            result = future.result()
        except Exception as ex:  # noqa: BLE001 - record and continue
            self._lifecycle_state[name] = (DiagnosticStatus.WARN, f'call failed: {ex}')
            return
        if result is None:
            self._lifecycle_state[name] = (DiagnosticStatus.WARN, 'no response')
            return
        label = result.current_state.label  # e.g. "active", "inactive"
        if label == 'active':
            self._lifecycle_state[name] = (DiagnosticStatus.OK, label)
        else:
            self._lifecycle_state[name] = (DiagnosticStatus.ERROR, label)
```

5. Add the diagnostic task:

```python
    def _diag_nav2_lifecycle(self, stat):
        worst = DiagnosticStatus.OK
        details = []
        all_unreachable = bool(self._lifecycle_state) and all(
            l == DiagnosticStatus.WARN and 'service unavailable' in m
            for l, m in self._lifecycle_state.values()
        )
        for name, (level, label) in self._lifecycle_state.items():
            stat.add(name, label)
            details.append(f'{name}={label}')
            if level > worst:
                worst = level

        # Escalate "all services unreachable" after grace to ERROR.
        if all_unreachable and not self._grace_active():
            worst = DiagnosticStatus.ERROR

        if worst == DiagnosticStatus.OK:
            stat.summary(DiagnosticStatus.OK, 'all Nav2 nodes active')
        else:
            stat.summary(worst, '; '.join(details))
        return stat
```

- [ ] **Step 2: Syntax check**

```bash
python -m py_compile diagnostics_watcher/diagnostics_watcher/watcher_node.py
```
Expected: no output.

- [ ] **Step 3: Commit**

```bash
git add diagnostics_watcher/diagnostics_watcher/watcher_node.py
git commit -m "diagnostics_watcher: Nav2 lifecycle state check"
```

---

## Task 8: Aggregator YAML config

**Files:**
- Create: `config/diagnostics_aggregator.yaml`

- [ ] **Step 1: Write the YAML**

Create `config/diagnostics_aggregator.yaml`:

```yaml
# Groups DiagnosticStatus entries from /diagnostics into a tree on
# /diagnostics_agg. The top-level node name passed to the aggregator
# node ('analyzers') must match the root key here.
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

- [ ] **Step 2: Commit**

```bash
git add config/diagnostics_aggregator.yaml
git commit -m "config: diagnostic_aggregator YAML grouping sensors/power/nav"
```

---

## Task 9: Launch wiring

**Files:**
- Modify: `launch/flatland_nav2.launch.py`

- [ ] **Step 1: Add the watcher and aggregator Node entries**

Open `launch/flatland_nav2.launch.py`.

After the existing `republisher = Node(...)` block (around line 158), add:

```python
    # Diagnostics watcher node — publishes /diagnostics for sim health
    diagnostics_watcher = Node(
        package='diagnostics_watcher',
        executable='watcher',
        name='diagnostics_watcher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Aggregator — republishes /diagnostics_agg grouped by Sensors/Power/Navigation
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

In the returned `LaunchDescription(...)` list at the bottom of the file, add the two new entries between `republisher,` and `rviz_node,`:

```python
        republisher,
        diagnostics_watcher,
        diagnostics_aggregator,
        rviz_node,
```

- [ ] **Step 2: Syntax check**

```bash
python -m py_compile launch/flatland_nav2.launch.py
```
Expected: no output.

- [ ] **Step 3: Commit**

```bash
git add launch/flatland_nav2.launch.py
git commit -m "launch: add diagnostics_watcher and diagnostic_aggregator nodes"
```

---

## Task 10: Dockerfile updates

**Files:**
- Modify: `Dockerfile`

Three edits: (a) apt-install the diagnostic packages, (b) COPY the new package source, (c) add it to the `colcon build --packages-select` list.

- [ ] **Step 1: Add apt packages**

Find the line `ros-jazzy-slam-toolbox \` in the apt install block and insert two lines after it:

```dockerfile
    ros-jazzy-slam-toolbox \
    ros-jazzy-diagnostic-updater \
    ros-jazzy-diagnostic-aggregator \
    cmake \
```

- [ ] **Step 2: Copy the new package source**

Find the line `COPY republisher/ /ros2_ws/src/republisher/` and add this line directly below it:

```dockerfile
# Copy project-owned diagnostics_watcher package
COPY diagnostics_watcher/ /ros2_ws/src/diagnostics_watcher/
```

- [ ] **Step 3: Add to the colcon build list**

Find the line:

```dockerfile
      --packages-select flatland_msgs flatland_server flatland_plugins republisher
```

and replace it with:

```dockerfile
      --packages-select flatland_msgs flatland_server flatland_plugins republisher diagnostics_watcher
```

- [ ] **Step 4: Verify the image builds**

```bash
docker compose build
```
Expected: succeeds. Look near the end for `Finished <<< diagnostics_watcher` in the colcon output.

- [ ] **Step 5: Commit**

```bash
git add Dockerfile
git commit -m "docker: install diagnostic_updater/aggregator and build diagnostics_watcher"
```

---

## Task 11: README documentation

**Files:**
- Modify: `README.md`

Four edits. Apply them in this order.

- [ ] **Step 1: Add bullet to "What's Included"**

Find the bullet `- **ROS2 Agent** - Connects the simulated robot to OpenRobOps (runs in a sidecar container, optional)` and insert this new bullet directly below it:

```markdown
- **ROS Diagnostics** - Health monitoring of sensors, battery, TF, and Nav2 lifecycle published on `/diagnostics` and grouped on `/diagnostics_agg`
```

- [ ] **Step 2: Add a new "## Diagnostics" section between "## InOrbit Agent" and "## File Structure"**

Find `## File Structure` (around line 231) and insert this block directly above it:

````markdown
## Diagnostics

A `diagnostics_watcher` node publishes ROS 2 diagnostics on `/diagnostics`, and a `diagnostic_aggregator` groups them into a tree on `/diagnostics_agg`. Both are launched by default with the rest of the simulation.

### What is monitored

| Check | Trigger |
|-------|---------|
| `scan_freshness` | `/scan` not received for > 2 s (ERROR), or rate < 5 Hz (WARN) |
| `odom_freshness` | `/odom` not received for > 1 s (ERROR) |
| `tf_map_to_base_link` | `map → base_link` lookup fails or stamp > 2 s old (ERROR) |
| `battery` | SOC ≤ 5% or message stale > 5 s (ERROR); SOC ≤ 20% (WARN) |
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

The full list (`scan_stale_sec`, `odom_stale_sec`, `cmd_vel_stale_sec`, `battery_stale_sec`, `tf_stale_sec`, `battery_warn_soc`, `battery_critical_soc`, `nav2_nodes`, `update_rate_hz`, `startup_grace_sec`) lives in `diagnostics_watcher/diagnostics_watcher/watcher_node.py`.

### Disabling

Comment out the `diagnostics_watcher` and `diagnostics_aggregator` `Node` entries (plus their references in the returned `LaunchDescription`) in `launch/flatland_nav2.launch.py`.

````

- [ ] **Step 3: Add the diagnostics topics to the ROS2 Topics table**

In the `## ROS2 Topics` table, just before the closing line about `/inorbit/custom_data`, add three rows:

```markdown
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `diagnostics_watcher` node |
| `/diagnostics_agg` | `diagnostic_msgs/DiagnosticArray` | `diagnostic_aggregator` (grouped tree) |
| `/diagnostics_toplevel_state` | `diagnostic_msgs/DiagnosticStatus` | `diagnostic_aggregator` (overall status) |
```

- [ ] **Step 4: Update the File Structure listing**

In the `## File Structure` code block:

(a) Add a line inside the `config/` group, right under `flatland_rviz.rviz`:

```
    diagnostics_aggregator.yaml     # Analyzer groups for diagnostic_aggregator
```

(b) Add a new package block between `plugins/` and `maps/`:

```
  republisher/                      # Project ROS2 package - InOrbit custom_data bridge
  diagnostics_watcher/              # Project ROS2 package - /diagnostics publisher
```

(`republisher/` is currently missing from the File Structure block — adding it here completes the picture.)

- [ ] **Step 5: Commit**

```bash
git add README.md
git commit -m "docs: README section for ROS diagnostics"
```

---

## Task 12: Integration verification

This task does not produce a commit. It runs the whole thing end-to-end.

- [ ] **Step 1: Rebuild and start the stack**

```bash
docker compose build
docker compose up -d
```

Wait ~15 s for bringup to settle.

- [ ] **Step 2: Confirm `/diagnostics_agg` is healthy**

```bash
docker compose exec flatland-nav2 bash -c \
  'source /ros2_ws/install/setup.bash && ros2 topic echo /diagnostics_agg --once'
```

Expected output contains three groups whose `level` field is `0` (OK):

- `name: "/Sensors"`
- `name: "/Power"`
- `name: "/Navigation"`

If any group reports `level: 1` or `level: 2`, expand its child statuses to identify which check is failing.

- [ ] **Step 3: Confirm the watcher reacts to a failure**

```bash
# Stop the flatland server inside the container; this should kill /scan and /odom.
docker compose exec flatland-nav2 bash -c \
  'source /ros2_ws/install/setup.bash && pkill -f flatland_server'

# Within ~3 seconds:
docker compose exec flatland-nav2 bash -c \
  'source /ros2_ws/install/setup.bash && ros2 topic echo /diagnostics_agg --once'
```

Expected: `/Sensors` reports `level: 2` (ERROR), with child statuses for `scan_freshness` and `odom_freshness` indicating staleness.

- [ ] **Step 4: Confirm `rqt_robot_monitor` renders**

```bash
docker compose exec flatland-nav2 bash -c \
  'source /ros2_ws/install/setup.bash && ros2 run rqt_robot_monitor rqt_robot_monitor &'
```

Expected: a GUI window with three top-level rows (Sensors, Power, Navigation), each expandable into the individual checks. Close the window when done.

- [ ] **Step 5: Tear down**

```bash
docker compose down
```

---

## Acceptance check

After Task 12, all of these should be true:

- `colcon build` (inside the image) succeeds with `diagnostics_watcher` in the list.
- `colcon test --packages-select diagnostics_watcher` is green (run inside the image; the pure-logic tests do not need a running ROS graph).
- `/diagnostics_agg` shows three top-level groups, all OK once warmed up.
- Killing flatland flips `Sensors` to ERROR within `scan_stale_sec + 1 s`.
- README has the four documentation updates from Task 11.
