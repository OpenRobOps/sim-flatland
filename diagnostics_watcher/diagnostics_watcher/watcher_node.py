"""ROS 2 diagnostics watcher: publishes /diagnostics for sim health."""

from collections import deque
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from diagnostic_updater import Updater

from sensor_msgs.msg import BatteryState, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from diagnostic_msgs.msg import DiagnosticStatus
from tf2_ros import Buffer, TransformListener, TransformException

from .status_logic import classify_battery, classify_freshness


def _best_effort_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
    )


class DiagnosticsWatcher(Node):
    """Publishes /diagnostics aggregating sim health checks."""

    def __init__(self):
        """Initialize the diagnostics watcher node."""
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
        self._last_scan = None
        self._scan_timestamps: deque = deque()
        self._last_odom = None
        self._last_cmd_vel = None
        self._last_battery_ts = None
        self._last_battery_percentage: Optional[float] = None

        # --- Subscriptions --------------------------------------------------
        # BestEffort QoS tolerates publishers using either reliability setting
        # (a BestEffort subscriber can still receive from a Reliable publisher).
        # Flatland's current /scan publisher is Reliable; this keeps the
        # subscription forward-compatible if that changes.
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
        self.create_subscription(
            BatteryState,
            self.get_parameter('battery_topic').value,
            self._on_battery,
            10,
        )

        # --- TF listener for map -> base_link ------------------------------
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # --- Diagnostic updater --------------------------------------------
        self._updater = Updater(self)
        self._updater.setHardwareID('flatland_sim')
        # Cancel the Updater's built-in 1 Hz timer; we drive it explicitly below.
        self._updater.timer.cancel()
        self._updater.add('scan_freshness', self._diag_scan)
        self._updater.add('odom_freshness', self._diag_odom)
        self._updater.add('cmd_vel_freshness', self._diag_cmd_vel)
        self._updater.add('battery', self._diag_battery)
        self._updater.add('tf_map_to_base_link', self._diag_tf)

        period = 1.0 / float(self.get_parameter('update_rate_hz').value)
        self.create_timer(period, self._tick)

    # --- Subscription callbacks ---------------------------------------------
    def _on_scan(self, _msg):
        now = self.get_clock().now()
        self._last_scan = now
        self._scan_timestamps.append(now)
        window = Duration(seconds=1.0)
        while self._scan_timestamps and (now - self._scan_timestamps[0]) > window:
            self._scan_timestamps.popleft()

    def _on_odom(self, _msg):
        self._last_odom = self.get_clock().now()

    def _on_cmd_vel(self, _msg):
        self._last_cmd_vel = self.get_clock().now()

    def _on_battery(self, msg: BatteryState):
        self._last_battery_ts = self.get_clock().now()
        self._last_battery_percentage = float(msg.percentage)

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
            self._grace_active(),
        )
        # WARN if rate dropped below 5 Hz over the last 1 s window.
        if level == DiagnosticStatus.OK and len(self._scan_timestamps) < 5:
            level = DiagnosticStatus.WARN
            msg = f'low scan rate: {len(self._scan_timestamps)} msg/s (< 5 Hz)'
        stat.summary(level, msg)
        return stat

    def _diag_odom(self, stat):
        level, msg = classify_freshness(
            self._age_sec(self._last_odom),
            float(self.get_parameter('odom_stale_sec').value),
            self._grace_active(),
        )
        stat.summary(level, msg)
        return stat

    def _diag_cmd_vel(self, stat):
        level, msg = classify_freshness(
            self._age_sec(self._last_cmd_vel),
            float(self.get_parameter('cmd_vel_stale_sec').value),
            self._grace_active(),
        )
        # cmd_vel never escalates beyond WARN — the robot is legitimately
        # idle when no goal is active.
        if level == DiagnosticStatus.ERROR:
            level = DiagnosticStatus.WARN
        stat.summary(level, msg)
        return stat

    def _diag_battery(self, stat):
        level, msg = classify_battery(
            self._last_battery_percentage,
            self._age_sec(self._last_battery_ts),
            float(self.get_parameter('battery_stale_sec').value),
            float(self.get_parameter('battery_warn_soc').value),
            float(self.get_parameter('battery_critical_soc').value),
            self._grace_active(),
        )
        stat.summary(level, msg)
        return stat

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


def main(args=None):
    """Run the diagnostics watcher node."""
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
