"""ROS 2 diagnostics watcher: publishes /diagnostics for sim health."""

from diagnostic_updater import Updater

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node


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

        # --- Diagnostic updater --------------------------------------------
        self._updater = Updater(self)
        self._updater.setHardwareID('flatland_sim')
        # Cancel the Updater's built-in 1 Hz timer; we drive it explicitly below.
        self._updater.timer.cancel()
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
