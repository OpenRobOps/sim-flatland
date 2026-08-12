import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
from nav2_msgs.action import NavigateToPose


class Republisher(Node):
    def __init__(self):
        super().__init__('republisher')

        self.pub = self.create_publisher(String, '/inorbit/custom_data', 10)

        self.create_subscription(
            BatteryState, '/battery_state', self._on_battery, 10)

        # NavigateToPose feedback is an action topic; subscribe to the hidden
        # feedback topic directly so we don't have to act as an action client.
        self.create_subscription(
            NavigateToPose.Impl.FeedbackMessage,
            '/navigate_to_pose/_action/feedback',
            self._on_nav_feedback, 10)

        # ORO "Message" actions publish 'echo=<text>' on the custom command
        # topic; echo it back verbatim as a custom_data key-value (key 'echo').
        self.create_subscription(
            String, '/inorbit/custom_command', self._on_custom_command, 10)

    def _publish_kv(self, key, value):
        msg = String()
        msg.data = f'{key}={value}'
        self.pub.publish(msg)

    def _on_battery(self, msg: BatteryState):
        self._publish_kv('battery_percentage', msg.percentage)
        self._publish_kv('battery_voltage', msg.voltage)
        charging = msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING
        self._publish_kv('battery_charging', 'true' if charging else 'false')

    def _on_custom_command(self, msg: String):
        if msg.data.startswith('echo='):
            self.pub.publish(msg)

    def _on_nav_feedback(self, msg):
        eta = msg.feedback.estimated_time_remaining
        self._publish_kv('estimated_time_remaining',
                         eta.sec + eta.nanosec / 1e9)


def main(args=None):
    rclpy.init(args=args)
    node = Republisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
