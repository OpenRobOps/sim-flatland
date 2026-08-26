### Action

The ISO 21423 agent accepts ISO `move` and `cancelRequest` requests, which OpenRobOps sends from the built-in navigation actions:

[NAVIGATE TO](/dashboards/navigation): Sends the robot to a point picked on the map (ISO `move`)  
[CANCEL NAVIGATION](/dashboards/robot): Cancels the current navigation (ISO `cancelRequest`)

The battery, dock and message actions of the ROS2 configuration are not available: they rely on a
custom ROS topic that has no ISO 21423 equivalent.

### Reported data

The robot publishes the ISO 21423 `status`, `odometry` and `batteryStatus` resources. OpenRobOps
maps them onto its built-in attributes: online state, `pose`, `speedLinear`, `speedAngular`,
`batteryPercentage`, `batteryVoltage` and `batteryIsCharging`. There are no key-value pairs,
ROS diagnostics or system vitals (CPU, disk, network) in this mode.
