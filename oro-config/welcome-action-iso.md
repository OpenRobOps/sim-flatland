### Action

The ISO 21423 agent accepts ISO `move` and `cancelRequest` requests, which OpenRobOps sends from the built-in navigation actions:

[NAVIGATE TO](/dashboards/navigation): Sends the robot to a point picked on the map (ISO `move`)  
[CANCEL NAVIGATION](/dashboards/robot): Cancels the current navigation (ISO `cancelRequest`)

**Dock**  
[DOCK A](/dashboards/robot), [DOCK D](/dashboards/robot), [DOCK (NEAREST)](/dashboards/robot): sent as the native ISO `dock` action (`dockActions: CHARGE`); the agent navigates to the charging zone and the simulated battery starts charging there

**Battery**  
[RESET](/dashboards/robot), [CHARGING](/dashboards/robot), [DISCHARGING](/dashboards/robot): simulation hacks forwarded as an OpenRobOps `customCommand` request, which the agent republishes on the sim's command topic

The Message actions of the ROS2 configuration are not available yet: their `echo` reply is key-value data, which has no ISO 21423 path.

### Reported data

The robot publishes the ISO 21423 `status`, `odometry` and `batteryStatus` resources. OpenRobOps
maps them onto its built-in attributes: online state, `pose`, `speedLinear`, `speedAngular`,
`batteryPercentage`, `batteryVoltage` and `batteryIsCharging`. There are no key-value pairs,
ROS diagnostics or system vitals (CPU, disk, network) in this mode.
