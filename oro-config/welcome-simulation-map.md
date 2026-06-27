### Flatland simulator

flatland-sim is a simple 2D simulation robot running [ROS2](https://www.ros.org/). It implements the [Flatland](https://github.com/avidbots/flatland) simulator and the [Nav2](https://github.com/ros-navigation/navigation2) navigation stack, providing a realistic simulation of the robot's behavior and navigation. It also implements a battery plugin that simulates the robot's battery state, which drains rather quickly for demo purposes, and recharges when the robot enters a charging zone.

### Map

The map consists of 4 offices and a corridor. Office A and D have charging zones. 

```
    ┌─────────────┬─────────────┐
    │      A      │      B      │
    │     dock    │             │
    └─────   ─────┴─────   ─────┘
    │         corridor          │
    ┌─────   ─────┬─────   ─────┐
    │      C      │      D      │
    │             │     dock    │
    └─────────────┴──────    ───┘
```
