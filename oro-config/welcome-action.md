### Action

Currently the robot accepts the following commands:

**Dock**  
[DOCK A](/dashboards/ops), [DOCK D](/dashboards/ops): Navigates to the charging zones in office "A" or "D"  
[DOCK (NEAREST)](/dashboards/ops): Goes to the nearest charging zone

**Battery**  
[RESET](/dashboards/ops): Replaces the battery to 100%  
[CHARGING](/dashboards/ops), [DISCHARGING](/dashboards/ops): toggles the charging state

**Others**  
[RESTART AGENT](/dashboards/ops): Restart agent running on a selected robot  
[CANCEL NAVIGATION](/dashboards/ops): Cancel the current navigation


### Key-value pairs

Additionally, the robot reports key-value pairs as custom data elements, including battery_charging, battery_percentage and battery_voltage for battery state, and estimated_time_remaining from the nav2 stack. These are added to other built-in attributes implemented by the agent such as CPU usage, disk usage, network rate, etc.
