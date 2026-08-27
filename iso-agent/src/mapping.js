// Pure ROS 2 message -> ISO 21423 resource mappings. No I/O, unit-tested in test/mapping.test.js.

/** geometry_msgs/Quaternion -> yaw (rad). */
export function yawFromQuaternion({ x, y, z, w }) {
  return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

/**
 * `/amcl_pose` (PoseWithCovarianceStamped) + `/odom` twist -> ISO `Odometry` body (no timestamp).
 * Flatland's map origin is [0,0,0], so map coordinates ARE the CCS coordinates; the matching
 * ORO calibration is the identity (see oro-config/iso-robot.yaml).
 */
export function toOdometry(ccsId, amclPose, odom) {
  const p = amclPose.pose.pose;
  const t = odom?.twist?.twist ?? { linear: { x: 0 }, angular: { z: 0 } };
  return {
    pose: {
      locationPoint: { ccsId, x: p.position.x, y: p.position.y, z: 0 },
      orientation: { yaw: yawFromQuaternion(p.orientation), pitch: 0, roll: 0 },
    },
    velocity: { linear: t.linear.x, angular: t.angular.z },
  };
}

// sensor_msgs/BatteryState power_supply_status
const CHARGING_STATE = { 1: 'CHARGING', 2: 'DISCHARGING', 3: 'NOT_CHARGING', 4: 'FULL' };

/** `/battery_state` (sensor_msgs/BatteryState) -> ISO `BatteryStatus` body (no timestamp). */
export function toBatteryStatus(msg) {
  const soc = Number.isFinite(msg.percentage) ? Math.min(1, Math.max(0, msg.percentage)) : 0;
  const out = { batterySoc: soc, batteryChargingState: CHARGING_STATE[msg.power_supply_status] ?? 'UNKNOWN' };
  if (Number.isFinite(msg.voltage)) out.batteryVoltage = msg.voltage;
  if (Number.isFinite(msg.current)) out.batteryCurrent = msg.current;
  return out;
}

/**
 * Operating states from the latest telemetry snapshot.
 * @param {{goalActive:boolean, linear:number, diagLevel:number, charging:boolean, soc:number, docking?:boolean}} s
 *   diagLevel is diagnostic_msgs/DiagnosticStatus.level (0 OK, 1 WARN, 2 ERROR, 3 STALE).
 */
export function deriveStates({ goalActive, linear, diagLevel, charging, soc, docking = false }) {
  const states = ['MODE_AUTO', diagLevel >= 2 ? 'NOT_READY' : 'READY'];
  if (Math.abs(linear) > 0.01) states.push(linear > 0 ? 'FORWARD' : 'REVERSE');
  else states.push(goalActive ? 'STOPPED' : 'IDLE');
  if (docking) states.push('DOCKING');
  if (charging) states.push('CHARGING');
  if (soc < 0.2) states.push('LOW_BATTERY');
  return states;
}

/** True when any goal in an action_msgs/GoalStatusArray is accepted or executing (status 1, 2, 3=canceling). */
export function goalActiveFrom(statusArray) {
  return (statusArray.status_list ?? []).some((g) => g.status >= 1 && g.status <= 3);
}

/**
 * OpenRobOps' `customData` extension resource (see ORO's ingest/src/server/iso21423/customData.js):
 * `/ISO_21423/v1/IMR/<uuid>/customData`, QoS 1, not retained, no schema.
 */
export const CUSTOM_DATA_RESOURCE = 'customData';
export const CUSTOM_DATA_RESOURCE_CONFIG = { qos: 1, retain: false };

/** One `key=value` line from /inorbit/custom_data -> [key, value], or null when there is no '='. */
export function parseKeyValue(line) {
  const i = line.indexOf('=');
  if (i <= 0) return null;
  return [line.slice(0, i), line.slice(i + 1)];
}

/** Accumulated pairs -> customData payload. */
export function toCustomData(values) {
  return { timestamp: new Date().toISOString(), values };
}

/** ISO `move` (location) or `dock` (dockLocation) properties -> nav2 NavigateToPose goal in the `map` frame. */
export function toNavGoal(props) {
  const target = props.location ?? props.dockLocation;
  const yaw = props.orientation?.yaw ?? 0;
  return {
    pose: {
      header: { frame_id: 'map' },
      pose: {
        position: { x: target.x, y: target.y, z: 0 },
        orientation: { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) },
      },
    },
    behavior_tree: '',
  };
}

/**
 * Wraps `fn` so it runs at most once per `ms` milliseconds; calls in between are dropped
 * (latest-value semantics are not needed: the final outcome is reported separately).
 */
export function throttle(fn, ms) {
  let last = -Infinity;
  return (...args) => {
    const now = Date.now();
    if (now - last < ms) return;
    last = now;
    fn(...args);
  };
}
