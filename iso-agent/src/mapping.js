// Pure ROS 2 message -> ISO 21423 resource mappings. No I/O, unit-tested in test/mapping.test.js.
import { nowTimestamp } from '@openrobops/iso21423';

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

/** Regular polygon approximating the turtlebot body circle (worlds/turtlebot.model.yaml: radius 0.22). */
export function circlePolygon(radius, n = 16) {
  return Array.from({ length: n }, (_, i) => {
    const a = (2 * Math.PI * i) / n;
    return { x: +(radius * Math.cos(a)).toFixed(4), y: +(radius * Math.sin(a)).toFixed(4) };
  });
}


/**
 * Rounded rectangle centred on the robot origin (+x forward): `length` along x, `width` along y,
 * corners of radius `r` drawn with `segs` arc segments each. Counter-clockwise from the front-right
 * corner.
 */
export function roundedRectPolygon(length, width, r, segs = 4) {
  const hx = length / 2 - r; const hy = width / 2 - r;
  const corners = [[hx, -hy, -Math.PI / 2], [hx, hy, 0], [-hx, hy, Math.PI / 2], [-hx, -hy, Math.PI]];
  const pts = [];
  for (const [cx, cy, a0] of corners) {
    for (let i = 0; i <= segs; i++) {
      const a = a0 + (Math.PI / 2) * (i / segs);
      pts.push({ x: +(cx + r * Math.cos(a)).toFixed(4), y: +(cy + r * Math.sin(a)).toFixed(4) });
    }
  }
  return pts;
}

/**
 * The flatland robot's outline as drawn on the map: nav2's 0.44 x 0.28 m costmap footprint
 * (config/nav2_params.yaml) enlarged proportionally to 0.60 x 0.38 m so it reads as a robot at
 * fleet zoom levels; 0.08 m rounded corners, 20 points.
 */
export const FLATLAND_FOOTPRINT = roundedRectPolygon(0.60, 0.38, 0.08);

/** ISO §3.1 ImrDetails for the flatland robot; all six required fields present. */
export function imrDetails({ uuid, version }) {
  const footprint = FLATLAND_FOOTPRINT;
  return {
    imrModel: 'flatland-nav2',
    imrSerialNumber: uuid,
    imrFootprint: footprint,
    imrWorkingArea: footprint,
    imrHeight: 0.4,
    softwareVersions: [{ moduleName: 'iso-agent', moduleVersion: version }],
  };
}

/** nav_msgs/Path → ISO stamped CCS points. nav2 leaves per-pose stamps at 0, so all points share `now`. */
export function toStampedPoints(ccsId, pathMsg) {
  const timestamp = nowTimestamp();
  return ((pathMsg && pathMsg.poses) || []).map((p) => ({
    timestamp, locationPoint: { ccsId, x: p.pose.position.x, y: p.pose.position.y, z: 0 },
  }));
}

const round3 = (v) => Math.round(v * 1000) / 1000;

/**
 * Dedupe key for a nav_msgs/Path plan: the rounded (x, y) positions only, ignoring per-pose
 * timestamps and the header. nav2 restamps every point with `nowTimestamp()` on every publish,
 * even when the plan itself hasn't changed, so a key derived from `toStampedPoints` output would
 * never repeat. Rounded to 3 decimals to ignore floating-point jitter between otherwise-identical
 * plans.
 */
export function planKey(pathMsg) {
  return JSON.stringify(
    ((pathMsg && pathMsg.poses) || []).map((p) => [round3(p.pose.position.x), round3(p.pose.position.y)]));
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
