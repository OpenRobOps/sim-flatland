// ISO 21423 IMR agent for flatland. Reads ROS 2 topics through rosbridge, publishes ISO
// status/odometry/batteryStatus to the fleet manager's broker, and serves ISO `move` requests
// as nav2 NavigateToPose goals. Configuration is env-only (see ../local/iso-agent.env.sh.example).
import { Ros, Topic, Action } from 'roslib';
import { Iso21423Client } from '@openrobops/iso21423';
import { toOdometry, toBatteryStatus, deriveStates, goalActiveFrom, toNavGoal } from './mapping.js';

const env = (k, d) => {
  const v = process.env[k] ?? d;
  if (v === undefined) throw new Error(`missing env ${k}`);
  return v;
};
const ENTITY_UUID = env('ISO_ENTITY_UUID').toLowerCase();
const CCS_ID = env('ISO_CCS_ID').toLowerCase();
const ORO_URL = env('ORO_URL').replace(/\/$/, '');
const ORO_API_KEY = env('ORO_API_KEY');
const ROSBRIDGE_URL = env('ROSBRIDGE_URL', 'ws://localhost:9090');
const ODOM_HZ = Number(env('ISO_ODOMETRY_HZ', '2'));

const log = (...a) => console.log(new Date().toISOString(), ...a);
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

/** Gate 1 of ORO onboarding: exchange the fleet api key for broker credentials. Retries until admitted. */
async function fetchBrokerConfig() {
  for (;;) {
    const res = await fetch(`${ORO_URL}/iso_mqtt_config`, {
      method: 'POST', headers: { 'content-type': 'application/json' },
      body: JSON.stringify({ apiKey: ORO_API_KEY, entityUuid: ENTITY_UUID }),
    }).catch((e) => ({ ok: false, status: 0, text: async () => e.message }));
    if (res.ok) return res.json();
    log(`iso_mqtt_config -> ${res.status}: ${(await res.text()).slice(0, 200)}; retrying in 10s`
      + (res.status === 403 ? ' (robot not admitted: apply oro-config/iso/iso-robot.yaml)' : ''));
    await sleep(10_000);
  }
}

async function main() {
  const cfg = await fetchBrokerConfig();
  // ORO's broker `protocol` field already carries '://' (e.g. "mqtt://"), same as /mqtt_config.
  const url = `${cfg.protocol}${cfg.hostname}:${cfg.port}`;
  log(`broker ${url} as ${cfg.username}`);

  const client = await Iso21423Client.connect({ url, security: { username: cfg.username, password: cfg.password } });
  client.on('connection', (s) => log('connection', s));
  client.on('diagnostic', (e) => log('diagnostic', e.code, JSON.stringify(e.detail ?? '')));
  const imr = await client.registerSelfEntity({
    entityUuid: ENTITY_UUID, entityType: 'IMR', manufacturerName: 'OpenRobOps flatland',
    details: { model: 'flatland-nav2' },
    capabilities: { provides: ['status', 'odometry', 'batteryStatus'], accepts: ['move', 'cancelRequest'] },
  });
  log('registered IMR', ENTITY_UUID);

  const ros = new Ros({ url: ROSBRIDGE_URL });
  ros.on('connection', () => log('rosbridge connected'));
  // Any rosbridge failure (incl. connecting before it listens at sim startup) exits; compose restarts us.
  ros.on('error', (e) => { log('rosbridge error; exiting for restart:', e?.message ?? e); process.exit(1); });
  ros.on('close', () => { log('rosbridge closed; exiting for restart'); process.exit(1); });
  const sub = (name, messageType, cb) => new Topic({ ros, name, messageType }).subscribe(cb);

  // ---- latest-value cache; publishing happens on timers / on change ----
  const latest = { amcl: null, odom: null, battery: null, diagLevel: 0, goalActive: false };
  sub('/amcl_pose', 'geometry_msgs/msg/PoseWithCovarianceStamped', (m) => { latest.amcl = m; });
  sub('/odom', 'nav_msgs/msg/Odometry', (m) => { latest.odom = m; });
  sub('/diagnostics_toplevel_state', 'diagnostic_msgs/msg/DiagnosticStatus', (m) => { latest.diagLevel = m.level; refreshStatus(); });
  sub('/navigate_to_pose/_action/status', 'action_msgs/msg/GoalStatusArray', (m) => { latest.goalActive = goalActiveFrom(m); refreshStatus(); });
  sub('/battery_state', 'sensor_msgs/msg/BatteryState', (m) => {
    latest.battery = m;
    imr.publishBatteryStatus(toBatteryStatus(m)).catch((e) => log('battery publish failed', e.message));
    refreshStatus();
  });

  // ponytail: /amcl_pose only updates while the robot moves; good enough for a sim. Use TF map->base_link if smoothness matters.
  setInterval(() => {
    if (!latest.amcl) return;
    imr.publishOdometry(toOdometry(CCS_ID, latest.amcl, latest.odom)).catch((e) => log('odometry publish failed', e.message));
    refreshStatus();
  }, 1000 / ODOM_HZ);

  let lastStates = '';
  function refreshStatus() {
    const b = latest.battery ? toBatteryStatus(latest.battery) : { batterySoc: 1, batteryChargingState: 'UNKNOWN' };
    const states = deriveStates({
      goalActive: latest.goalActive, linear: latest.odom?.twist?.twist?.linear?.x ?? 0,
      diagLevel: latest.diagLevel, charging: b.batteryChargingState === 'CHARGING', soc: b.batterySoc,
    });
    const key = states.join(',');
    if (key === lastStates) return;
    lastStates = key;
    imr.publishStatus({ states }).catch((e) => log('status publish failed', e.message));
  }
  refreshStatus();

  // ---- ISO move -> nav2 NavigateToPose. cancelRequest is handled by the SDK and surfaces as ctx.signal. ----
  const nav = new Action({ ros, name: '/navigate_to_pose', actionType: 'nav2_msgs/action/NavigateToPose' });
  imr.onRequest('move', (action, ctx) => new Promise((resolve) => {
    log('move ->', JSON.stringify(action.properties.location));
    const goalId = nav.sendGoal(toNavGoal(action.properties),
      () => resolve(ctx.succeeded()),
      (fb) => ctx.progress({ distanceRemaining: fb.distance_remaining }),
      (err) => resolve(ctx.aborted('GENERAL_FAILURE', String(err))));
    ctx.signal.addEventListener('abort', () => { if (goalId) nav.cancelGoal(goalId); resolve(ctx.aborted('REJECTED', 'canceled')); });
  }));

  const shutdown = async () => { log('shutting down'); await imr.unregister().catch(() => {}); await client.close().catch(() => {}); process.exit(0); };
  process.on('SIGINT', shutdown);
  process.on('SIGTERM', shutdown);
}

main().catch((e) => { console.error(e); process.exit(1); });
