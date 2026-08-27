import { test } from 'node:test';
import assert from 'node:assert/strict';
import { assertValid } from '@openrobops/iso21423/schema';
import {
  toOdometry, toBatteryStatus, deriveStates, goalActiveFrom, toNavGoal, yawFromQuaternion, parseKeyValue, toCustomData, throttle,
  circlePolygon, imrDetails } from '../src/mapping.js';

const ts = () => new Date().toISOString();

test('odometry from amcl_pose + odom validates and round-trips yaw', () => {
  const amcl = { pose: { pose: { position: { x: 3, y: 4, z: 0 }, orientation: { x: 0, y: 0, z: Math.SQRT1_2, w: Math.SQRT1_2 } } } };
  const odom = { twist: { twist: { linear: { x: 0.5 }, angular: { z: -0.1 } } } };
  const o = toOdometry('0b1c2d3e-4f50-4a6b-8c7d-9e0f1a2b3c4d', amcl, odom);
  assertValid('odometry', { ...o, timestamp: ts() });
  assert.equal(o.pose.locationPoint.x, 3);
  assert.ok(Math.abs(o.pose.orientation.yaw - Math.PI / 2) < 1e-9);
  assert.deepEqual(o.velocity, { linear: 0.5, angular: -0.1 });
});

test('battery maps power_supply_status and clamps NaN percentage', () => {
  const b = toBatteryStatus({ percentage: 0.42, voltage: 12.1, current: -1.5, power_supply_status: 1 });
  assertValid('batteryStatus', { ...b, timestamp: ts() });
  assert.equal(b.batterySoc, 0.42);
  assert.equal(b.batteryChargingState, 'CHARGING');
  assert.equal(toBatteryStatus({ percentage: NaN, voltage: NaN, power_supply_status: 0 }).batterySoc, 0);
});

test('states: idle vs moving vs charging/low battery', () => {
  assert.deepEqual(deriveStates({ goalActive: false, linear: 0, diagLevel: 0, charging: false, soc: 0.9 }), ['MODE_AUTO', 'READY', 'IDLE']);
  assert.deepEqual(deriveStates({ goalActive: true, linear: 0.3, diagLevel: 2, charging: true, soc: 0.1 }),
    ['MODE_AUTO', 'NOT_READY', 'FORWARD', 'CHARGING', 'LOW_BATTERY']);
  assertValid('entityStatus', { entityId: '5f8c1e2a-6b3d-4a9f-8e11-2c7d9a4b1f00', timestamp: ts(),
    states: deriveStates({ goalActive: true, linear: 0, diagLevel: 0, charging: false, soc: 0.5 }) });
});

test('states: docking flag', () => {
  assert.deepEqual(deriveStates({ goalActive: true, linear: 0.2, diagLevel: 0, charging: false, soc: 0.5, docking: true }),
    ['MODE_AUTO', 'READY', 'FORWARD', 'DOCKING']);
});

test('dock -> nav goal uses dockLocation', () => {
  const g = toNavGoal({ dockLocation: { ccsId: '0b1c2d3e-4f50-4a6b-8c7d-9e0f1a2b3c4d', x: 9, y: 18.5, z: 0 }, dockActions: ['CHARGE'] });
  assert.deepEqual(g.pose.pose.position, { x: 9, y: 18.5, z: 0 });
});

test('goal active from GoalStatusArray', () => {
  assert.equal(goalActiveFrom({ status_list: [{ status: 4 }, { status: 2 }] }), true);
  assert.equal(goalActiveFrom({ status_list: [{ status: 4 }] }), false);
});

test('move -> nav goal quaternion matches yaw', () => {
  const g = toNavGoal({ location: { ccsId: '0b1c2d3e-4f50-4a6b-8c7d-9e0f1a2b3c4d', x: 1, y: 2, z: 0 }, orientation: { yaw: 1, pitch: 0, roll: 0 } });
  assert.equal(g.pose.header.frame_id, 'map');
  assert.ok(Math.abs(yawFromQuaternion(g.pose.pose.orientation) - 1) < 1e-9);
});

test('custom_data key=value parsing keeps = inside the value and rejects keyless lines', () => {
  assert.deepEqual(parseKeyValue('echo=a=b'), ['echo', 'a=b']);
  assert.deepEqual(parseKeyValue('battery_charging=true'), ['battery_charging', 'true']);
  assert.equal(parseKeyValue('=x'), null);
  assert.equal(parseKeyValue('nothing'), null);
  const d = toCustomData({ echo: 'hi' });
  assert.deepEqual(d.values, { echo: 'hi' });
  assert.ok(!Number.isNaN(Date.parse(d.timestamp)));
});

test('throttle: runs the first call, drops calls inside the window, runs again after it', () => {
  const calls = [];
  const realNow = Date.now;
  let now = 1_000_000;
  Date.now = () => now;
  try {
    const t = throttle((v) => calls.push(v), 2000);
    t('a'); t('b'); now += 1999; t('c'); now += 1; t('d'); t('e');
  } finally {
    Date.now = realNow;
  }
  assert.deepEqual(calls, ['a', 'd']);
});

test('imrDetails is a spec-valid IMR identity with a 16-point footprint of the 0.22 m body', () => {
  const d = imrDetails({ uuid: '7b1a9c3e-1111-4222-8333-444455556666', version: '1.2.3' });
  assert.equal(d.imrModel, 'flatland-nav2');
  assert.equal(d.imrSerialNumber, '7b1a9c3e-1111-4222-8333-444455556666');
  assert.equal(d.imrFootprint.length, 16);
  for (const p of d.imrFootprint) assert.ok(Math.abs(Math.hypot(p.x, p.y) - 0.22) < 1e-3);
  assert.deepEqual(d.imrWorkingArea, d.imrFootprint);
  assert.equal(d.imrHeight, 0.4);
  assert.deepEqual(d.softwareVersions, [{ moduleName: 'iso-agent', moduleVersion: '1.2.3' }]);
  // The compiled schema's entityIdentity.details is `type: "object"` (unconstrained), so it won't
  // catch a wrong ImrDetails field name — assert moduleName/moduleVersion (SoftwareVersion, above)
  // and the other field names explicitly instead of relying on assertValid for them.
  assertValid('entityIdentity', {
    id: '7b1a9c3e-1111-4222-8333-444455556666', timestamp: ts(), entityType: 'IMR', manufacturerName: 'x',
    capabilities: { provides: [], accepts: { requests: [] } }, details: d,
  });
});

test('circlePolygon: n points evenly spaced on the given radius', () => {
  const pts = circlePolygon(1, 4);
  assert.equal(pts.length, 4);
  assert.deepEqual(pts[0], { x: 1, y: 0 });
  assert.ok(Math.abs(pts[1].x) < 1e-3 && Math.abs(pts[1].y - 1) < 1e-3);
});
