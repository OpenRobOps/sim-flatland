# Echo / Message Actions — Design

**Date:** 2026-08-12
**Status:** Approved (pre-implementation)

## Goal

Let an operator send a message to the simulated robot from ORO and see it echoed back as robot data. An ORO `PublishToTopic` action publishes `echo=<text>` on `/inorbit/custom_command`; the sim echoes the string back as a key-value on `/inorbit/custom_data` (key `echo`). On top of that, "warning" and "error" message values drive a Status + Incident visible in Fleet View.

## Non-goals

- Echoing other custom commands (`dock=A`, `reset`, ...) — only `echo=`-prefixed messages are republished.
- A new ROS node — the existing `republisher` node is extended.
- New dashboards or widgets — the existing "Key Value pairs" widget already displays the `echo` key; Fleet View gets one extra status row.

## Part 1 — Echo behavior (implement + verify first)

`republisher/republisher/republisher_node.py`:

- Add a subscription to `/inorbit/custom_command` (`std_msgs/String`) — the topic where the ORO agent publishes `PublishToTopic` action messages (the battery plugin already consumes `reset`/`charge`/`dock…` from it).
- If `msg.data` starts with `echo=`, republish the string **verbatim** on `/inorbit/custom_data` — it is already in `key=value` form, so the key-value is `echo=<text>`. Ignore everything else.

Verification: run the sim, then

```bash
ros2 topic pub -1 /inorbit/custom_command std_msgs/msg/String "data: 'echo=hi'"
ros2 topic echo /inorbit/custom_data   # expect data: 'echo=hi'
```

and confirm a non-echo message (e.g. `data: 'reset'`) is not republished.

## Part 2 — ORO config (`oro-config/config.yaml`)

Four `ActionDefinition`s in group **Message**, all `type: PublishToTopic`, mirroring the existing Dock/Battery actions:

| id | label | message argument |
|---|---|---|
| `MessageHello` | Hello | `echo=Hello from ORO!` |
| `MessageWarning` | Warning | `echo=warning` |
| `MessageError` | Error | `echo=error` |
| `MessageCustom` | Message | parameterized — user-typed text, published as `echo=<text>` |

Plus:

- **DataSourceDefinition** `message`: `keyValue` key `echo`, label "Message" (needed as the trigger for Status/Incident).
- **StatusDefinition** `message`: value `error` → ERROR, value `warning` → WARNING, anything else OK. Error rule listed first (first-match evaluation, same reason as the battery thresholds).
- **IncidentDefinition** `message`: label "Message", warning → SEV 2, error → SEV 1, no auto/manual actions.
- **Fleet View**: add `{id: message, label: Message}` to the `fleetStatus` widget statuses (alongside CPU/Disk/Battery).

### Open items (resolve against ORO's config schema during implementation; flag if unsupported, don't guess)

1. Exact argument syntax for a **user-input parameterized** `PublishToTopic` action (existing actions all use fixed `value`s).
2. The **string-equality** status function name (existing `StatusDefinition` only shows numeric `BELOW`).

## Part 3 — Docs

Add the Message action group and the `echo` key-value to `oro-config/welcome-action.md` (embedded into `config.yaml` via `embed-welcome-content.mjs`).

## Files

| Path | Change |
|---|---|
| `republisher/republisher/republisher_node.py` | Subscribe to `/inorbit/custom_command`, republish `echo=`-prefixed strings |
| `oro-config/config.yaml` | 4 ActionDefinitions, DataSource + Status + Incident `message`, fleetStatus row |
| `oro-config/welcome-action.md` | Document Message actions and `echo` key |

## Order

1. Part 1, verified end-to-end in the running sim.
2. Part 2, applied via the existing setup scripts and verified in ORO (actions fire, status/incident trigger on warning/error, Fleet View row shows).
3. Part 3.
