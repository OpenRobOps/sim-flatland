# Echo / Message Actions Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Echo ORO "Message" action strings back as an `echo` key-value on `/inorbit/custom_data`, with warning/error message values driving a Status + Incident shown in Fleet View.

**Architecture:** The ORO agent publishes `PublishToTopic` action messages as `std_msgs/String` on `/inorbit/custom_command`. The existing `republisher` node gains a subscription that republishes `echo=`-prefixed strings verbatim to `/inorbit/custom_data` (already `key=value` form, key `echo`). ORO-side config (`oro-config/config.yaml`) adds 4 actions, a DataSource, a Status, an Incident, and a Fleet View status row.

**Tech Stack:** ROS 2 Jazzy (rclpy), Docker Compose, ORO config-as-code YAML applied with the `inorbit` CLI.

**Spec:** `docs/superpowers/specs/2026-08-12-echo-message-design.md`

## Global Constraints

- Echo ONLY messages starting with `echo=`; all other custom commands (`dock=A`, `reset`, ...) must NOT be republished.
- The echoed string is republished **verbatim** (the key-value key is `echo`).
- ORO config schema facts (verified against the ORO monorepo, `web/imports/server/configAPI/` and `web/imports/shared/actions.js`):
  - Status functions accept `EQUALS` with a string param: `{function: EQUALS, params: ["error"], status: ERROR}`. First matching rule wins, so the ERROR rule must be listed before WARNING.
  - Parameterized action arguments: an argument with `input: {control: text}` is filled by the user at run time; another argument's `value` may reference it as `{{name}}` (PublishToTopic interpolates `{{...}}` from sibling arguments).
  - Valid incident severities: `SEV 0`..`SEV 3`.
- There is no automated ROS test harness in this repo; verification is done in the running sim with `ros2` CLI inside the `flatland_nav2` container.

---

### Task 1: Republisher echo behavior

**Files:**
- Modify: `republisher/republisher/republisher_node.py`

**Interfaces:**
- Consumes: `/inorbit/custom_command` (`std_msgs/String`), published by the ORO agent for `PublishToTopic` actions (the C++ battery plugin already consumes `reset`/`charge`/`dock…` from this topic — do not touch it).
- Produces: `echo=<text>` messages on `/inorbit/custom_data`, which the agent forwards to ORO as key-values. Task 2's DataSource keys off `echo`.

- [ ] **Step 1: Add the subscription and callback**

In `republisher/republisher/republisher_node.py`, add to `__init__` (after the existing subscriptions):

```python
        # ORO "Message" actions publish 'echo=<text>' on the custom command
        # topic; echo it back verbatim as a custom_data key-value (key 'echo').
        self.create_subscription(
            String, '/inorbit/custom_command', self._on_custom_command, 10)
```

and add the callback (next to the other `_on_*` methods):

```python
    def _on_custom_command(self, msg: String):
        if msg.data.startswith('echo='):
            self.pub.publish(msg)
```

- [ ] **Step 2: Rebuild and start the sim**

```bash
cd /home/herchu/inorbit/flatland
docker compose build flatland-nav2
docker compose up -d
```

Expected: containers start; `docker compose ps` shows `flatland_nav2` running.

- [ ] **Step 3: Verify echo (positive case)**

In one terminal, watch custom_data:

```bash
docker exec flatland_nav2 bash -ic "ros2 topic echo /inorbit/custom_data"
```

In another, publish an echo command:

```bash
docker exec flatland_nav2 bash -ic "ros2 topic pub -1 /inorbit/custom_command std_msgs/msg/String \"data: 'echo=hi there'\""
```

Expected: the watcher prints `data: echo=hi there` (battery key-values will also stream by; look for the `echo=` line).

Note: if `bash -ic` does not source the ROS environment in this image, use `bash -c "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 topic ..."` instead.

- [ ] **Step 4: Verify non-echo messages are ignored (negative case)**

```bash
docker exec flatland_nav2 bash -ic "ros2 topic pub -1 /inorbit/custom_command std_msgs/msg/String \"data: 'not-an-echo'\""
```

Expected: NO `not-an-echo` line appears on the `/inorbit/custom_data` watcher (battery keys keep streaming; that's fine).

- [ ] **Step 5: Commit**

```bash
cd /home/herchu/inorbit/flatland
git add republisher/republisher/republisher_node.py
git commit -m "republisher: echo 'echo=' custom commands back as custom_data"
```

---

### Task 2: ORO config — Message actions, DataSource, Status, Incident, Fleet View

**Files:**
- Modify: `oro-config/config.yaml`

**Interfaces:**
- Consumes: the `echo` key-value produced by Task 1.
- Produces: ORO entities `MessageHello`, `MessageWarning`, `MessageError`, `MessageCustom` (actions), `message` (DataSource + Status + Incident), and a `message` row in the fleetStatus widget.

- [ ] **Step 1: Add the four ActionDefinitions**

In `oro-config/config.yaml`, append after the `DockD` ActionDefinition (keep the `---` separators):

```yaml
---
apiVersion: v0.1
kind: ActionDefinition
metadata:
  id: MessageHello
  scope: ''
spec:
  label: Hello
  group: Message
  type: PublishToTopic
  arguments:
  - name: message
    value: echo=Hello from ORO!
  description: 'Echo a hello message back from the robot'
  lock: true
---
apiVersion: v0.1
kind: ActionDefinition
metadata:
  id: MessageWarning
  scope: ''
spec:
  label: Warning
  group: Message
  type: PublishToTopic
  arguments:
  - name: message
    value: echo=warning
  description: 'Set the message status to warning'
  lock: true
---
apiVersion: v0.1
kind: ActionDefinition
metadata:
  id: MessageError
  scope: ''
spec:
  label: Error
  group: Message
  type: PublishToTopic
  arguments:
  - name: message
    value: echo=error
  description: 'Set the message status to error'
  lock: true
---
apiVersion: v0.1
kind: ActionDefinition
metadata:
  id: MessageCustom
  scope: ''
spec:
  label: Message
  group: Message
  type: PublishToTopic
  arguments:
  - name: message
    value: echo={{text}}
  - name: text
    type: string
    input:
      control: text
  description: 'Echo a custom message back from the robot'
  lock: true
```

- [ ] **Step 2: Add DataSource, Status and Incident definitions**

Append after the `nav2Health` DataSourceDefinition (before the DashboardDefinition section):

```yaml
---
apiVersion: v0.1
kind: DataSourceDefinition
metadata:
  id: message
  scope: ""
spec:
  label: Message
  source:
    keyValue:
      key: "echo"
```

Append to the Status/Incident section at the end of the file (after the `batteryPercentage` IncidentDefinition):

```yaml
---
# Status on the echoed message: 'error' and 'warning' values raise the
# corresponding status; any other message is OK. The ERROR rule is listed
# first because the evaluator stops at the first matching rule.
apiVersion: v0.1
kind: StatusDefinition
metadata:
  id: message
  scope: ''
spec:
  rules:
  - function: EQUALS
    params: ["error"]
    status: ERROR
  - function: EQUALS
    params: ["warning"]
    status: WARNING
---
apiVersion: v0.1
kind: IncidentDefinition
metadata:
  id: message
  scope: ''
spec:
  label: Message
  labelTemplate: '{{robotName}}: message {{value}}'
  warning:
    severity: SEV 2
  error:
    severity: SEV 1
```

Note: if `inorbit apply` rejects `labelTemplate: '{{robotName}}: message {{value}}'` (the `{{value}}` placeholder is unverified), fall back to `'{{robotName}}: message'`.

- [ ] **Step 3: Add the Fleet View status row**

In the `fleet` DashboardDefinition, `fleetStatus` widget, extend `statuses`:

```yaml
        statuses:
        - id: cpuLoadPercentage
          label: CPU
        - id: diskUsagePercentage
          label: Disk
        - id: batteryPercentage
          label: Battery
        - id: message
          label: Message
```

- [ ] **Step 4: Apply the config**

```bash
cd /home/herchu/inorbit/flatland/oro-config
source setup-oro-local.sh
inorbit apply -f config.yaml
```

Expected: apply succeeds for every document, no validation errors. If a schema error is reported for the new documents, stop and report it verbatim (don't guess alternative syntax).

- [ ] **Step 5: Verify in ORO end-to-end**

With the sim + agent running (`docker compose up -d` from Task 1) and local ORO at http://localhost:3000:

1. Ops dashboard → Actions widget shows group **Message** with Hello / Warning / Error / Message.
2. Run **Hello** → Robot dashboard "Key Value pairs" widget shows `echo` = `Hello from ORO!`.
3. Run **Message**, type `all good` when prompted → `echo` = `all good`, no incident.
4. Run **Warning** → a "Message" incident appears at SEV 2 (warning).
5. Run **Error** → the incident escalates to SEV 1 (error).
6. Fleet dashboard → Fleet Status widget shows a **Message** row reflecting the error.
7. Run **Hello** again → status returns to OK and the incident resolves.

Expected: all seven observations hold. If a step fails, stop and report which one.

- [ ] **Step 6: Commit**

```bash
cd /home/herchu/inorbit/flatland
git add oro-config/config.yaml
git commit -m "oro-config: Message actions with echo status + incident"
```

---

### Task 3: Docs — welcome dashboard text

**Files:**
- Modify: `oro-config/welcome-action.md`
- Regenerate: `oro-config/config.yaml` (via `node embed-welcome-content.mjs`)

**Interfaces:**
- Consumes: the Message actions and `echo` key from Tasks 1–2.
- Produces: updated Welcome dashboard copy.

- [ ] **Step 1: Update welcome-action.md**

In `oro-config/welcome-action.md`, add a **Message** block after the **Battery** block:

```markdown
**Message**  
[HELLO](/dashboards/ops), [MESSAGE](/dashboards/ops): Echoes a message back from the robot as an `echo` key-value  
[WARNING](/dashboards/ops), [ERROR](/dashboards/ops): Sets the message status, raising an incident visible in Fleet View
```

and in the "Key-value pairs" paragraph, extend the first sentence to mention the echo key, changing:

```markdown
Additionally, the robot reports key-value pairs as custom data elements, including `battery_charging`, `battery_percentage` and `battery_voltage` for battery state, and `estimated_time_remaining` from the nav2 stack. 
```

to:

```markdown
Additionally, the robot reports key-value pairs as custom data elements, including `battery_charging`, `battery_percentage` and `battery_voltage` for battery state, `estimated_time_remaining` from the nav2 stack, and `echo` for messages echoed back by the Message actions. 
```

- [ ] **Step 2: Regenerate config.yaml and re-apply**

```bash
cd /home/herchu/inorbit/flatland/oro-config
node embed-welcome-content.mjs
git diff config.yaml   # expect ONLY the welcome dashboard text to change
source setup-oro-local.sh
inorbit apply -f config.yaml
```

Expected: regeneration only touches the Welcome dashboard text widgets (Task 2's additions remain intact); apply succeeds.

- [ ] **Step 3: Verify**

Open the Welcome dashboard in ORO: the Action widget lists the Message group and the key-value paragraph mentions `echo`.

- [ ] **Step 4: Commit**

```bash
cd /home/herchu/inorbit/flatland
git add oro-config/welcome-action.md oro-config/config.yaml
git commit -m "oro-config: document Message actions in welcome dashboard"
```
