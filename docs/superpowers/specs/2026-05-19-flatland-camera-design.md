# Wolfenstein-Style Camera Plugin for Flatland — Design

**Date:** 2026-05-19
**Status:** Approved (pre-implementation)

## Goal

Give the simulated robot a forward-facing camera that publishes synthetic RGB images to ROS 2, rendered from the 2D Flatland world using Wolfenstein 3D-style raycasting. The image is visible in rviz by default and is usable by any downstream consumer that subscribes to standard `sensor_msgs/Image` (the InOrbit agent, image_view, image_proc, etc.).

## Non-goals

- Photo-realistic rendering. Walls are flat-shaded with simple depth and directional shading — no textures, no lighting model, no shadows.
- Floor projected from the map (the floor is a solid color).
- Multiple cameras per robot. The plugin supports configuring more than one Camera plugin instance, but the sample world ships one forward camera.
- Depth (`sensor_msgs/Image` encoding `32FC1`) or point-cloud output. RGB only in this iteration.
- Driving the camera from anywhere other than the model body it's attached to. No standalone "free camera" mode.
- ROS 2 lifecycle plugins, parameter changes at runtime — all config is YAML at startup, matching Battery and Laser.

## Approach

A new C++ `Camera` plugin lives in `plugins/camera.{h,cpp}`, registered through the same patch-based mechanism that registers `Battery`. It uses `b2World::RayCast` (the same primitive `Laser` uses) to cast one ray per image column, projects each hit into a wall column on a preallocated image buffer, fills sky / floor with solid colors, and publishes `sensor_msgs/Image`, `sensor_msgs/CameraInfo`, and `sensor_msgs/CompressedImage` on configurable topics. The rviz default layout gains an Image display so the camera output is visible immediately on `docker compose up`.

Alternatives considered and rejected during brainstorming:

- **External ROS 2 node reading `/scan` + `/map`** — couples camera FOV to laser FOV, decouples sim clocks. Rejected.
- **Offline post-processing of bagfiles** — not real-time, defeats the goal of "publishing camera images to ROS topics during the sim." Rejected.

## Architecture

```
flatland_server (Box2D world)
   │
   ├── Laser plugin       ── b2World::RayCast ──► /scan
   ├── Battery plugin     ──────────────────────► /battery_state, ...
   └── Camera plugin (NEW)
         ├── b2World::RayCast (one ray per image column, thread-pooled)
         ├── per-column projection → cv::Mat (rgb8)
         └── publishes:
               /image_raw            sensor_msgs/Image
               /image_raw/compressed sensor_msgs/CompressedImage (jpeg)
               /camera_info          sensor_msgs/CameraInfo
               (optional) /tf        base_link → camera_link
```

The plugin is a `flatland_server::ModelPlugin` with the same two-method lifecycle the existing plugins use:

- `OnInitialize(YAML::Node)` — parse params, validate, allocate image/buffer, precompute per-column ray directions and `cos_correction` table, build static `CameraInfo`, create publishers and (optionally) TF broadcaster.
- `BeforePhysicsStep(Timekeeper&)` — gated by `UpdateTimer` at `update_rate`. If any image-topic subscriber exists, perform the raycast pass, render the frame, publish. Always broadcast TF if enabled.

## Files

| Path | Purpose |
|---|---|
| `plugins/camera.h` | `flatland_plugins::Camera` class declaration |
| `plugins/camera.cpp` | Implementation: parsing, raycast loop, column projection, shading, publishing |
| `plugins/test/camera_math_test.cpp` | gtest binary for pure projection / shading math (opt-in; not run in CI today) |
| `patches/flatland/0014-flatland_plugins-register-camera-plugin-source-proje.patch` | Mirrors patch `0011` for battery: adds `src/camera.cpp` to the `flatland_plugins_lib` sources and adds `find_package(OpenCV REQUIRED)` + OpenCV target link |
| `patches/flatland/0015-flatland_plugins-register-camera-plugin-class-projec.patch` | Mirrors patch `0012` for battery: registers `flatland_plugins::Camera` with pluginlib in `flatland_plugins/plugin_description.xml` |
| `Dockerfile` | Add `COPY plugins/camera.h` and `COPY plugins/camera.cpp` lines next to the existing battery copies (before the patch-apply step) |
| `worlds/turtlebot.model.yaml` | New `- type: Camera ...` block under `plugins:` |
| `config/flatland_rviz.rviz` | New `rviz_default_plugins/Image` display pre-subscribed to `/image_raw` |
| `README.md` | New "Camera Simulation" section (parameters, topics, verification commands) |

## Build dependencies

`flatland_plugins/CMakeLists.txt` currently has no `find_package(OpenCV)`. Patch 0014 adds it, plus the corresponding `target_link_libraries(flatland_plugins_lib ... ${OpenCV_LIBS})`. `sensor_msgs` is already pulled in transitively by Laser's `LaserScan` and Battery's `BatteryState` — no new ROS package dependencies needed for `Image`, `CameraInfo`, or `CompressedImage`. No `cv_bridge` dependency: the `Image` message is built directly from `cv::Mat::data` to keep the dependency footprint minimal.

## YAML parameters

Added to `worlds/turtlebot.model.yaml` under the existing `plugins:` list:

```yaml
- type: Camera
  name: front_camera
  body: base_link
  frame: camera_link              # TF frame (namespaced like laser's frame)
  topic: image_raw                # base topic; /compressed and /camera_info are siblings
  origin: [0.18, 0.0, 0.0]        # mount offset (x, y, theta) relative to body
  update_rate: 10.0               # Hz

  # Optics
  width: 320                      # pixels
  height: 240
  fov_deg: 90.0                   # horizontal FOV
  range: 8.0                      # max ray distance (m)

  # Raycast filter
  layers: ["all"]                 # which Box2D layers to hit (Laser convention)
  ignore_self: true               # skip fixtures belonging to this model

  # Look
  wall_height: 1.0                # virtual wall height (m), used in column projection
  eye_height: 0.5                 # camera eye height (m), shifts horizon
  shade_min: 0.15                 # brightness at range
  shade_max: 1.00                 # brightness at distance 0
  directional_shading: 0.85       # multiplier for "side-lit" walls (uses hit normal)
  sky_color:   [120, 140, 160]    # RGB 0-255
  floor_color: [60,  55,  50]
  fog_color:   [30,  30,  30]     # distance-based fog blend

  # Publishing
  broadcast_tf: false             # if true, publish base_link → camera_link
  publish_camera_info: true
  publish_compressed: true
  jpeg_quality: 75
```

### Defaults (when key is omitted)

| Key | Default |
|---|---|
| `frame` | `camera_link` |
| `topic` | `image_raw` |
| `origin` | `[0, 0, 0]` |
| `update_rate` | `10.0` |
| `width` | `320` |
| `height` | `240` |
| `fov_deg` | `90.0` |
| `range` | `8.0` |
| `layers` | `["all"]` |
| `ignore_self` | `true` |
| `wall_height` | `1.0` |
| `eye_height` | `0.5` |
| `shade_min` | `0.15` |
| `shade_max` | `1.0` |
| `directional_shading` | `0.85` |
| `sky_color` | `[120, 140, 160]` |
| `floor_color` | `[60, 55, 50]` |
| `fog_color` | `[30, 30, 30]` |
| `broadcast_tf` | `false` |
| `publish_camera_info` | `true` |
| `publish_compressed` | `true` |
| `jpeg_quality` | `75` |

### Validation

All numeric params with physical meaning must be `> 0` (`width`, `height`, `update_rate`, `range`, `fov_deg`, `wall_height`). `fov_deg` must be `< 180`. Color triples must be exactly three ints in `[0, 255]`. `jpeg_quality` must be in `[1, 100]`. `layers` is resolved via `GetModel()->GetCfr()->GetCategoryBits(...)`; invalid layer names throw with the offending names listed (identical behavior to Laser). Any validation failure throws `YAMLException`, which `flatland_server` reports loudly at startup.

## Rendering algorithm

Per-frame work inside `BeforePhysicsStep`, after the `UpdateTimer` rate gate and a subscriber check identical to Laser's:

### 1. Pose & ray-direction table

Camera origin and yaw in world frame are computed the way Laser does: take `body_->GetPhysicsBody()->GetTransform()` and compose with the static body→camera matrix built once in `OnInitialize`.

Per-column ray *directions* in the camera frame are precomputed once into a table of length `width`:

```
for col in 0..width:
    a = (fov/2) - (col + 0.5) * (fov / width)   # +fov/2 at col=0 (left), -fov/2 at col=width-1
    ray_dir_cam[col]    = (cos(a), sin(a))
    cos_correction[col] = cos(a)                # used to kill fisheye later
```

Each frame these directions are rotated by the current camera yaw (a 2×2 multiply per column) and added to the origin to produce raycast endpoints at distance `range`.

### 2. Raycast pass

Mirror `laser.cpp`'s thread-pool pattern. One task per column casts a ray and writes its result into a per-column results array. The callback is a small variant of `LaserCallback`:

```cpp
struct CameraRayCb : public b2RayCastCallback {
  Camera *parent;
  bool did_hit = false;
  float fraction = 1.0f;
  b2Vec2 normal;
  uint16_t hit_category = 0;

  float ReportFixture(b2Fixture *fix, const b2Vec2 &, const b2Vec2 &n, float f) override {
    uint16_t cat = fix->GetFilterData().categoryBits;
    if (!(cat & parent->layers_bits_)) return -1.0f;          // layer filter
    if (fix->IsSensor()) return -1.0f;                         // ignore sensors
    if (parent->ignore_self_ &&
        fix->GetBody() == parent->self_b2body_) return -1.0f;  // ignore own model
    did_hit = true;
    fraction = f;
    normal = n;
    hit_category = cat;
    return f;                                                  // tighten the ray
  }
};
```

Per column: `dist_raw = fraction * range` if hit, otherwise marked as "miss".

### 3. Fisheye correction → column geometry

```
dist        = dist_raw * cos_correction[col]
focal_y     = (height/2) / tan(fov_v/2)      # fov_v from aspect ratio + fov_h
column_h_px = clamp(focal_y * wall_height / dist, 0, height)
horizon_row = height/2 + (eye_height - wall_height/2) * focal_y / dist
top         = clamp(horizon_row - column_h_px/2, 0, height)
bottom      = clamp(horizon_row + column_h_px/2, 0, height)
```

`fov_v` is derived from `fov_h` and the aspect ratio so square objects render square.

### 4. Shading

```
t       = clamp(dist / range, 0, 1)            # 0=near, 1=far
shade   = lerp(shade_max, shade_min, t)        # depth darkening
n_dot   = abs(dot(ray_dir_world, normal))      # 1=head-on, 0=grazing
if n_dot < 0.5: shade *= directional_shading   # side-lit walls slightly darker
wall_rgb = (255 * shade, 255 * shade, 255 * shade)
wall_rgb = lerp(wall_rgb, fog_color, t * t)    # quadratic fog falloff
```

### 5. Image composition

One `cv::Mat(height, width, CV_8UC3)` is allocated once in `OnInitialize` and reused every frame. Per column:

- Rows `[0, top)`            → `sky_color`
- Rows `[top, bottom)`       → `wall_rgb` (or omitted if no hit)
- Rows `[bottom, height)`    → `floor_color`

Each fill is a memset-style run; total per-frame work is `O(width × height)` bytes.

### 6. Publishing

- **`sensor_msgs/Image`** — built directly (no `cv_bridge` dependency). `encoding = "rgb8"`, `step = width * 3`, `data.assign(mat.data, mat.data + step*height)`. `header.stamp = timekeeper.GetSimTime()`. `header.frame_id` = namespaced `frame` (`<model_namespace>/camera_link` via `GetModel()->NameSpaceTF(frame_)`).
- **`sensor_msgs/CameraInfo`** — built once in `OnInitialize` (`K = [fx, 0, cx; 0, fy, cy; 0,0,1]` with `fx = fy = focal_y`, `cx = width/2`, `cy = height/2`; distortion = zeros; `distortion_model = "plumb_bob"`). Only the stamp is updated per frame.
- **`sensor_msgs/CompressedImage`** (when `publish_compressed: true`) — `cv::imencode(".jpg", mat, buf, {cv::IMWRITE_JPEG_QUALITY, jpeg_quality})`, copy `buf` into `msg.data`, `format = "jpeg"`, same stamp + frame as Image.

### Performance gate

If `image_pub_->get_subscription_count() + camera_info_pub_->get_subscription_count() + compressed_pub_->get_subscription_count() == 0`, skip the entire render+publish pass. TF (if enabled) is still broadcast. This mirrors Laser's no-subscriber early exit.

## Threading

Reuses the `ThreadPool` pattern from `laser.cpp` for parallel column raycasts. `b2World::RayCast` is read-only-safe across simultaneous columns within a single cast pass — Laser already exercises this guarantee against the same world. No new locks needed.

## Error handling

| Case | Behavior |
|---|---|
| Missing / invalid YAML param | `YAMLException` at load — flatland_server aborts loudly. |
| `fov_deg >= 180`, negative dimensions, bad color triple, bad jpeg_quality | `YAMLException` at load. |
| Unknown collision layer name in `layers:` | `YAMLException` listing the bad name(s). Same path Laser uses. |
| Ray misses everything within `range` | Column has only sky + floor, no wall. |
| `dist < 1e-4` (camera clipping a wall) | Treat as full-column wall to avoid divide-by-zero. |
| `top < 0` or `bottom > height` | Clamp to image bounds before writing. |
| Degenerate hit normal | Skip directional shading for that column (treat as head-on). |
| `cv::imencode` fails | `RCLCPP_WARN_THROTTLE(1.0)` once per second; skip compressed publish that frame; raw publish continues. |
| No subscribers on any image topic | Skip render+publish entirely; TF (if enabled) still goes out. |

`broadcast_tf` defaults to `false` per the chosen ROS topic set. With this default, `header.frame_id` references `camera_link` but no transform for that frame is published by this plugin — the rviz Image display still works (it doesn't need TF), but the rviz Camera display would not align. README documents the flip.

## rviz integration

A new entry is added to `config/flatland_rviz.rviz` under `Displays:`:

```yaml
- Class: rviz_default_plugins/Image
  Enabled: true
  Image Topic: /image_raw
  Max Value: 1
  Median window: 5
  Min Value: 0
  Name: Camera Image
  Normalize Range: true
  Queue Size: 2
  Transport Hint: raw
  Unreliable: false
  Value: true
```

Placed after the Laser-related display so the panel order is: Map → Robot → Laser → **Camera Image** → Battery → costmaps. On `docker compose up` the panel becomes visible immediately once the simulator starts publishing.

## Testing strategy

### Unit tests

`plugins/test/camera_math_test.cpp` exercises pure functions extracted into a header (no Box2D, no ROS):

- `column_h_px` clamps to image height when `dist → 0`.
- `column_h_px → 0` when `dist → range`.
- Fisheye correction equalizes opposite-edge rays against a parallel wall.
- Shading returns `shade_max` at `dist = 0` and ≤ `shade_min * directional_shading` at `dist = range`.

The test binary is added to `flatland_plugins/CMakeLists.txt` through the patch but is not wired into CI; it runs locally via `colcon test --packages-select flatland_plugins`.

### Integration verification (README checklist)

A "Verifying the camera" subsection added to the README, modeled on the Battery section's verification commands:

```bash
# Topics exist and publish at the configured rate
docker compose exec flatland-nav2 ros2 topic hz /image_raw         # ~10 Hz
docker compose exec flatland-nav2 ros2 topic hz /image_raw/compressed
docker compose exec flatland-nav2 ros2 topic echo /camera_info --once

# Image fields match config
docker compose exec flatland-nav2 ros2 topic echo /image_raw --field width    --once   # 320
docker compose exec flatland-nav2 ros2 topic echo /image_raw --field height   --once   # 240
docker compose exec flatland-nav2 ros2 topic echo /image_raw --field encoding --once   # rgb8

# Visual: rviz "Camera Image" panel shows walls + horizon
xhost +local:docker && docker compose up
```

Plus one behavioral check: drive the robot toward a wall — walls grow; rotate — walls slide horizontally; back away — walls shrink and darken.

### Failure modes (manual)

- Bad config (`fov_deg: 200`) → flatland_server fails fast with a readable error.
- Camera origin inside a wall → image is fully filled with the near-wall color, no crash.
- `update_rate: 1.0` → `ros2 topic hz /image_raw` reports ~1 Hz.
- `broadcast_tf: true` → `ros2 run tf2_tools view_frames` shows `base_link → camera_link`.

### No-subscribers performance

With the simulator running and no image-topic subscribers, `top` inside the container should show CPU usage indistinguishable from a build without the Camera plugin loaded — confirms the early-exit gate.

## Logging

`OnInitialize` ends with one `RCLCPP_INFO` line summarizing resolved config — resolution, FOV, range, topic names, `broadcast_tf` value. This is louder than Laser's `RCLCPP_DEBUG` so the camera's presence and configuration are obvious in `docker compose up` output.

## Open questions / follow-ups (not in scope for this iteration)

- Texture-mapped walls (Wolfenstein-faithful) — would require sampling a texture per pixel based on the hit point along the wall; needs assets and per-fixture material metadata.
- Per-layer wall colors (e.g., red zone vs. blue zone) — easy follow-up using `hit_category` from the callback; deferred until a multi-layer world exists.
- Depth-image output (`32FC1`) for use with Nav2 depth costmap — would reuse the raycast pass with a different encoder.
- Floor projected from the occupancy grid — needs per-pixel map lookups beneath the horizon; significantly more compute, deferred.
