# Wolfenstein-Style Camera Plugin Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a synthetic forward-facing camera plugin to Flatland that publishes RGB images to ROS 2 using Wolfenstein 3D-style raycasting, visible in rviz on `docker compose up`.

**Architecture:** New C++ `flatland_server::ModelPlugin` in `plugins/camera.{h,cpp}` registered through two new patches that mirror the existing battery-plugin patches (0011/0012). Uses `b2World::RayCast` (one ray per image column) feeding a column projection + depth shader. Publishes `sensor_msgs/Image`, `sensor_msgs/CameraInfo`, and `sensor_msgs/CompressedImage`. Pure math is extracted into a header-only file so it can be unit-tested on the host without Docker.

**Tech Stack:** C++17, ROS 2 Jazzy, flatland_server, Box2D, OpenCV (for JPEG encode), Docker / docker compose.

**Spec:** `docs/superpowers/specs/2026-05-19-flatland-camera-design.md`

---

## File Structure

| Path | Action | Responsibility |
|---|---|---|
| `plugins/camera_math.h` | Create | Header-only pure math (column geometry, shading). No ROS/Box2D deps. Host-testable. |
| `plugins/test/camera_math_test.cpp` | Create | Standalone host-runnable test for `camera_math.h`. Uses `<cassert>` only. |
| `plugins/camera.h` | Create | `flatland_plugins::Camera` class declaration. |
| `plugins/camera.cpp` | Create | YAML parsing, raycast loop, rendering, ROS publishing, optional TF. |
| `patches/flatland/0014-flatland_plugins-register-camera-plugin-source-proje.patch` | Create | Adds `src/camera.cpp` to `flatland_plugins_lib`; adds `find_package(OpenCV REQUIRED)` and links `${OpenCV_LIBS}`. |
| `patches/flatland/0015-flatland_plugins-register-camera-plugin-class-projec.patch` | Create | Registers `flatland_plugins::Camera` with pluginlib in `plugin_description.xml`. |
| `Dockerfile` | Modify | Add `COPY plugins/camera.h …/include/flatland_plugins/` and `COPY plugins/camera.cpp …/src/` next to existing battery copies. |
| `worlds/turtlebot.model.yaml` | Modify | Add `- type: Camera ...` block under the `plugins:` list. |
| `config/flatland_rviz.rviz` | Modify | Add `rviz_default_plugins/Image` display pre-subscribed to `/image_raw`. |
| `README.md` | Modify | Add "Camera Simulation" section (parameters, topics, verification commands). |

---

### Task 1: Pure math header and standalone host tests (TDD)

**Files:**
- Create: `plugins/camera_math.h`
- Create: `plugins/test/camera_math_test.cpp`

This task is strict TDD on the host. The math has zero dependencies on ROS, Box2D, or Flatland and is fully testable with `g++` alone — no Docker iteration required.

- [ ] **Step 1: Write the failing test**

Create `plugins/test/camera_math_test.cpp`:

```cpp
// Standalone host test for camera_math.h. Run with:
//   g++ -std=c++17 -I plugins plugins/test/camera_math_test.cpp -o /tmp/camera_math_test \
//     && /tmp/camera_math_test
#include "../camera_math.h"
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>

using flatland_plugins::ColumnGeometry;
using flatland_plugins::ColumnSpan;
using flatland_plugins::FocalY;
using flatland_plugins::Rgb;
using flatland_plugins::ShadeColor;
using flatland_plugins::VerticalFov;

#define ASSERT_NEAR(a, b, eps) do {                                            \
  if (std::abs((a) - (b)) > (eps)) {                                           \
    std::cerr << "ASSERT_NEAR failed at " << __FILE__ << ":" << __LINE__       \
              << ": " << (a) << " vs " << (b) << " (eps=" << (eps) << ")\n";   \
    std::exit(1);                                                              \
  }                                                                            \
} while (0)

int main() {
  const int W = 320, H = 240;
  const float fov_h = M_PI / 2.0f;                 // 90 deg
  const float fov_v = VerticalFov(fov_h, W, H);
  const float focal_y = FocalY(H, fov_v);

  // 1. dist=0 fills the full column
  {
    ColumnSpan span = ColumnGeometry(0.0f, focal_y, 1.0f, 0.5f, H);
    assert(span.top == 0);
    assert(span.bottom == H);
    std::cout << "test_column_at_zero_fills: OK\n";
  }

  // 2. dist=range yields a small column
  {
    ColumnSpan span = ColumnGeometry(8.0f, focal_y, 1.0f, 0.5f, H);
    int h = span.bottom - span.top;
    assert(h > 0 && h < H / 4);
    std::cout << "test_column_at_range_small: OK (h=" << h << ")\n";
  }

  // 3. column_h scales as 1/dist (focal_y * wall_height / dist)
  {
    ColumnSpan span = ColumnGeometry(2.0f, focal_y, 1.0f, 0.5f, H);
    int h = span.bottom - span.top;
    int expected = static_cast<int>(focal_y * 1.0f / 2.0f);
    ASSERT_NEAR(h, expected, 2);
    std::cout << "test_column_height_inverse: OK (h=" << h
              << ", expected~" << expected << ")\n";
  }

  // 4. raising eye_height shifts horizon down (larger row index)
  {
    ColumnSpan centered = ColumnGeometry(2.0f, focal_y, 1.0f, 0.5f, H);
    ColumnSpan high_eye = ColumnGeometry(2.0f, focal_y, 1.0f, 0.8f, H);
    int center_horizon = (centered.top + centered.bottom) / 2;
    int high_horizon   = (high_eye.top + high_eye.bottom) / 2;
    assert(high_horizon > center_horizon);
    std::cout << "test_horizon_shifts_with_eye_height: OK\n";
  }

  // 5. shade near is bright
  {
    Rgb fog{30, 30, 30};
    Rgb c = ShadeColor(0.0f, 8.0f, 1.0f, 0.15f, 1.0f, 0.85f, fog);
    assert(c.r > 240 && c.g > 240 && c.b > 240);
    std::cout << "test_shade_near_is_bright: OK (r=" << static_cast<int>(c.r) << ")\n";
  }

  // 6. shade far is dim (fog-dominated)
  {
    Rgb fog{30, 30, 30};
    Rgb c = ShadeColor(8.0f, 8.0f, 1.0f, 0.15f, 1.0f, 0.85f, fog);
    assert(c.r < 60);
    std::cout << "test_shade_far_is_dim: OK (r=" << static_cast<int>(c.r) << ")\n";
  }

  // 7. grazing-angle hits are dimmer than head-on
  {
    Rgb fog{30, 30, 30};
    Rgb head_on = ShadeColor(2.0f, 8.0f, 1.0f, 0.15f, 1.0f, 0.85f, fog);
    Rgb grazing = ShadeColor(2.0f, 8.0f, 0.3f, 0.15f, 1.0f, 0.85f, fog);
    assert(grazing.r < head_on.r);
    std::cout << "test_directional_shading: OK\n";
  }

  // 8. fisheye cosine table is symmetric across columns
  {
    float fov_h_local = M_PI / 2.0f;
    float a_left  = (fov_h_local / 2.0f) - (0          + 0.5f) * (fov_h_local / W);
    float a_right = (fov_h_local / 2.0f) - ((W - 1)    + 0.5f) * (fov_h_local / W);
    ASSERT_NEAR(std::cos(a_left), std::cos(a_right), 1e-5);
    std::cout << "test_fisheye_cosines_symmetric: OK\n";
  }

  std::cout << "All camera_math tests passed.\n";
  return 0;
}
```

- [ ] **Step 2: Run the test to verify it fails (compile error — header missing)**

Run:
```bash
g++ -std=c++17 -I plugins plugins/test/camera_math_test.cpp -o /tmp/camera_math_test
```
Expected: compile error `fatal error: camera_math.h: No such file or directory`.

- [ ] **Step 3: Create `plugins/camera_math.h`**

```cpp
// Pure, header-only math helpers for the Camera plugin.
// No ROS, Box2D, or Flatland dependencies — host-testable.
#ifndef FLATLAND_PLUGINS_CAMERA_MATH_H
#define FLATLAND_PLUGINS_CAMERA_MATH_H

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace flatland_plugins {

struct ColumnSpan {
  int top;     // first row of the wall column (inclusive, 0 = top of image)
  int bottom;  // one past last row (exclusive)
};

struct Rgb {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

// Vertical FOV derived from horizontal FOV and aspect ratio.
inline float VerticalFov(float fov_h, int width, int height) {
  return 2.0f *
         std::atan(std::tan(fov_h / 2.0f) *
                   (static_cast<float>(height) / static_cast<float>(width)));
}

// Vertical focal length in pixels.
inline float FocalY(int height, float fov_v) {
  return (static_cast<float>(height) / 2.0f) / std::tan(fov_v / 2.0f);
}

// Project a wall hit at perpendicular distance `dist` onto the image column.
// `dist` must already have the cos(angle) fisheye correction applied.
// `wall_height` is the virtual wall height (m); `eye_height` is the camera
// eye height (m) measured from the floor.
inline ColumnSpan ColumnGeometry(float dist, float focal_y, float wall_height,
                                  float eye_height, int image_height) {
  if (dist < 1.0e-4f) {
    return {0, image_height};
  }
  float column_h = std::min(focal_y * wall_height / dist,
                            static_cast<float>(image_height));
  float horizon = static_cast<float>(image_height) / 2.0f +
                  (eye_height - wall_height / 2.0f) * focal_y / dist;
  int top = std::max(0, static_cast<int>(horizon - column_h / 2.0f));
  int bottom = std::min(image_height,
                        static_cast<int>(horizon + column_h / 2.0f));
  return {top, bottom};
}

// Shade a wall column. `n_dot` is |dot(ray_dir_world, hit_normal)|: 1=head-on,
// 0=grazing. Pass a negative `n_dot` to disable directional shading.
inline Rgb ShadeColor(float dist, float range, float n_dot, float shade_min,
                     float shade_max, float directional, Rgb fog) {
  float t = std::clamp(dist / range, 0.0f, 1.0f);
  float shade = shade_max + (shade_min - shade_max) * t;
  if (n_dot >= 0.0f && n_dot < 0.5f) {
    shade *= directional;
  }
  uint8_t base = static_cast<uint8_t>(
      std::clamp(255.0f * shade, 0.0f, 255.0f));
  float f = t * t;
  auto blend = [f](uint8_t a, uint8_t b) -> uint8_t {
    return static_cast<uint8_t>(
        static_cast<float>(a) * (1.0f - f) + static_cast<float>(b) * f);
  };
  return Rgb{blend(base, fog.r), blend(base, fog.g), blend(base, fog.b)};
}

}  // namespace flatland_plugins

#endif  // FLATLAND_PLUGINS_CAMERA_MATH_H
```

- [ ] **Step 4: Run the test to verify it passes**

Run:
```bash
g++ -std=c++17 -I plugins plugins/test/camera_math_test.cpp -o /tmp/camera_math_test \
  && /tmp/camera_math_test
```
Expected output (exit code 0):
```
test_column_at_zero_fills: OK
test_column_at_range_small: OK (h=...)
test_column_height_inverse: OK (h=..., expected~...)
test_horizon_shifts_with_eye_height: OK
test_shade_near_is_bright: OK (r=255)
test_shade_far_is_dim: OK (r=...)
test_directional_shading: OK
test_fisheye_cosines_symmetric: OK
All camera_math tests passed.
```

- [ ] **Step 5: Commit**

```bash
git add plugins/camera_math.h plugins/test/camera_math_test.cpp
git commit -m "$(cat <<'EOF'
Add header-only camera math + host tests

Pure column-projection and shading math for the upcoming Camera plugin,
extracted into a header so the math can be unit-tested on the host
without spinning up the Flatland Docker build cycle.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

### Task 2: Plugin skeleton — buildable and visible in logs

**Files:**
- Create: `plugins/camera.h`
- Create: `plugins/camera.cpp` (minimal stub)
- Create: `patches/flatland/0014-flatland_plugins-register-camera-plugin-source-proje.patch`
- Create: `patches/flatland/0015-flatland_plugins-register-camera-plugin-class-projec.patch`
- Modify: `Dockerfile`
- Modify: `worlds/turtlebot.model.yaml`

Goal of this task: the Docker image builds, the simulator starts, and `docker compose up` shows an INFO log line proving the Camera plugin loaded. No publishing, no raycasting yet.

- [ ] **Step 1: Create `plugins/camera.h` with the minimal class shell**

```cpp
#ifndef FLATLAND_PLUGINS_CAMERA_H
#define FLATLAND_PLUGINS_CAMERA_H

#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <rclcpp/rclcpp.hpp>

namespace flatland_plugins {

class Camera : public flatland_server::ModelPlugin {
 public:
  void OnInitialize(const YAML::Node &config) override;
  void BeforePhysicsStep(const flatland_server::Timekeeper &timekeeper) override;

 private:
  UpdateTimer update_timer_;
};

}  // namespace flatland_plugins

#endif  // FLATLAND_PLUGINS_CAMERA_H
```

- [ ] **Step 2: Create `plugins/camera.cpp` with a stub implementation that logs on load**

```cpp
#include <flatland_plugins/camera.h>
#include <flatland_server/yaml_reader.h>
#include <pluginlib/class_list_macros.hpp>

using namespace flatland_server;

namespace flatland_plugins {

void Camera::OnInitialize(const YAML::Node &config) {
  (void)config;
  update_timer_.SetRate(10.0);
  RCLCPP_INFO(rclcpp::get_logger("CameraPlugin"),
              "Camera plugin '%s' loaded (stub: no rendering yet)",
              GetName().c_str());
}

void Camera::BeforePhysicsStep(const Timekeeper &timekeeper) {
  if (!update_timer_.CheckUpdate(timekeeper)) return;
  // Stub: rendering wired up in later tasks.
}

}  // namespace flatland_plugins

PLUGINLIB_EXPORT_CLASS(flatland_plugins::Camera, flatland_server::ModelPlugin)
```

- [ ] **Step 3: Modify `Dockerfile` to copy the new plugin files into the upstream tree**

Find the existing block (around line 52-54):
```dockerfile
# Copy project-owned battery plugin sources (the patches only register them)
COPY plugins/battery.h /ros2_ws/src/flatland/flatland_plugins/include/flatland_plugins/
COPY plugins/battery.cpp /ros2_ws/src/flatland/flatland_plugins/src/
```

Replace it with:
```dockerfile
# Copy project-owned plugin sources (the patches only register them)
COPY plugins/battery.h /ros2_ws/src/flatland/flatland_plugins/include/flatland_plugins/
COPY plugins/battery.cpp /ros2_ws/src/flatland/flatland_plugins/src/
COPY plugins/camera.h /ros2_ws/src/flatland/flatland_plugins/include/flatland_plugins/
COPY plugins/camera.cpp /ros2_ws/src/flatland/flatland_plugins/src/
```

- [ ] **Step 4: Generate the two new patches via the documented patch-stack workflow**

The patches are managed through `git format-patch` against the upstream submodule. The README ("Updating the flatland patches") describes the workflow. Execute it:

```bash
cd third_party/flatland
BASE=$(git rev-parse HEAD)
# If a `work` branch already exists from prior patch work, delete it first:
git branch -D work 2>/dev/null || true
git checkout -b work "$BASE"
git am --whitespace=nowarn ../../patches/flatland/*.patch
```

Now edit `flatland_plugins/CMakeLists.txt` inside the submodule:

1. Add `find_package(OpenCV REQUIRED)` next to the other `find_package` calls (after `find_package(Eigen3 REQUIRED)`).
2. Add `src/camera.cpp` to the `add_library(flatland_plugins_lib SHARED ...)` list (right after the `src/battery.cpp` line that patch 0011 already added).
3. Add `${OpenCV_LIBS}` to the `target_link_libraries(flatland_plugins_lib ...)` block.

Commit:
```bash
git add flatland_plugins/CMakeLists.txt
git commit -m "flatland_plugins: register camera plugin source (project-specific)

Adds src/camera.cpp to the flatland_plugins_lib sources and pulls in
OpenCV for cv::imencode (JPEG compressed image publishing). The actual
camera.{h,cpp} files are provided by the OpenRobOps sim-flatland
project and COPYed into place at image build time."
```

Now edit `flatland_plugins/plugin_description.xml` inside the submodule. Add this block before the closing `</library>` tag:
```xml
  <class type="flatland_plugins::Camera" base_class_type="flatland_server::ModelPlugin">
    <description>Wolfenstein-style raycasted camera plugin</description>
  </class>
```

Commit:
```bash
git add flatland_plugins/plugin_description.xml
git commit -m "flatland_plugins: register camera plugin class (project-specific)

Register flatland_plugins::Camera with pluginlib so the simulation can
spawn it via plugin type at runtime."
```

Regenerate the patch stack:
```bash
rm ../../patches/flatland/*.patch
git format-patch "$BASE"..HEAD -o ../../patches/flatland/
cd ../..
ls patches/flatland/
```

Expected: 15 patch files, including the two new ones starting with `0014-` and `0015-`.

- [ ] **Step 5: Add a Camera plugin block to `worlds/turtlebot.model.yaml`**

Append under the existing `plugins:` list (after the `ModelTfPublisher` block):

```yaml
  - type: Camera
    name: front_camera
    body: base_link
```

(Defaults will fill in everything else once Task 3 wires the YAML parser.)

- [ ] **Step 6: Build the Docker image**

```bash
docker compose build
```
Expected: builds successfully. The two new patches apply cleanly during the patch step. If any patch fails to apply, inspect the build log and fix the patch (likely a line-context mismatch with another patch).

- [ ] **Step 7: Run the simulator and verify the plugin loads**

```bash
docker compose up 2>&1 | grep -i 'CameraPlugin\|camera plugin'
```
Expected: at least one line like
```
[INFO] [...] [CameraPlugin]: Camera plugin 'front_camera' loaded (stub: no rendering yet)
```

Stop the container with `Ctrl+C`.

- [ ] **Step 8: Commit**

```bash
git add plugins/camera.h plugins/camera.cpp Dockerfile worlds/turtlebot.model.yaml \
        patches/flatland/ third_party/flatland
git commit -m "$(cat <<'EOF'
Add Camera plugin skeleton — loads in flatland, no rendering yet

Empty flatland_plugins::Camera ModelPlugin registered through two new
patches (0014 sources + OpenCV link, 0015 pluginlib class registration)
mirroring the battery plugin pattern. Verified the plugin is picked up
by flatland_server on docker compose up.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

### Task 3: YAML parameter parsing

**Files:**
- Modify: `plugins/camera.h`
- Modify: `plugins/camera.cpp`
- Modify: `worlds/turtlebot.model.yaml` (full param block)

Goal: parse every config parameter from the spec, validate them, store them in members. Log the resolved config on init. Still no rendering or publishing.

- [ ] **Step 1: Expand `plugins/camera.h` with all member fields**

Replace the file with:
```cpp
#ifndef FLATLAND_PLUGINS_CAMERA_H
#define FLATLAND_PLUGINS_CAMERA_H

#include <flatland_plugins/camera_math.h>
#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/types.h>
#include <rclcpp/rclcpp.hpp>
#include <cstdint>
#include <string>
#include <unordered_set>
#include <vector>

class b2Body;

namespace flatland_server {
class Body;
}

namespace flatland_plugins {

class Camera : public flatland_server::ModelPlugin {
 public:
  void OnInitialize(const YAML::Node &config) override;
  void BeforePhysicsStep(const flatland_server::Timekeeper &timekeeper) override;

 private:
  void ParseParameters(const YAML::Node &config);

  // Config
  std::string topic_;
  std::string frame_id_;
  flatland_server::Pose origin_;
  double update_rate_;
  int width_;
  int height_;
  double fov_deg_;
  double range_;
  uint16_t layers_bits_;
  bool ignore_self_;
  double wall_height_;
  double eye_height_;
  double shade_min_;
  double shade_max_;
  double directional_shading_;
  Rgb sky_color_;
  Rgb floor_color_;
  Rgb fog_color_;
  bool broadcast_tf_;
  bool publish_camera_info_;
  bool publish_compressed_;
  int jpeg_quality_;

  // Runtime
  flatland_server::Body *body_;
  std::unordered_set<b2Body *> self_b2bodies_;
  UpdateTimer update_timer_;
};

}  // namespace flatland_plugins

#endif  // FLATLAND_PLUGINS_CAMERA_H
```

- [ ] **Step 2: Implement `ParseParameters` in `plugins/camera.cpp`**

Replace the file with:
```cpp
#include <flatland_plugins/camera.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/yaml_reader.h>
#include <boost/algorithm/string/join.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <stdexcept>

using namespace flatland_server;

namespace flatland_plugins {

namespace {

Rgb ParseRgb(YamlReader &reader, const std::string &key, Rgb fallback) {
  if (!reader.Node()[key]) return fallback;
  auto list = reader.GetList<int>(key, 3, 3);
  for (int v : list) {
    if (v < 0 || v > 255) {
      throw YAMLException("Camera '" + key +
                          "' must be three ints in [0, 255]");
    }
  }
  return Rgb{static_cast<uint8_t>(list[0]), static_cast<uint8_t>(list[1]),
             static_cast<uint8_t>(list[2])};
}

}  // namespace

void Camera::ParseParameters(const YAML::Node &config) {
  YamlReader reader(node_, config);
  std::string body_name = reader.Get<std::string>("body");
  topic_                = reader.Get<std::string>("topic", "image_raw");
  frame_id_             = reader.Get<std::string>("frame", "camera_link");
  origin_               = reader.GetPose("origin", Pose(0, 0, 0));
  update_rate_          = reader.Get<double>("update_rate", 10.0);
  width_                = reader.Get<int>("width", 320);
  height_               = reader.Get<int>("height", 240);
  fov_deg_              = reader.Get<double>("fov_deg", 90.0);
  range_                = reader.Get<double>("range", 8.0);
  ignore_self_          = reader.Get<bool>("ignore_self", true);
  wall_height_          = reader.Get<double>("wall_height", 1.0);
  eye_height_           = reader.Get<double>("eye_height", 0.5);
  shade_min_            = reader.Get<double>("shade_min", 0.15);
  shade_max_            = reader.Get<double>("shade_max", 1.0);
  directional_shading_  = reader.Get<double>("directional_shading", 0.85);
  sky_color_            = ParseRgb(reader, "sky_color",   Rgb{120, 140, 160});
  floor_color_          = ParseRgb(reader, "floor_color", Rgb{60,  55,  50});
  fog_color_            = ParseRgb(reader, "fog_color",   Rgb{30,  30,  30});
  broadcast_tf_         = reader.Get<bool>("broadcast_tf", false);
  publish_camera_info_  = reader.Get<bool>("publish_camera_info", true);
  publish_compressed_   = reader.Get<bool>("publish_compressed", true);
  jpeg_quality_         = reader.Get<int>("jpeg_quality", 75);

  std::vector<std::string> layers =
      reader.GetList<std::string>("layers", {"all"}, -1, -1);

  reader.EnsureAccessedAllKeys();

  if (width_ <= 0 || height_ <= 0) {
    throw YAMLException("Camera width and height must be > 0");
  }
  if (update_rate_ <= 0 || range_ <= 0 || wall_height_ <= 0) {
    throw YAMLException(
        "Camera update_rate, range, and wall_height must be > 0");
  }
  if (fov_deg_ <= 0 || fov_deg_ >= 180.0) {
    throw YAMLException("Camera fov_deg must be in (0, 180)");
  }
  if (jpeg_quality_ < 1 || jpeg_quality_ > 100) {
    throw YAMLException("Camera jpeg_quality must be in [1, 100]");
  }

  body_ = GetModel()->GetBody(body_name);
  if (!body_) {
    throw YAMLException("Camera: cannot find body with name " + body_name);
  }

  std::vector<std::string> invalid_layers;
  layers_bits_ = GetModel()->GetCfr()->GetCategoryBits(layers, &invalid_layers);
  if (!invalid_layers.empty()) {
    throw YAMLException("Camera: cannot find layer(s): {" +
                        boost::algorithm::join(invalid_layers, ",") + "}");
  }

  if (ignore_self_) {
    for (auto *b : GetModel()->GetBodies()) {
      self_b2bodies_.insert(b->GetPhysicsBody());
    }
  }
}

void Camera::OnInitialize(const YAML::Node &config) {
  ParseParameters(config);
  update_timer_.SetRate(update_rate_);
  RCLCPP_INFO(
      rclcpp::get_logger("CameraPlugin"),
      "Camera '%s' configured: %dx%d @ %.1f deg fov, range=%.1fm, %.1f Hz, "
      "topic=%s, frame=%s, broadcast_tf=%d, publish_camera_info=%d, "
      "publish_compressed=%d",
      GetName().c_str(), width_, height_, fov_deg_, range_, update_rate_,
      topic_.c_str(), frame_id_.c_str(), broadcast_tf_, publish_camera_info_,
      publish_compressed_);
}

void Camera::BeforePhysicsStep(const Timekeeper &timekeeper) {
  if (!update_timer_.CheckUpdate(timekeeper)) return;
  // Rendering & publishing in Task 4 / 5 / 6.
}

}  // namespace flatland_plugins

PLUGINLIB_EXPORT_CLASS(flatland_plugins::Camera, flatland_server::ModelPlugin)
```

- [ ] **Step 3: Update `worlds/turtlebot.model.yaml` with the full param block**

Replace the minimal block added in Task 2:
```yaml
  - type: Camera
    name: front_camera
    body: base_link
```
with the full configured block:
```yaml
  - type: Camera
    name: front_camera
    body: base_link
    frame: camera_link
    topic: image_raw
    origin: [0.18, 0.0, 0.0]
    update_rate: 10.0
    width: 320
    height: 240
    fov_deg: 90.0
    range: 8.0
    layers: ["all"]
    ignore_self: true
    wall_height: 1.0
    eye_height: 0.5
    shade_min: 0.15
    shade_max: 1.0
    directional_shading: 0.85
    sky_color: [120, 140, 160]
    floor_color: [60, 55, 50]
    fog_color: [30, 30, 30]
    broadcast_tf: false
    publish_camera_info: true
    publish_compressed: true
    jpeg_quality: 75
```

- [ ] **Step 4: Rebuild and verify the INFO log contains the parsed config**

```bash
docker compose build
docker compose up 2>&1 | grep -i 'Camera ' | head -5
```
Expected: at least one line like
```
[INFO] [...] [CameraPlugin]: Camera 'front_camera' configured: 320x240 @ 90.0 deg fov, range=8.0m, 10.0 Hz, topic=image_raw, frame=camera_link, broadcast_tf=0, publish_camera_info=1, publish_compressed=1
```

Stop the container with `Ctrl+C`.

- [ ] **Step 5: Verify a bad config fails fast**

Temporarily set `fov_deg: 200.0` in the YAML, rebuild config files (no docker rebuild needed because worlds/ is bind-mounted), and run:
```bash
docker compose up 2>&1 | grep -i 'fov_deg\|YAML' | head -5
```
Expected: a `YAMLException` with the message `Camera fov_deg must be in (0, 180)`, and the simulator exits.

Revert `fov_deg` to `90.0`.

- [ ] **Step 6: Commit**

```bash
git add plugins/camera.h plugins/camera.cpp worlds/turtlebot.model.yaml
git commit -m "$(cat <<'EOF'
Parse Camera plugin YAML parameters

Wire up all 20+ camera parameters from the spec (optics, raycast filter,
look, publishing flags) with defaults and validation. Bad configs now
abort flatland_server at startup with readable errors.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

### Task 4: Image publishing — solid color frame end-to-end

**Files:**
- Modify: `plugins/camera.h`
- Modify: `plugins/camera.cpp`

Goal: a `sensor_msgs/Image` of dimensions `width_ × height_` filled with a solid color is published on `image_raw` at `update_rate_` Hz. Proves the entire publish path before we tackle raycasting.

- [ ] **Step 1: Add publisher + image buffer to `plugins/camera.h`**

Add includes near the top:
```cpp
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/image.hpp>
```

Add to the private section (after the existing runtime members):
```cpp
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  cv::Mat frame_;
  sensor_msgs::msg::Image image_msg_;
```

- [ ] **Step 2: Initialize publisher + buffer in `OnInitialize`**

In `plugins/camera.cpp`, at the end of `OnInitialize` (before the closing brace), insert:
```cpp
  image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(topic_, 1);

  frame_ = cv::Mat(height_, width_, CV_8UC3, cv::Scalar(0, 0, 0));

  image_msg_.height = height_;
  image_msg_.width = width_;
  image_msg_.encoding = "rgb8";
  image_msg_.is_bigendian = 0;
  image_msg_.step = width_ * 3;
  image_msg_.header.frame_id = GetModel()->NameSpaceTF(frame_id_);
```

- [ ] **Step 3: Fill + publish a solid-color frame each tick**

Replace the body of `BeforePhysicsStep`:
```cpp
void Camera::BeforePhysicsStep(const Timekeeper &timekeeper) {
  if (!update_timer_.CheckUpdate(timekeeper)) return;
  if (image_pub_->get_subscription_count() == 0) return;

  // Solid floor_color fill — sanity check, real rendering in Task 5.
  frame_.setTo(cv::Scalar(floor_color_.r, floor_color_.g, floor_color_.b));

  image_msg_.data.assign(
      frame_.data, frame_.data + (image_msg_.step * image_msg_.height));
  image_msg_.header.stamp = timekeeper.GetSimTime();
  image_pub_->publish(image_msg_);
}
```

- [ ] **Step 4: Rebuild and verify the topic is publishing**

```bash
docker compose build
docker compose up -d
sleep 8  # give nav2 time to come up
docker compose exec flatland-nav2 ros2 topic list | grep image_raw
docker compose exec flatland-nav2 ros2 topic hz /image_raw
```
Expected: `/image_raw` appears in the topic list, and `ros2 topic hz` reports ~10 Hz once a subscriber attaches (`ros2 topic hz` itself counts).

Verify dimensions and encoding:
```bash
docker compose exec flatland-nav2 ros2 topic echo /image_raw --field width --once
docker compose exec flatland-nav2 ros2 topic echo /image_raw --field height --once
docker compose exec flatland-nav2 ros2 topic echo /image_raw --field encoding --once
```
Expected output: `320`, `240`, `rgb8`.

Stop:
```bash
docker compose down
```

- [ ] **Step 5: Commit**

```bash
git add plugins/camera.h plugins/camera.cpp
git commit -m "$(cat <<'EOF'
Publish a stub Image from the Camera plugin

Allocate the reusable cv::Mat and sensor_msgs::Image, publish a solid
floor-colored frame at update_rate Hz when subscribed. Proves the
publish path end-to-end so the next task can focus on raycasting and
rendering.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

### Task 5: Raycast rendering — real Wolfenstein-style image

**Files:**
- Modify: `plugins/camera.h`
- Modify: `plugins/camera.cpp`

Goal: the published image actually reflects the world — walls grow as the robot approaches, fade with distance, shift horizontally when rotating. This is the core algorithm.

- [ ] **Step 1: Add ray-table fields and ThreadPool to `plugins/camera.h`**

Add includes:
```cpp
#include <Eigen/Dense>
#include <thirdparty/ThreadPool.h>
#include <Box2D/Box2D.h>
#include <vector>
```

Add to the public section (before `OnInitialize`):
```cpp
  Camera() : pool_(std::thread::hardware_concurrency() + 1) {}
```

Add `#include <thread>` to the header includes.

Add to the private section:
```cpp
  // Precomputed per-column ray data (camera frame).
  std::vector<float> ray_dir_cam_x_;
  std::vector<float> ray_dir_cam_y_;
  std::vector<float> cos_correction_;
  float focal_y_;

  // Cached transform body→camera (3x3 in homogeneous coords, matching laser.cpp).
  Eigen::Matrix3f m_body_to_camera_;

  ThreadPool pool_;
```

- [ ] **Step 2: Build the ray table in `OnInitialize`**

In `plugins/camera.cpp`, add inside `OnInitialize` immediately after `ParseParameters(config)` and before the publisher creation block from Task 4:

```cpp
  // Cache body→camera transform.
  {
    float c = std::cos(origin_.theta);
    float s = std::sin(origin_.theta);
    m_body_to_camera_ << c, -s, static_cast<float>(origin_.x),
                         s,  c, static_cast<float>(origin_.y),
                         0,  0, 1;
  }

  // Precompute per-column ray directions in the camera frame.
  float fov_h = static_cast<float>(fov_deg_) * static_cast<float>(M_PI) / 180.0f;
  ray_dir_cam_x_.resize(width_);
  ray_dir_cam_y_.resize(width_);
  cos_correction_.resize(width_);
  for (int col = 0; col < width_; ++col) {
    float a = (fov_h / 2.0f) -
              (static_cast<float>(col) + 0.5f) * (fov_h / static_cast<float>(width_));
    ray_dir_cam_x_[col] = std::cos(a);
    ray_dir_cam_y_[col] = std::sin(a);
    cos_correction_[col] = std::cos(a);
  }

  float fov_v = VerticalFov(fov_h, width_, height_);
  focal_y_ = FocalY(height_, fov_v);
```

- [ ] **Step 3: Define the raycast callback**

Inside `plugins/camera.cpp`, before the `Camera::ParseParameters` function, add:
```cpp
struct CameraRayCb : public b2RayCastCallback {
  Camera *parent;
  bool did_hit;
  float fraction;
  b2Vec2 normal;
  uint16_t hit_category;

  explicit CameraRayCb(Camera *p)
      : parent(p), did_hit(false), fraction(1.0f), normal(), hit_category(0) {}

  float ReportFixture(b2Fixture *fix, const b2Vec2 & /*point*/,
                      const b2Vec2 &n, float f) override {
    uint16_t cat = fix->GetFilterData().categoryBits;
    if (!(cat & parent->layers_bits_)) return -1.0f;
    if (fix->IsSensor()) return -1.0f;
    if (parent->ignore_self_ &&
        parent->self_b2bodies_.count(fix->GetBody()) > 0) {
      return -1.0f;
    }
    did_hit = true;
    fraction = f;
    normal = n;
    hit_category = cat;
    return f;
  }
};
```

To let the callback access private members, make it a friend by adding inside the `Camera` class body in `plugins/camera.h` (private section is fine for a friend declaration):
```cpp
  friend struct CameraRayCb;
```

- [ ] **Step 4: Implement raycast + rendering in `BeforePhysicsStep`**

Replace the body of `BeforePhysicsStep` again:
```cpp
void Camera::BeforePhysicsStep(const Timekeeper &timekeeper) {
  if (!update_timer_.CheckUpdate(timekeeper)) return;
  if (image_pub_->get_subscription_count() == 0) return;

  // World→camera transform.
  const b2Transform &t = body_->GetPhysicsBody()->GetTransform();
  Eigen::Matrix3f m_world_to_body;
  m_world_to_body << t.q.c, -t.q.s, t.p.x,
                     t.q.s,  t.q.c, t.p.y,
                     0,      0,     1;
  Eigen::Matrix3f m_world_to_camera = m_world_to_body * m_body_to_camera_;

  Eigen::Vector3f cam_origin_h = m_world_to_camera * Eigen::Vector3f(0, 0, 1);
  b2Vec2 cam_origin(cam_origin_h(0), cam_origin_h(1));

  float c = m_world_to_camera(0, 0);
  float s = m_world_to_camera(1, 0);

  // Per-column raycasts via the thread pool (same pattern as laser.cpp).
  struct Hit {
    bool did_hit;
    float dist_raw;
    b2Vec2 dir_world;
    b2Vec2 normal;
  };
  std::vector<std::future<Hit>> results(width_);
  for (int col = 0; col < width_; ++col) {
    float dx_c = ray_dir_cam_x_[col];
    float dy_c = ray_dir_cam_y_[col];
    float dx_w = c * dx_c - s * dy_c;
    float dy_w = s * dx_c + c * dy_c;
    b2Vec2 end(cam_origin.x + dx_w * static_cast<float>(range_),
               cam_origin.y + dy_w * static_cast<float>(range_));
    results[col] = pool_.enqueue([this, cam_origin, end, dx_w, dy_w]() {
      CameraRayCb cb(this);
      GetModel()->GetPhysicsWorld()->RayCast(&cb, cam_origin, end);
      Hit h;
      h.did_hit = cb.did_hit;
      h.dist_raw = cb.did_hit ? cb.fraction * static_cast<float>(range_)
                              : static_cast<float>(range_);
      h.dir_world = b2Vec2(dx_w, dy_w);
      h.normal = cb.normal;
      return h;
    });
  }

  // Fill image column by column.
  cv::Vec3b sky(sky_color_.r, sky_color_.g, sky_color_.b);
  cv::Vec3b floor(floor_color_.r, floor_color_.g, floor_color_.b);

  for (int col = 0; col < width_; ++col) {
    Hit h = results[col].get();
    float dist = h.dist_raw * cos_correction_[col];

    ColumnSpan span = h.did_hit
        ? ColumnGeometry(dist, focal_y_,
                         static_cast<float>(wall_height_),
                         static_cast<float>(eye_height_), height_)
        : ColumnSpan{height_ / 2, height_ / 2};

    cv::Vec3b wall;
    if (h.did_hit) {
      float n_dot = std::abs(h.dir_world.x * h.normal.x +
                             h.dir_world.y * h.normal.y);
      // Degenerate normal: treat as head-on.
      if (h.normal.LengthSquared() < 1e-6f) n_dot = 1.0f;
      Rgb shaded = ShadeColor(
          dist, static_cast<float>(range_), n_dot,
          static_cast<float>(shade_min_), static_cast<float>(shade_max_),
          static_cast<float>(directional_shading_), fog_color_);
      wall = cv::Vec3b(shaded.r, shaded.g, shaded.b);
    }

    for (int row = 0; row < span.top; ++row) {
      frame_.at<cv::Vec3b>(row, col) = sky;
    }
    if (h.did_hit) {
      for (int row = span.top; row < span.bottom; ++row) {
        frame_.at<cv::Vec3b>(row, col) = wall;
      }
    }
    for (int row = std::max(span.bottom, span.top); row < height_; ++row) {
      frame_.at<cv::Vec3b>(row, col) = floor;
    }
  }

  image_msg_.data.assign(
      frame_.data, frame_.data + (image_msg_.step * image_msg_.height));
  image_msg_.header.stamp = timekeeper.GetSimTime();
  image_pub_->publish(image_msg_);
}
```

- [ ] **Step 5: Rebuild and verify a rendered image**

```bash
docker compose build
docker compose up -d
sleep 10
docker compose exec flatland-nav2 ros2 topic hz /image_raw
```
Expected: ~10 Hz with subscribers attached.

Open a quick visual check (the rviz config doesn't have the Image display yet — we add it in Task 7 — but `rqt_image_view` works):
```bash
docker compose exec flatland-nav2 bash -lc 'source /opt/ros/jazzy/setup.bash && \
  source /ros2_ws/install/setup.bash && ros2 run rqt_image_view rqt_image_view /image_raw'
```
Expected: a window showing a horizon-split image with gray-shaded walls visible (the spawn pose `[5, 15, 0]` faces toward the office wall). Rotate the robot via rviz `2D Goal Pose` or `cmd_vel` and confirm walls slide horizontally; drive forward and confirm walls grow.

Stop:
```bash
docker compose down
```

- [ ] **Step 6: Commit**

```bash
git add plugins/camera.h plugins/camera.cpp
git commit -m "$(cat <<'EOF'
Render Wolfenstein-style frames from Box2D raycasts

Cast one ray per image column through Flatland's Box2D world using the
same thread pool pattern as the Laser plugin, project each hit into a
wall column with cos(angle) fisheye correction, and depth-shade the
column with quadratic fog blending. Walls now appear in the published
/image_raw stream as the robot moves through the world.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

### Task 6: CameraInfo, CompressedImage, and optional TF

**Files:**
- Modify: `plugins/camera.h`
- Modify: `plugins/camera.cpp`

Goal: the remaining two ROS topics plus opt-in TF broadcast.

- [ ] **Step 1: Add headers, publishers, TF broadcaster to `plugins/camera.h`**

Add includes:
```cpp
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <tf2_ros/transform_broadcaster.h>
```

Add to private members:
```cpp
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped camera_tf_;
```

- [ ] **Step 2: Wire publishers + static CameraInfo + TF transform in `OnInitialize`**

In `plugins/camera.cpp`, at the end of `OnInitialize`, append:
```cpp
  if (publish_camera_info_) {
    camera_info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
        topic_ + "/camera_info", 1);
    camera_info_msg_.header.frame_id = image_msg_.header.frame_id;
    camera_info_msg_.height = height_;
    camera_info_msg_.width  = width_;
    camera_info_msg_.distortion_model = "plumb_bob";
    camera_info_msg_.d.assign(5, 0.0);
    camera_info_msg_.k = {focal_y_, 0.0f, width_ / 2.0f,
                          0.0f, focal_y_, height_ / 2.0f,
                          0.0f, 0.0f, 1.0f};
    camera_info_msg_.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    camera_info_msg_.p = {focal_y_, 0.0f, width_ / 2.0f, 0.0f,
                          0.0f, focal_y_, height_ / 2.0f, 0.0f,
                          0.0f, 0.0f, 1.0f, 0.0f};
  }

  if (publish_compressed_) {
    compressed_pub_ = node_->create_publisher<sensor_msgs::msg::CompressedImage>(
        topic_ + "/compressed", 1);
  }

  if (broadcast_tf_) {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    tf2::Quaternion q;
    q.setRPY(0, 0, origin_.theta);
    camera_tf_.header.frame_id = GetModel()->NameSpaceTF(body_->GetName());
    camera_tf_.child_frame_id  = image_msg_.header.frame_id;
    camera_tf_.transform.translation.x = origin_.x;
    camera_tf_.transform.translation.y = origin_.y;
    camera_tf_.transform.translation.z = 0;
    camera_tf_.transform.rotation.x = q.x();
    camera_tf_.transform.rotation.y = q.y();
    camera_tf_.transform.rotation.z = q.z();
    camera_tf_.transform.rotation.w = q.w();
  }
```

Add the required include in `plugins/camera.cpp` at the top:
```cpp
#include <opencv2/imgcodecs.hpp>
#include <tf2/LinearMath/Quaternion.h>
```

- [ ] **Step 3: Extend the subscriber-gate and per-frame publishing**

Replace the subscriber-gate line in `BeforePhysicsStep`:
```cpp
  if (image_pub_->get_subscription_count() == 0) return;
```
with:
```cpp
  size_t subs = image_pub_->get_subscription_count();
  if (publish_camera_info_) subs += camera_info_pub_->get_subscription_count();
  if (publish_compressed_)  subs += compressed_pub_->get_subscription_count();
  if (subs == 0) {
    if (broadcast_tf_) {
      camera_tf_.header.stamp = timekeeper.GetSimTime();
      tf_broadcaster_->sendTransform(camera_tf_);
    }
    return;
  }
```

At the very end of `BeforePhysicsStep` (after `image_pub_->publish(image_msg_)`), append:
```cpp
  if (publish_camera_info_) {
    camera_info_msg_.header.stamp = image_msg_.header.stamp;
    camera_info_pub_->publish(camera_info_msg_);
  }

  if (publish_compressed_) {
    sensor_msgs::msg::CompressedImage cmsg;
    cmsg.header = image_msg_.header;
    cmsg.format = "jpeg";
    // OpenCV expects BGR for JPEG; the in-memory frame is RGB, so convert.
    cv::Mat bgr;
    cv::cvtColor(frame_, bgr, cv::COLOR_RGB2BGR);
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
    if (!cv::imencode(".jpg", bgr, cmsg.data, params)) {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("CameraPlugin"),
                           *node_->get_clock(), 1000,
                           "cv::imencode(.jpg) failed; skipping compressed frame");
    } else {
      compressed_pub_->publish(cmsg);
    }
  }

  if (broadcast_tf_) {
    camera_tf_.header.stamp = image_msg_.header.stamp;
    tf_broadcaster_->sendTransform(camera_tf_);
  }
```

Add the include for cvtColor:
```cpp
#include <opencv2/imgproc.hpp>
```

- [ ] **Step 4: Rebuild and verify the extra topics**

```bash
docker compose build
docker compose up -d
sleep 10
docker compose exec flatland-nav2 ros2 topic list | grep image_raw
docker compose exec flatland-nav2 ros2 topic hz /image_raw/compressed
docker compose exec flatland-nav2 ros2 topic echo /image_raw/camera_info --once
```
Expected: `/image_raw`, `/image_raw/compressed`, `/image_raw/camera_info` all present; `hz` reports ~10 Hz; `CameraInfo` shows `width: 320`, `height: 240`, and `k` populated with the focal length.

Test the optional TF: change `broadcast_tf: true` in `worlds/turtlebot.model.yaml` (bind-mounted, no rebuild). Restart and check:
```bash
docker compose down
docker compose up -d
sleep 10
docker compose exec flatland-nav2 ros2 run tf2_ros tf2_echo base_link camera_link
```
Expected: a transform with translation `(0.18, 0.0, 0.0)`.

Revert `broadcast_tf` to `false` (the user's chosen default).

- [ ] **Step 5: Stop and commit**

```bash
docker compose down
git add plugins/camera.h plugins/camera.cpp
git commit -m "$(cat <<'EOF'
Add CameraInfo, CompressedImage, and optional TF to Camera plugin

Publishes /image_raw/camera_info with a synthetic plumb_bob intrinsics
matrix built once at init; /image_raw/compressed with JPEG encoding via
OpenCV; and (when broadcast_tf is true) a base_link→camera_link static
transform on /tf.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

### Task 7: rviz Image display and README documentation

**Files:**
- Modify: `config/flatland_rviz.rviz`
- Modify: `README.md`

Goal: the camera image is visible in rviz immediately on `docker compose up`, and the README explains how to configure and verify the camera.

- [ ] **Step 1: Inspect the existing rviz config to find the right insertion point**

```bash
grep -n 'Class: rviz_default_plugins' config/flatland_rviz.rviz | head -10
```
Note the position of the LaserScan display — the Image entry goes immediately after it (per the spec's panel order: Map → Robot → Laser → Camera Image → Battery → costmaps).

- [ ] **Step 2: Add the Image display block**

Open `config/flatland_rviz.rviz`. Locate the `Displays:` list (top-level key under `Visualization Manager:`). After the existing `rviz_default_plugins/LaserScan` block (which ends at its trailing `Value: true`), insert:

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

Use the same indentation as the surrounding entries (typically 4 spaces under `Displays:`).

- [ ] **Step 3: Restart the stack and verify the panel appears**

```bash
xhost +local:docker
docker compose up
```
Expected: rviz opens. In the Displays panel, "Camera Image" is listed and checked. A separate floating Image panel shows the rendered camera feed (or the dock appears at the bottom — depending on rviz version, the Image display may render in its own dockable subwindow).

Stop with `Ctrl+C`.

- [ ] **Step 4: Add a Camera section to the README**

Open `README.md`. Find the "Battery Simulation" section. Immediately after it (before the next top-level section), insert:

````markdown
## Camera Simulation

The robot includes a synthetic forward-facing camera that renders a Wolfenstein
3D-style image of the 2D world using one Box2D raycast per image column.
The image is depth-shaded, with a solid sky and floor split at the horizon.

The image is visible in rviz by default (the "Camera Image" display, pre-subscribed
to `/image_raw`) and can also be viewed with `rqt_image_view /image_raw`.

### Configuration

Parameters in `worlds/turtlebot.model.yaml` (all optional — defaults shown):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `width` | 320 | Image width in pixels |
| `height` | 240 | Image height in pixels |
| `fov_deg` | 90.0 | Horizontal field of view (degrees) |
| `range` | 8.0 | Max ray distance in meters |
| `update_rate` | 10.0 | Publish rate in Hz |
| `origin` | `[0, 0, 0]` | Camera mount offset `[x, y, theta]` relative to body |
| `layers` | `["all"]` | Box2D collision layers the camera sees |
| `ignore_self` | true | Skip the camera's own model when raycasting |
| `wall_height` | 1.0 | Virtual wall height (m), affects column projection |
| `eye_height` | 0.5 | Camera eye height (m); shifts horizon |
| `shade_min` / `shade_max` | 0.15 / 1.0 | Brightness at far / near distance |
| `directional_shading` | 0.85 | Multiplier for grazing-angle hits (1.0 disables) |
| `sky_color`, `floor_color`, `fog_color` | gray-ish defaults | RGB `[0-255]` triples |
| `broadcast_tf` | false | Publish `base_link → camera_link` transform |
| `publish_camera_info` | true | Publish synthetic `sensor_msgs/CameraInfo` |
| `publish_compressed` | true | Publish JPEG-compressed image |
| `jpeg_quality` | 75 | JPEG quality (1-100) when compressed publishing is on |

### Topics

| Topic | Type | Notes |
|-------|------|-------|
| `/image_raw` | `sensor_msgs/Image` | `rgb8` encoding |
| `/image_raw/camera_info` | `sensor_msgs/CameraInfo` | Static intrinsics (plumb_bob, zero distortion) |
| `/image_raw/compressed` | `sensor_msgs/CompressedImage` | JPEG, on if `publish_compressed: true` |

### Verifying the camera

```bash
# Topics exist and publish at the configured rate
docker compose exec flatland-nav2 ros2 topic hz /image_raw            # ~10 Hz
docker compose exec flatland-nav2 ros2 topic hz /image_raw/compressed
docker compose exec flatland-nav2 ros2 topic echo /image_raw/camera_info --once

# Image fields match config
docker compose exec flatland-nav2 ros2 topic echo /image_raw --field width    --once  # 320
docker compose exec flatland-nav2 ros2 topic echo /image_raw --field height   --once  # 240
docker compose exec flatland-nav2 ros2 topic echo /image_raw --field encoding --once  # rgb8

# Drive the robot toward a wall and watch walls grow / brighten in rviz
```
````

- [ ] **Step 5: Final integration check**

```bash
docker compose up
```
Expected: rviz opens with the Camera Image panel showing the rendered scene. Drive the robot via `2D Goal Pose` — the wall scrolls in the image as the robot rotates and approaches walls.

Stop with `Ctrl+C`.

- [ ] **Step 6: Commit**

```bash
git add config/flatland_rviz.rviz README.md
git commit -m "$(cat <<'EOF'
Show Camera image in rviz by default + document camera plugin

Adds an rviz_default_plugins/Image display to the default layout
(pre-subscribed to /image_raw) and a new Camera Simulation section to
the README covering parameters, topics, and verification commands.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Notes for the implementer

- **Patches are stateful.** If your local `third_party/flatland` already has a `work` branch from prior patch edits, delete it (`git branch -D work`) before re-running the `git checkout -b work "$BASE"` step in Task 2.
- **Docker rebuilds are slow.** Bind-mounted directories (`worlds/`, `config/`, `local/`) don't require a rebuild — only source/Dockerfile/patches changes do. Task 3's step 5 (bad-config test) uses a bind-mounted YAML edit and avoids a rebuild.
- **Threading model.** The `ThreadPool` pattern is borrowed from `laser.cpp`. `b2World::RayCast` is safe to call in parallel against the same world *within one cast pass* because that's exactly what the laser plugin already does.
- **CV color order.** OpenCV's `imencode` and `cvtColor` use BGR by convention. We hold the frame as RGB (because `sensor_msgs/Image` encoding is `rgb8`) and convert only when feeding `imencode`. Don't conflate the two color orders.
- **No CI test wiring.** The `camera_math_test.cpp` is host-runnable only; it isn't added to `colcon test` because the project doesn't run `colcon test` in CI today. Run it locally as the patch in Task 1 step 4 shows.
