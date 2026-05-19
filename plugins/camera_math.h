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
  // Clamp both ends against both bounds. Without this, large eye_height vs
  // wall_height ratios can push `top` past image_height (or `bottom` below 0),
  // and the rendering loop in camera.cpp would write outside the cv::Mat.
  int top = std::clamp(static_cast<int>(horizon - column_h / 2.0f),
                       0, image_height);
  int bottom = std::clamp(static_cast<int>(horizon + column_h / 2.0f),
                          0, image_height);
  return {top, bottom};
}

// Shade a wall column. `n_dot` is |dot(ray_dir_world, hit_normal)|: 1=head-on,
// 0=grazing. Only grazing-angle hits (`n_dot < 0.5`) are dimmed by `directional`.
// Pass a negative `n_dot` to skip the grazing check entirely (e.g. degenerate
// normals where the dot product is meaningless).
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
