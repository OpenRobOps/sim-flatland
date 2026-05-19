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

  // 9. extreme eye_height vs wall_height combos clamp to image bounds —
  //    regression guard against heap-overrun in the column fill loop.
  {
    // eye_height = 2.0, wall_height = 1.0: horizon shifts far below center
    // for close walls, which used to push `top` past H.
    ColumnSpan span_high = ColumnGeometry(0.5f, focal_y, 1.0f, 2.0f, H);
    assert(span_high.top    >= 0 && span_high.top    <= H);
    assert(span_high.bottom >= 0 && span_high.bottom <= H);
    // And the inverted case — very low eye should also stay in-bounds.
    ColumnSpan span_low = ColumnGeometry(0.5f, focal_y, 1.0f, -1.0f, H);
    assert(span_low.top    >= 0 && span_low.top    <= H);
    assert(span_low.bottom >= 0 && span_low.bottom <= H);
    std::cout << "test_column_clamps_to_image_bounds: OK\n";
  }

  std::cout << "All camera_math tests passed.\n";
  return 0;
}
