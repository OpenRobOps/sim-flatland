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
