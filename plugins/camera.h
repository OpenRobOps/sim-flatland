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
