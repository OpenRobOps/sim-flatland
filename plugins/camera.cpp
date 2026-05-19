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
