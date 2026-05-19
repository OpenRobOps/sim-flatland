#ifndef FLATLAND_PLUGINS_CAMERA_H
#define FLATLAND_PLUGINS_CAMERA_H

#include <flatland_plugins/camera_math.h>
#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/types.h>
#include <opencv2/core.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>
#include <thirdparty/ThreadPool.h>
#include <Box2D/Box2D.h>
#include <cstdint>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

namespace flatland_plugins {

class Camera : public flatland_server::ModelPlugin {
 public:
  Camera() : pool_(std::thread::hardware_concurrency() + 1) {}

  void OnInitialize(const YAML::Node &config) override;
  void BeforePhysicsStep(const flatland_server::Timekeeper &timekeeper) override;

 private:
  friend struct CameraRayCb;

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
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
  cv::Mat frame_;
  sensor_msgs::msg::Image image_msg_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped camera_tf_;

  // Precomputed per-column ray data (camera frame).
  std::vector<float> ray_dir_cam_x_;
  std::vector<float> ray_dir_cam_y_;
  std::vector<float> cos_correction_;
  float focal_y_;

  // Cached transform body→camera (3x3 in homogeneous coords, matching laser.cpp).
  Eigen::Matrix3f m_body_to_camera_;

  ThreadPool pool_;
};

}  // namespace flatland_plugins

#endif  // FLATLAND_PLUGINS_CAMERA_H
