#include <flatland_plugins/camera.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/yaml_reader.h>
#include <boost/algorithm/string/join.hpp>
#include <pluginlib/class_list_macros.hpp>

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

  image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(topic_, 1);

  frame_ = cv::Mat(height_, width_, CV_8UC3, cv::Scalar(0, 0, 0));

  image_msg_.height = height_;
  image_msg_.width = width_;
  image_msg_.encoding = "rgb8";
  image_msg_.is_bigendian = 0;
  image_msg_.step = width_ * 3;
  image_msg_.header.frame_id = GetModel()->NameSpaceTF(frame_id_);
}

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

}  // namespace flatland_plugins

PLUGINLIB_EXPORT_CLASS(flatland_plugins::Camera, flatland_server::ModelPlugin)
