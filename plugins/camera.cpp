#include <flatland_plugins/camera.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/yaml_reader.h>
#include <boost/algorithm/string/join.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Quaternion.h>

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
  image_msg_.step = static_cast<uint32_t>(frame_.step);
  image_msg_.header.frame_id = GetModel()->NameSpaceTF(frame_id_);

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
}

void Camera::BeforePhysicsStep(const Timekeeper &timekeeper) {
  if (!update_timer_.CheckUpdate(timekeeper)) return;
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
      frame_.data,
      frame_.data + frame_.step * static_cast<size_t>(image_msg_.height));
  image_msg_.header.stamp = timekeeper.GetSimTime();
  image_pub_->publish(image_msg_);

  if (publish_camera_info_) {
    camera_info_msg_.header.stamp = image_msg_.header.stamp;
    camera_info_pub_->publish(camera_info_msg_);
  }

  if (publish_compressed_) {
    sensor_msgs::msg::CompressedImage cmsg;
    cmsg.header = image_msg_.header;
    // image_transport compressed-subscriber convention: the prefix is the
    // source image encoding (matches image_msg_.encoding), so consumers
    // reconstruct the right channel order after JPEG decode.
    cmsg.format = "rgb8; jpeg compressed";
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
}

}  // namespace flatland_plugins

PLUGINLIB_EXPORT_CLASS(flatland_plugins::Camera, flatland_server::ModelPlugin)
