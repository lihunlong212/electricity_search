#include "activity_control_pkg/route_target_publisher.hpp"

#include <angles/angles.h>

#include <chrono>
#include <clocale>
#include <cmath>
#include <ctime>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace activity_control_pkg
{

namespace
{
constexpr double kDefaultTimerPeriodSec = 0.05;
}  // namespace

RouteTargetPublisherNode::RouteTargetPublisherNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("route_target_publisher", options),
  current_idx_(std::numeric_limits<std::size_t>::max()),
  has_height_(false),
  current_height_cm_(0.0),
  photo_target_height_cm_(0.0),
  photo_dwell_time_sec_(0.0),
  photo_capture_timeout_sec_(0.0),
  mission_complete_sent_(false),
  mission_phase_(MissionPhase::kIdle),
  photo_capture_result_ready_(false),
  photo_capture_result_success_(false)
{
  pos_tol_cm_ = declare_parameter("position_tolerance_cm", 9.0);
  yaw_tol_deg_ = declare_parameter("yaw_tolerance_deg", 5.0);
  height_tol_cm_ = declare_parameter("height_tolerance_cm", 12.0);
  map_frame_ = declare_parameter("map_frame", "map");
  laser_link_frame_ = declare_parameter("laser_link_frame", "laser_link");
  output_topic_ = declare_parameter("output_topic", "/target_position");
  photo_target_height_cm_ = declare_parameter("photo_target_height_cm", 50.0);
  photo_dwell_time_sec_ = declare_parameter("photo_dwell_time_sec", 2.0);
  photo_capture_timeout_sec_ = declare_parameter("photo_capture_timeout_sec", 3.0);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto durable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  target_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(output_topic_, durable_qos);
  active_controller_pub_ = create_publisher<std_msgs::msg::UInt8>("/active_controller", durable_qos);
  visual_takeover_active_pub_ =
    create_publisher<std_msgs::msg::Bool>("/visual_takeover_active", durable_qos);
  photo_capture_request_pub_ =
    create_publisher<std_msgs::msg::String>("/photo_capture_request", rclcpp::QoS(10).reliable());
  mission_complete_pub_ =
    create_publisher<std_msgs::msg::Empty>("/mission_complete", rclcpp::QoS(10).reliable());

  height_sub_ = create_subscription<std_msgs::msg::Int16>(
    "/height",
    rclcpp::QoS(10),
    std::bind(&RouteTargetPublisherNode::heightCallback, this, std::placeholders::_1));
  photo_capture_result_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/photo_capture_result",
    rclcpp::QoS(10),
    std::bind(&RouteTargetPublisherNode::photoCaptureResultCallback, this, std::placeholders::_1));

  monitor_timer_ = create_wall_timer(
    std::chrono::duration<double>(kDefaultTimerPeriodSec),
    std::bind(&RouteTargetPublisherNode::monitorTimerCallback, this));

  publishVisualTakeoverState(false);

  RCLCPP_INFO(
    get_logger(),
    "RouteTargetPublisher initialized: map=%s laser_link=%s topic=%s",
    map_frame_.c_str(),
    laser_link_frame_.c_str(),
    output_topic_.c_str());
  RCLCPP_INFO(
    get_logger(),
    "Tolerances: position=%.1fcm yaw=%.1fdeg height=%.1fcm",
    pos_tol_cm_,
    yaw_tol_deg_,
    height_tol_cm_);
  RCLCPP_INFO(
    get_logger(),
    "Photo capture: target_height=%.1fcm dwell=%.1fs timeout=%.1fs",
    photo_target_height_cm_,
    photo_dwell_time_sec_,
    photo_capture_timeout_sec_);
}

void RouteTargetPublisherNode::addTarget(const Target & target)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const bool was_empty = targets_.empty();
  const bool was_completed =
    current_idx_ != std::numeric_limits<std::size_t>::max() && current_idx_ >= targets_.size();
  targets_.push_back(target);
  if (was_empty || was_completed) {
    mission_complete_sent_ = false;
    current_idx_ = was_completed ? targets_.size() - 1 : 0;
    publishCurrent();
  }
}

std::size_t RouteTargetPublisherNode::currentIndex() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_idx_;
}

std::size_t RouteTargetPublisherNode::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return targets_.size();
}

void RouteTargetPublisherNode::publishCurrent()
{
  if (current_idx_ != std::numeric_limits<std::size_t>::max() && current_idx_ < targets_.size()) {
    mission_phase_ = MissionPhase::kNormalNavigation;
    photo_capture_result_ready_ = false;
    pending_photo_filename_.clear();
    publishVisualTakeoverState(false);
    publishTarget(getPublishedTarget(targets_[current_idx_]), current_idx_ == 0);
  } else {
    mission_phase_ = MissionPhase::kIdle;
  }
}

void RouteTargetPublisherNode::publishTarget(const Target & target, bool init_flag)
{
  std_msgs::msg::Float32MultiArray message;
  message.data.resize(4);
  message.data[0] = static_cast<float>(target.x_cm);
  message.data[1] = static_cast<float>(target.y_cm);
  message.data[2] = static_cast<float>(target.z_cm);
  message.data[3] = static_cast<float>(target.yaw_deg);
  target_pub_->publish(message);

  std_msgs::msg::UInt8 active_msg;
  active_msg.data = 2;
  active_controller_pub_->publish(active_msg);

  RCLCPP_INFO(
    get_logger(),
    "Published target: x=%.1fcm y=%.1fcm z=%.1fcm yaw=%.1fdeg photo=%s%s",
    target.x_cm,
    target.y_cm,
    target.z_cm,
    target.yaw_deg,
    target.is_takeover ? "true" : "false",
    init_flag ? " (first)" : "");
}

Target RouteTargetPublisherNode::getPublishedTarget(const Target & target) const
{
  Target published_target = target;
  if (isPhotoCapturePhase() && target.is_takeover) {
    published_target.z_cm = photo_target_height_cm_;
  }
  return published_target;
}

void RouteTargetPublisherNode::heightCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  current_height_cm_ = static_cast<double>(msg->data);
  has_height_ = true;
}

void RouteTargetPublisherNode::photoCaptureResultCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (mission_phase_ != MissionPhase::kWaitingPhotoAck) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Ignoring /photo_capture_result because no photo capture request is pending.");
    return;
  }

  photo_capture_result_ready_ = true;
  photo_capture_result_success_ = msg->data;
}

bool RouteTargetPublisherNode::getCurrentPose(
  double & x_cm,
  double & y_cm,
  double & z_cm,
  double & yaw_deg)
{
  try {
    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
      map_frame_, laser_link_frame_, tf2::TimePointZero);
    x_cm = meterToCm(transform.transform.translation.x);
    y_cm = meterToCm(transform.transform.translation.y);
    z_cm = has_height_ ? current_height_cm_ : 0.0;

    tf2::Quaternion q;
    tf2::fromMsg(transform.transform.rotation, q);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    yaw_deg = radToDeg(yaw);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "TF lookup failed (%s -> %s): %s",
      map_frame_.c_str(),
      laser_link_frame_.c_str(),
      ex.what());
    return false;
  }
}

bool RouteTargetPublisherNode::isReached(
  const Target & target,
  double x_cm,
  double y_cm,
  double z_cm,
  double yaw_deg) const
{
  const double dx = target.x_cm - x_cm;
  const double dy = target.y_cm - y_cm;
  const double dxy = std::hypot(dx, dy);
  const double dz = target.z_cm - z_cm;
  const double dyaw = normalizeAngleDeg(target.yaw_deg - yaw_deg);

  const bool z_ok = std::fabs(dz) <= height_tol_cm_;
  const bool xy_ok = dxy <= pos_tol_cm_;
  const bool yaw_ok = std::fabs(dyaw) <= yaw_tol_deg_;

  if (target.z_cm > 20.0) {
    if (current_idx_ == 0) {
      return z_ok;
    }
    return z_ok && xy_ok;
  }

  return z_ok && xy_ok && yaw_ok;
}

void RouteTargetPublisherNode::enterPhotoCapture()
{
  mission_phase_ = MissionPhase::kPhotoDescend;
  photo_capture_result_ready_ = false;
  pending_photo_filename_.clear();
  if (current_idx_ < targets_.size()) {
    publishTarget(getPublishedTarget(targets_[current_idx_]), false);
  }
  publishVisualTakeoverState(false);
  RCLCPP_INFO(
    get_logger(),
    "Target %zu reached. Starting photo capture flow at %.1fcm.",
    current_idx_,
    photo_target_height_cm_);
}

void RouteTargetPublisherNode::requestPhotoCapture()
{
  pending_photo_filename_ = buildPhotoFilename();
  photo_capture_result_ready_ = false;

  std_msgs::msg::String message;
  message.data = pending_photo_filename_;
  photo_capture_request_pub_->publish(message);

  photo_capture_request_time_ = now();
  mission_phase_ = MissionPhase::kWaitingPhotoAck;

  RCLCPP_INFO(
    get_logger(),
    "Requested photo capture for target %zu: %s",
    current_idx_,
    pending_photo_filename_.c_str());
}

void RouteTargetPublisherNode::finalizePhotoCapture(bool success, const std::string & reason)
{
  const std::string filename = pending_photo_filename_;
  pending_photo_filename_.clear();
  photo_capture_result_ready_ = false;

  if (success) {
    RCLCPP_INFO(
      get_logger(),
      "Photo capture succeeded for target %zu: %s",
      current_idx_,
      filename.c_str());
  } else {
    RCLCPP_WARN(
      get_logger(),
      "Photo capture failed for target %zu (%s). Skipping to next target.",
      current_idx_,
      reason.c_str());
  }

  advanceToNextTarget();
}

void RouteTargetPublisherNode::advanceToNextTarget()
{
  ++current_idx_;
  photo_capture_result_ready_ = false;
  pending_photo_filename_.clear();

  if (current_idx_ < targets_.size()) {
    publishCurrent();
  } else {
    current_idx_ = targets_.size();
    mission_phase_ = MissionPhase::kCompleted;
    publishVisualTakeoverState(false);
    if (!mission_complete_sent_ && mission_complete_pub_) {
      std_msgs::msg::Empty mission_complete_msg;
      mission_complete_pub_->publish(mission_complete_msg);
      mission_complete_sent_ = true;
    }
    std_msgs::msg::UInt8 active_msg;
    active_msg.data = 3;
    active_controller_pub_->publish(active_msg);
    RCLCPP_INFO(get_logger(), "All targets completed.");
  }
}

void RouteTargetPublisherNode::publishVisualTakeoverState(bool active)
{
  std_msgs::msg::Bool msg;
  msg.data = active;
  visual_takeover_active_pub_->publish(msg);
}

std::string RouteTargetPublisherNode::buildPhotoFilename() const
{
  const auto now_time = std::chrono::system_clock::now();
  const std::time_t now_tt = std::chrono::system_clock::to_time_t(now_time);

  std::tm local_tm{};
#if defined(_WIN32)
  localtime_s(&local_tm, &now_tt);
#else
  localtime_r(&now_tt, &local_tm);
#endif

  std::ostringstream oss;
  oss << "waypoint_" << std::setw(2) << std::setfill('0') << (current_idx_ + 1U) << "_"
      << std::put_time(&local_tm, "%Y%m%d_%H%M%S") << ".jpg";
  return oss.str();
}

bool RouteTargetPublisherNode::isPhotoCapturePhase() const
{
  return mission_phase_ == MissionPhase::kPhotoDescend ||
         mission_phase_ == MissionPhase::kPhotoDwell ||
         mission_phase_ == MissionPhase::kWaitingPhotoAck;
}

void RouteTargetPublisherNode::monitorTimerCallback()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (current_idx_ != std::numeric_limits<std::size_t>::max() && current_idx_ >= targets_.size()) {
    std_msgs::msg::UInt8 active_msg;
    active_msg.data = 3;
    active_controller_pub_->publish(active_msg);
    publishVisualTakeoverState(false);
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "All targets completed. Keeping stop signal active.");
    return;
  }

  if (current_idx_ == std::numeric_limits<std::size_t>::max()) {
    mission_phase_ = MissionPhase::kIdle;
    return;
  }

  double x_cm = 0.0;
  double y_cm = 0.0;
  double z_cm = 0.0;
  double yaw_deg = 0.0;
  if (!getCurrentPose(x_cm, y_cm, z_cm, yaw_deg)) {
    return;
  }

  const Target & target = targets_[current_idx_];
  const rclcpp::Time now_time = now();

  switch (mission_phase_) {
    case MissionPhase::kNormalNavigation:
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Current target %zu: x=%.1f y=%.1f z=%.1f yaw=%.1f photo=%s",
        current_idx_,
        target.x_cm,
        target.y_cm,
        target.z_cm,
        target.yaw_deg,
        target.is_takeover ? "true" : "false");

      if (isReached(target, x_cm, y_cm, z_cm, yaw_deg)) {
        const double dx = target.x_cm - x_cm;
        const double dy = target.y_cm - y_cm;
        const double dz = target.z_cm - z_cm;
        const double dyaw = normalizeAngleDeg(target.yaw_deg - yaw_deg);
        RCLCPP_INFO(
          get_logger(),
          "Target %zu reached: pos_err=(%.1f, %.1f, %.1f)cm yaw_err=%.1fdeg current=(%.1f, %.1f, %.1f, %.1f)",
          current_idx_,
          dx,
          dy,
          dz,
          dyaw,
          x_cm,
          y_cm,
          z_cm,
          yaw_deg);

        if (target.is_takeover) {
          enterPhotoCapture();
          return;
        }

        advanceToNextTarget();
      }
      return;

    case MissionPhase::kPhotoDescend: {
      const double height_error_cm = photo_target_height_cm_ - z_cm;
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Photo descend for target %zu: target_z=%.1fcm current_z=%.1fcm height_err=%.1fcm",
        current_idx_,
        photo_target_height_cm_,
        z_cm,
        height_error_cm);

      if (std::fabs(height_error_cm) <= height_tol_cm_) {
        mission_phase_ = MissionPhase::kPhotoDwell;
        photo_dwell_start_time_ = now_time;
        RCLCPP_INFO(
          get_logger(),
          "Photo height reached for target %zu. Starting dwell timer %.1fs.",
          current_idx_,
          photo_dwell_time_sec_);
      }
      return;
    }

    case MissionPhase::kPhotoDwell: {
      const double height_error_cm = photo_target_height_cm_ - z_cm;
      if (std::fabs(height_error_cm) > height_tol_cm_) {
        mission_phase_ = MissionPhase::kPhotoDescend;
        RCLCPP_WARN(
          get_logger(),
          "Target %zu drifted away from photo height by %.1fcm. Restarting descend phase.",
          current_idx_,
          height_error_cm);
        return;
      }

      const double dwell_elapsed = (now_time - photo_dwell_start_time_).seconds();
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Photo dwell for target %zu: %.1fs / %.1fs",
        current_idx_,
        dwell_elapsed,
        photo_dwell_time_sec_);

      if (dwell_elapsed >= photo_dwell_time_sec_) {
        requestPhotoCapture();
      }
      return;
    }

    case MissionPhase::kWaitingPhotoAck: {
      if (photo_capture_result_ready_) {
        finalizePhotoCapture(photo_capture_result_success_, photo_capture_result_success_ ? "ok" : "camera save failed");
        return;
      }

      const double wait_elapsed = (now_time - photo_capture_request_time_).seconds();
      if (wait_elapsed > photo_capture_timeout_sec_) {
        finalizePhotoCapture(false, "photo capture timed out");
        return;
      }

      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Waiting for photo capture result on target %zu: %.1fs / %.1fs",
        current_idx_,
        wait_elapsed,
        photo_capture_timeout_sec_);
      return;
    }

    case MissionPhase::kCompleted:
    case MissionPhase::kIdle:
      return;
  }
}

double RouteTargetPublisherNode::meterToCm(double value_m)
{
  return value_m * 100.0;
}

double RouteTargetPublisherNode::radToDeg(double value_rad)
{
  return value_rad * 180.0 / M_PI;
}

double RouteTargetPublisherNode::normalizeAngleDeg(double angle_deg) const
{
  const double normalized = angles::normalize_angle(angles::from_degrees(angle_deg));
  return angles::to_degrees(normalized);
}

RouteTestNode::RouteTestNode(
  const std::shared_ptr<RouteTargetPublisherNode> & route_node,
  const rclcpp::NodeOptions & options)
: rclcpp::Node("route_test_node", options),
  route_node_(route_node),
  route_locked_(false)
{
  std::setlocale(LC_ALL, "");

  routes_ = buildRoutes();
  route_choice_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "/route_choice",
    rclcpp::QoS(10),
    std::bind(&RouteTestNode::routeChoiceCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "Route selection node is waiting on /route_choice. Available routes: %zu",
    routes_.size());
}

void RouteTestNode::routeChoiceCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  const RouteId route_id = msg->data;
  if (route_locked_) {
    RCLCPP_INFO(
      get_logger(),
      "Ignoring /route_choice=%u because a route is already active or has already started.",
      static_cast<unsigned>(route_id));
    return;
  }

  const auto route_it = routes_.find(route_id);
  if (route_it == routes_.end()) {
    RCLCPP_WARN(
      get_logger(),
      "Received unsupported /route_choice=%u. Route will not start.",
      static_cast<unsigned>(route_id));
    return;
  }

  if (route_it->second.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "Received /route_choice=%u, but the route is empty. Ignoring.",
      static_cast<unsigned>(route_id));
    return;
  }

  loadRoute(route_id, route_it->second);
}

std::unordered_map<RouteTestNode::RouteId, std::vector<Target>> RouteTestNode::buildRoutes() const
{
  std::unordered_map<RouteId, std::vector<Target>> routes;

  routes.emplace(RouteId{1}, std::vector<Target>{
    Target{0.0, 0.0, 130.0, 0.0, false},
    Target{125.0, 100.0, 130.0, 0.0, true},
    Target{0.0, 0.0, 130.0, 0.0, false},
    Target{0.0, 0.0, 0.0, 0.0, false},
  });

  routes.emplace(RouteId{2}, std::vector<Target>{
    Target{0.0, 0.0, 130.0, 0.0, false},
    Target{125.0, -100.0, 130.0, 0.0, true},
    Target{0.0, 0.0, 130.0, 0.0, false},
    Target{0.0, 0.0, 0.0, 0.0, false},
  });

  return routes;
}

void RouteTestNode::loadRoute(RouteId route_id, const std::vector<Target> & route)
{
  route_locked_ = true;

  RCLCPP_INFO(
    get_logger(),
    "Received /route_choice=%u. Loading route with %zu targets.",
    static_cast<unsigned>(route_id),
    route.size());

  for (std::size_t index = 0; index < route.size(); ++index) {
    const auto & target = route[index];
    route_node_->addTarget(target);
    RCLCPP_INFO(
      get_logger(),
      "Loaded route %u target %zu/%zu: x=%.1f y=%.1f z=%.1f yaw=%.1f photo=%s",
      static_cast<unsigned>(route_id),
      index + 1,
      route.size(),
      target.x_cm,
      target.y_cm,
      target.z_cm,
      target.yaw_deg,
      target.is_takeover ? "true" : "false");
  }

  const auto current = route_node_->currentIndex();
  RCLCPP_INFO(
    get_logger(),
    "Route %u is now active. Current target index=%zu",
    static_cast<unsigned>(route_id),
    (current == std::numeric_limits<std::size_t>::max() ? 0 : current + 1));
}

}  // namespace activity_control_pkg
