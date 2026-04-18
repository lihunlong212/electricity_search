#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace activity_control_pkg
{

struct Target
{
  double x_cm;
  double y_cm;
  double z_cm;
  double yaw_deg;
  bool is_takeover = false;
};

class RouteTargetPublisherNode : public rclcpp::Node
{
public:
  explicit RouteTargetPublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void addTarget(const Target & target);
  std::size_t currentIndex() const;
  std::size_t size() const;

private:
  enum class MissionPhase
  {
    kIdle,
    kNormalNavigation,
    kPhotoDescend,
    kPhotoDwell,
    kWaitingPhotoAck,
    kCompleted
  };

  void publishCurrent();
  void publishTarget(const Target & target, bool init_flag);
  Target getPublishedTarget(const Target & target) const;

  bool getCurrentPose(double & x_cm, double & y_cm, double & z_cm, double & yaw_deg);
  bool isReached(const Target & target, double x_cm, double y_cm, double z_cm, double yaw_deg) const;

  void monitorTimerCallback();
  void heightCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void photoCaptureResultCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void enterPhotoCapture();
  void requestPhotoCapture();
  void finalizePhotoCapture(bool success, const std::string & reason);
  void advanceToNextTarget();
  void publishVisualTakeoverState(bool active);
  std::string buildPhotoFilename() const;
  bool isPhotoCapturePhase() const;

  static double meterToCm(double value_m);
  static double radToDeg(double value_rad);
  double normalizeAngleDeg(double angle_deg) const;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr active_controller_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr visual_takeover_active_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr photo_capture_request_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr mission_complete_pub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr height_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr photo_capture_result_sub_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  mutable std::mutex mutex_;
  std::vector<Target> targets_;
  std::size_t current_idx_;

  bool has_height_;
  double current_height_cm_;

  double pos_tol_cm_;
  double yaw_tol_deg_;
  double height_tol_cm_;
  double photo_target_height_cm_;
  double photo_dwell_time_sec_;
  double photo_capture_timeout_sec_;

  std::string map_frame_;
  std::string laser_link_frame_;
  std::string output_topic_;
  bool mission_complete_sent_;
  MissionPhase mission_phase_;
  rclcpp::Time photo_dwell_start_time_;
  rclcpp::Time photo_capture_request_time_;
  bool photo_capture_result_ready_;
  bool photo_capture_result_success_;
  std::string pending_photo_filename_;
};

class RouteTestNode : public rclcpp::Node
{
public:
  explicit RouteTestNode(
    const std::shared_ptr<RouteTargetPublisherNode> & route_node,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using RouteId = std::uint8_t;

  void routeChoiceCallback(const std_msgs::msg::UInt8::SharedPtr msg);
  std::unordered_map<RouteId, std::vector<Target>> buildRoutes() const;
  void loadRoute(RouteId route_id, const std::vector<Target> & route);

  std::shared_ptr<RouteTargetPublisherNode> route_node_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr route_choice_sub_;
  std::unordered_map<RouteId, std::vector<Target>> routes_;
  bool route_locked_;
};

}  // namespace activity_control_pkg
