#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <numeric>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

class DroneCameraNode : public rclcpp::Node
{
public:
  DroneCameraNode()
  : Node("drone_camera_node"),
    camera_device_(declare_parameter<std::string>("camera_device", "/dev/video0")),
    frame_width_(declare_parameter<int>("frame_width", 640)),
    frame_height_(declare_parameter<int>("frame_height", 480)),
    fps_(declare_parameter<double>("fps", 15.0)),
    window_name_(declare_parameter<std::string>("window_name", "drone_camera_preview")),
    fine_data_topic_(declare_parameter<std::string>("fine_data_topic", "/fine_data")),
    apriltag_code_topic_(declare_parameter<std::string>("apriltag_code_topic", "/apriltag_code")),
    photo_capture_request_topic_(
      declare_parameter<std::string>("photo_capture_request_topic", "/photo_capture_request")),
    photo_capture_result_topic_(
      declare_parameter<std::string>("photo_capture_result_topic", "/photo_capture_result")),
    photo_save_dir_(declare_parameter<std::string>("photo_save_dir", "src/photo")),
    jpeg_quality_(declare_parameter<int>("jpeg_quality", 95)),
    apriltag_dictionary_name_(
      declare_parameter<std::string>("apriltag_dictionary", "DICT_APRILTAG_36h11"))
  {
    fine_data_pub_ =
      create_publisher<std_msgs::msg::Int32MultiArray>(fine_data_topic_, rclcpp::QoS(10));
    apriltag_code_pub_ =
      create_publisher<std_msgs::msg::Int32>(apriltag_code_topic_, rclcpp::QoS(10));
    photo_capture_result_pub_ =
      create_publisher<std_msgs::msg::Bool>(photo_capture_result_topic_, rclcpp::QoS(10));
    photo_capture_request_sub_ = create_subscription<std_msgs::msg::String>(
      photo_capture_request_topic_,
      rclcpp::QoS(10),
      std::bind(&DroneCameraNode::photoCaptureRequestCallback, this, std::placeholders::_1));

    apriltag_dictionary_ = cv::aruco::getPredefinedDictionary(
      dictionaryNameToId(apriltag_dictionary_name_));
    detector_params_ = cv::aruco::DetectorParameters::create();

    ensurePhotoSaveDirExists();

    if (!camera_.open(camera_device_)) {
      throw std::runtime_error("Failed to open camera device " + camera_device_);
    }

    if (frame_width_ > 0) {
      camera_.set(cv::CAP_PROP_FRAME_WIDTH, frame_width_);
    }
    if (frame_height_ > 0) {
      camera_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height_);
    }
    if (fps_ > 0.0) {
      camera_.set(cv::CAP_PROP_FPS, fps_);
    }

    const auto period = std::chrono::duration<double>(1.0 / std::max(fps_, 1.0));
    frame_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&DroneCameraNode::frameTimerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "Camera preview enabled. camera_device=%s fine_data_topic=%s apriltag_code_topic=%s photo_dir=%s",
      camera_device_.c_str(),
      fine_data_topic_.c_str(),
      apriltag_code_topic_.c_str(),
      std::filesystem::absolute(photo_save_dir_).string().c_str());
  }

  ~DroneCameraNode() override
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (camera_.isOpened()) {
      camera_.release();
    }
    cv::destroyAllWindows();
  }

private:
  static int dictionaryNameToId(const std::string & dictionary_name)
  {
    if (dictionary_name == "DICT_APRILTAG_16h5") {
      return cv::aruco::DICT_APRILTAG_16h5;
    }
    if (dictionary_name == "DICT_APRILTAG_25h9") {
      return cv::aruco::DICT_APRILTAG_25h9;
    }
    if (dictionary_name == "DICT_APRILTAG_36h10") {
      return cv::aruco::DICT_APRILTAG_36h10;
    }
    if (dictionary_name == "DICT_APRILTAG_36h11") {
      return cv::aruco::DICT_APRILTAG_36h11;
    }

    throw std::runtime_error("Unsupported AprilTag dictionary: " + dictionary_name);
  }

  static double contourAreaFromCorners(const std::vector<cv::Point2f> & corners)
  {
    if (corners.size() < 4U) {
      return 0.0;
    }
    return std::abs(cv::contourArea(corners));
  }

  void ensurePhotoSaveDirExists() const
  {
    std::error_code ec;
    std::filesystem::create_directories(photo_save_dir_, ec);
    if (ec) {
      throw std::runtime_error(
        "Failed to create photo directory " + std::filesystem::absolute(photo_save_dir_).string() +
        ": " + ec.message());
    }
  }

  void publishPhotoCaptureResult(bool success) const
  {
    std_msgs::msg::Bool msg;
    msg.data = success;
    photo_capture_result_pub_->publish(msg);
  }

  cv::Mat captureLatestFrame()
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (!camera_.isOpened()) {
      return cv::Mat();
    }

    if (latest_frame_.empty()) {
      cv::Mat frame;
      if (!camera_.read(frame) || frame.empty()) {
        return cv::Mat();
      }
      latest_frame_ = frame.clone();
    }

    return latest_frame_.clone();
  }

  void photoCaptureRequestCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string requested_filename = std::filesystem::path(msg->data).filename().string();
    if (requested_filename.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty /photo_capture_request filename.");
      publishPhotoCaptureResult(false);
      return;
    }

    try {
      ensurePhotoSaveDirExists();
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Photo directory setup failed: %s", ex.what());
      publishPhotoCaptureResult(false);
      return;
    }

    cv::Mat frame = captureLatestFrame();
    if (frame.empty()) {
      RCLCPP_WARN(get_logger(), "No valid camera frame available for photo capture.");
      publishPhotoCaptureResult(false);
      return;
    }

    const std::filesystem::path output_path = std::filesystem::path(photo_save_dir_) / requested_filename;
    std::vector<int> params;
    const std::string extension = output_path.extension().string();
    if (extension == ".jpg" || extension == ".jpeg" || extension == ".JPG" || extension == ".JPEG") {
      params = {cv::IMWRITE_JPEG_QUALITY, std::clamp(jpeg_quality_, 0, 100)};
    }

    bool write_ok = false;
    try {
      write_ok = cv::imwrite(output_path.string(), frame, params);
    } catch (const cv::Exception & ex) {
      RCLCPP_ERROR(
        get_logger(),
        "OpenCV failed to save %s: %s",
        output_path.string().c_str(),
        ex.what());
      publishPhotoCaptureResult(false);
      return;
    }

    if (!write_ok) {
      RCLCPP_WARN(
        get_logger(),
        "cv::imwrite returned false while saving %s",
        output_path.string().c_str());
      publishPhotoCaptureResult(false);
      return;
    }

    RCLCPP_INFO(get_logger(), "Saved inspection photo: %s", output_path.string().c_str());
    publishPhotoCaptureResult(true);
  }

  void detectAprilTagAndPublish(const cv::Mat & frame)
  {
    std::vector<int> tag_ids;
    std::vector<std::vector<cv::Point2f>> tag_corners;

    cv::aruco::detectMarkers(frame, apriltag_dictionary_, tag_corners, tag_ids, detector_params_);
    if (tag_ids.empty()) {
      return;
    }

    std::size_t best_index = 0;
    double best_area = contourAreaFromCorners(tag_corners.front());
    for (std::size_t i = 1; i < tag_corners.size(); ++i) {
      const double area = contourAreaFromCorners(tag_corners[i]);
      if (area > best_area) {
        best_area = area;
        best_index = i;
      }
    }

    const auto & best_corners = tag_corners[best_index];
    const auto center = std::accumulate(
      best_corners.begin(),
      best_corners.end(),
      cv::Point2f(0.0F, 0.0F),
      [](const cv::Point2f & sum, const cv::Point2f & point) {
        return cv::Point2f(sum.x + point.x, sum.y + point.y);
      });

    const float tag_center_x = center.x / static_cast<float>(best_corners.size());
    const float tag_center_y = center.y / static_cast<float>(best_corners.size());
    const float image_center_x = static_cast<float>(frame.cols) / 2.0F;
    const float image_center_y = static_cast<float>(frame.rows) / 2.0F;

    const int x_offset = static_cast<int>(std::lround(image_center_y - tag_center_y));
    const int y_offset = static_cast<int>(std::lround(image_center_x - tag_center_x));

    std_msgs::msg::Int32MultiArray fine_data_msg;
    fine_data_msg.data = {x_offset, y_offset};
    fine_data_pub_->publish(fine_data_msg);

    std_msgs::msg::Int32 apriltag_code_msg;
    apriltag_code_msg.data = tag_ids[best_index];
    apriltag_code_pub_->publish(apriltag_code_msg);
  }

  void frameTimerCallback()
  {
    cv::Mat frame;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (!camera_.isOpened()) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000, "Camera is not opened.");
        return;
      }

      if (!camera_.read(frame) || frame.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Failed to read frame from camera.");
        return;
      }

      latest_frame_ = frame.clone();
    }

    cv::imshow(window_name_, frame);
    cv::waitKey(1);

    detectAprilTagAndPublish(frame);
  }

  std::string camera_device_;
  int frame_width_;
  int frame_height_;
  double fps_;
  std::string window_name_;
  std::string fine_data_topic_;
  std::string apriltag_code_topic_;
  std::string photo_capture_request_topic_;
  std::string photo_capture_result_topic_;
  std::string photo_save_dir_;
  int jpeg_quality_;
  std::string apriltag_dictionary_name_;

  std::mutex frame_mutex_;
  cv::VideoCapture camera_;
  cv::Mat latest_frame_;
  cv::Ptr<cv::aruco::Dictionary> apriltag_dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr fine_data_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr apriltag_code_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr photo_capture_result_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr photo_capture_request_sub_;
  rclcpp::TimerBase::SharedPtr frame_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DroneCameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
