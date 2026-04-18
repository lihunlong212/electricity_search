#include <clocale>
#include <string>
#include <vector>

#include "activity_control_pkg/route_target_publisher.hpp"

int main(int argc, char ** argv)
{
  std::setlocale(LC_ALL, "");
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  std::vector<std::string> args;
  for (int i = 1; i < argc; ++i) {
    args.emplace_back(argv[i]);
  }
  options.arguments(args);

  auto node = std::make_shared<activity_control_pkg::RouteTargetPublisherNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
