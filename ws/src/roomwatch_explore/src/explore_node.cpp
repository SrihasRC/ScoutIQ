#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "roomwatch_explore/explore.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<roomwatch_explore::Explore>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
