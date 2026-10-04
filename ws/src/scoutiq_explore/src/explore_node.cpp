#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "scoutiq_explore/explore.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<scoutiq_explore::Explore>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
