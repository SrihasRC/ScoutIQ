#include "scoutiq_explore/costmap_client.hpp"

#include <mutex>
#include <string>

#include "nav2_costmap_2d/cost_values.hpp"
#include <tf2/exceptions.h>

namespace scoutiq_explore
{

static std::array<unsigned char, 256> init_translation_table()
{
  std::array<unsigned char, 256> cost_translation_table;

  // Linearly mapped from [0..100] to [0..255]
  for (size_t i = 0; i < 256; ++i) {
    cost_translation_table[i] = static_cast<unsigned char>(1 + (251 * (i - 1)) / 97);
  }

  cost_translation_table[0] = nav2_costmap_2d::FREE_SPACE;
  cost_translation_table[99] = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  cost_translation_table[100] = nav2_costmap_2d::LETHAL_OBSTACLE;
  cost_translation_table[static_cast<unsigned char>(-1)] = nav2_costmap_2d::NO_INFORMATION;

  return cost_translation_table;
}

static const std::array<unsigned char, 256> cost_translation_table__ = init_translation_table();

Costmap2DClient::Costmap2DClient(
  rclcpp::Node * node,
  tf2_ros::Buffer * tf_buffer,
  const std::string & costmap_topic,
  const std::string & costmap_updates_topic,
  const std::string & robot_base_frame,
  double transform_tolerance)
: node_(node),
  tf_buffer_(tf_buffer),
  global_frame_("map"),
  robot_base_frame_(robot_base_frame),
  transform_tolerance_(transform_tolerance),
  has_map_(false)
{
  // Transient-local QoS per CONTRACT for maps
  auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    costmap_topic, map_qos,
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      updateFullMap(msg);
    });

  auto updates_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  costmap_updates_sub_ = node_->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
    costmap_updates_topic, updates_qos,
    [this](const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg) {
      updatePartialMap(msg);
    });

  RCLCPP_INFO(
    node_->get_logger(),
    "Costmap2DClient subscribed to costmap topic: %s, updates: %s",
    costmap_topic.c_str(), costmap_updates_topic.c_str());
}

void Costmap2DClient::updateFullMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  global_frame_ = msg->header.frame_id;

  unsigned int size_in_cells_x = msg->info.width;
  unsigned int size_in_cells_y = msg->info.height;
  double resolution = msg->info.resolution;
  double origin_x = msg->info.origin.position.x;
  double origin_y = msg->info.origin.position.y;

  costmap_.resizeMap(size_in_cells_x, size_in_cells_y, resolution, origin_x, origin_y);

  {
    std::lock_guard<std::recursive_mutex> lock(*(costmap_.getMutex()));
    unsigned char * costmap_data = costmap_.getCharMap();
    size_t costmap_size = costmap_.getSizeInCellsX() * costmap_.getSizeInCellsY();
    for (size_t i = 0; i < costmap_size && i < msg->data.size(); ++i) {
      unsigned char cell_cost = static_cast<unsigned char>(msg->data[i]);
      costmap_data[i] = cost_translation_table__[cell_cost];
    }
  }

  latest_map_msg_ = msg;
  has_map_ = true;
  RCLCPP_DEBUG(
    node_->get_logger(),
    "Costmap2DClient received full map: %u x %u @ %.3f m/cell",
    size_in_cells_x, size_in_cells_y, resolution);
}

void Costmap2DClient::updatePartialMap(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg)
{
  if (msg->x < 0 || msg->y < 0) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Costmap2DClient: negative coordinates in partial map update: x: %d, y: %d",
      msg->x, msg->y);
    return;
  }
  global_frame_ = msg->header.frame_id;

  size_t x0 = static_cast<size_t>(msg->x);
  size_t y0 = static_cast<size_t>(msg->y);
  size_t xn = msg->width + x0;
  size_t yn = msg->height + y0;

  std::lock_guard<std::recursive_mutex> lock(*(costmap_.getMutex()));
  size_t costmap_xn = costmap_.getSizeInCellsX();
  size_t costmap_yn = costmap_.getSizeInCellsY();

  unsigned char * costmap_data = costmap_.getCharMap();
  size_t i = 0;
  for (size_t y = y0; y < yn && y < costmap_yn; ++y) {
    for (size_t x = x0; x < xn && x < costmap_xn; ++x) {
      size_t idx = costmap_.getIndex(x, y);
      unsigned char cell_cost = static_cast<unsigned char>(msg->data[i]);
      costmap_data[idx] = cost_translation_table__[cell_cost];
      if (latest_map_msg_ && idx < latest_map_msg_->data.size()) {
        latest_map_msg_->data[idx] = msg->data[i];
      }
      ++i;
    }
  }
}

bool Costmap2DClient::getRobotPose(geometry_msgs::msg::Pose & pose) const
{
  if (tf_buffer_ == nullptr) {
    return false;
  }

  try {
    auto tf_stamped = tf_buffer_->lookupTransform(
      global_frame_, robot_base_frame_, tf2::TimePointZero,
      tf2::durationFromSec(transform_tolerance_));
    pose.position.x = tf_stamped.transform.translation.x;
    pose.position.y = tf_stamped.transform.translation.y;
    pose.position.z = tf_stamped.transform.translation.z;
    pose.orientation = tf_stamped.transform.rotation;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "Costmap2DClient: lookupTransform from %s to %s failed: %s",
      global_frame_.c_str(), robot_base_frame_.c_str(), ex.what());
    return false;
  }
}

}  // namespace scoutiq_explore
