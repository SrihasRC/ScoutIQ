#ifndef SCOUTIQ_EXPLORE__COSTMAP_CLIENT_HPP_
#define SCOUTIQ_EXPLORE__COSTMAP_CLIENT_HPP_

#include <array>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

namespace scoutiq_explore
{

class Costmap2DClient
{
public:
  Costmap2DClient(
    rclcpp::Node * node,
    tf2_ros::Buffer * tf_buffer,
    const std::string & costmap_topic,
    const std::string & costmap_updates_topic,
    const std::string & robot_base_frame,
    double transform_tolerance);

  bool getRobotPose(geometry_msgs::msg::Pose & pose) const;

  nav2_costmap_2d::Costmap2D * getCostmap()
  {
    return &costmap_;
  }

  const nav2_costmap_2d::Costmap2D * getCostmap() const
  {
    return &costmap_;
  }

  const std::string & getGlobalFrameID() const
  {
    return global_frame_;
  }

  const std::string & getBaseFrameID() const
  {
    return robot_base_frame_;
  }

  bool hasMap() const
  {
    return has_map_;
  }

  nav_msgs::msg::OccupancyGrid::SharedPtr getLatestOccupancyGrid() const
  {
    return latest_map_msg_;
  }

protected:
  void updateFullMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void updatePartialMap(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg);

  rclcpp::Node * node_{nullptr};
  tf2_ros::Buffer * tf_buffer_{nullptr};

  nav2_costmap_2d::Costmap2D costmap_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_msg_;

  std::string global_frame_{"map"};
  std::string robot_base_frame_{"base_link"};
  double transform_tolerance_{0.3};
  bool has_map_{false};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr costmap_updates_sub_;
};

}  // namespace scoutiq_explore

#endif  // SCOUTIQ_EXPLORE__COSTMAP_CLIENT_HPP_
