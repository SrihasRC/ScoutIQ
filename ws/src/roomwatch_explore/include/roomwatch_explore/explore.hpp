#ifndef ROOMWATCH_EXPLORE__EXPLORE_HPP_
#define ROOMWATCH_EXPLORE__EXPLORE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "roomwatch_explore/costmap_client.hpp"
#include "roomwatch_explore/frontier_search.hpp"

namespace roomwatch_explore
{

class Explore : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit Explore(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~Explore();

  void start();
  void stop();
  void makePlan();
  bool saveMap(const std::string & custom_dir = "");

private:
  void visualizeFrontiers(const std::vector<Frontier> & frontiers);
  bool goalOnBlacklist(const geometry_msgs::msg::Point & goal);
  void reachedGoal(
    const GoalHandleNav::WrappedResult & result,
    const geometry_msgs::msg::Point & frontier_goal);

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<Costmap2DClient> costmap_client_;
  FrontierSearch search_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  GoalHandleNav::SharedPtr current_goal_handle_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr termination_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr contract_termination_publisher_;

  rclcpp::TimerBase::SharedPtr exploring_timer_;
  std::mutex planning_mutex_;

  std::vector<geometry_msgs::msg::Point> frontier_blacklist_;
  geometry_msgs::msg::Point prev_goal_;
  double prev_distance_{0.0};
  rclcpp::Time last_progress_;
  size_t last_markers_count_{0};
  bool is_exploring_{false};
  bool goal_in_flight_{false};

  // Parameters
  std::string robot_base_frame_{"base_link"};
  std::string costmap_topic_{"map"};
  std::string costmap_updates_topic_{"map_updates"};
  double planner_frequency_{1.0};
  double progress_timeout_{30.0};
  double potential_scale_{1e-3};
  double orientation_scale_{0.0};
  double gain_scale_{1.0};
  double min_frontier_size_{0.5};
  double min_frontier_spacing_{2.0};
  double local_frontier_filter_radius_{5.0};
  double min_local_frontiers_{5.0};
  double min_global_frontiers_{5.0};
  double global_frontier_filter_radius_{5.0};
  double transform_tolerance_{0.3};
  bool visualize_{true};
  bool save_map_{true};
  std::string run_dir_{""};
  std::string data_dir_{"data"};
};

}  // namespace roomwatch_explore

#endif  // ROOMWATCH_EXPLORE__EXPLORE_HPP_
