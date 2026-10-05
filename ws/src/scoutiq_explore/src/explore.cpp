#include "scoutiq_explore/explore.hpp"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_map_server/map_io.hpp"
#include "scoutiq_explore/distance.hpp"

namespace scoutiq_explore
{

Explore::Explore(const rclcpp::NodeOptions & options)
: Node("explore", options),
  prev_distance_(0.0),
  last_markers_count_(0),
  is_exploring_(false)
{
  // Declare parameters
  robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "base_link");
  costmap_topic_ = declare_parameter<std::string>("costmap_topic", "map");
  costmap_updates_topic_ = declare_parameter<std::string>("costmap_updates_topic", "map_updates");
  planner_frequency_ = declare_parameter<double>("planner_frequency", 1.0);
  progress_timeout_ = declare_parameter<double>("progress_timeout", 30.0);
  visualize_ = declare_parameter<bool>("visualize", true);
  potential_scale_ = declare_parameter<double>("potential_scale", 1e-3);
  orientation_scale_ = declare_parameter<double>("orientation_scale", 0.0);
  gain_scale_ = declare_parameter<double>("gain_scale", 1.0);
  min_frontier_size_ = declare_parameter<double>("min_frontier_size", 0.5);
  local_frontier_filter_radius_ = declare_parameter<double>("local_frontier_filter_radius", 5.0);
  min_local_frontiers_ = declare_parameter<double>("min_local_frontiers", 5.0);
  min_global_frontiers_ = declare_parameter<double>("min_global_frontiers", 5.0);
  global_frontier_filter_radius_ = declare_parameter<double>("global_frontier_filter_radius", 5.0);
  min_frontier_spacing_ = declare_parameter<double>("min_frontier_spacing", 2.0);
  transform_tolerance_ = declare_parameter<double>("transform_tolerance", 0.3);
  save_map_ = declare_parameter<bool>("save_map", true);
  run_dir_ = declare_parameter<std::string>("run_dir", "");
  data_dir_ = declare_parameter<std::string>("data_dir", "data");

  RCLCPP_INFO(
    get_logger(),
    "Dynamic Window Frontier Exploration Node Initialized:\n"
    "  planner_frequency: %.2f Hz\n"
    "  potential_scale: %.4f\n"
    "  gain_scale: %.2f\n"
    "  min_frontier_size: %.2f m\n"
    "  local_frontier_filter_radius: %.2f m\n"
    "  min_local_frontiers: %.1f\n"
    "  min_global_frontiers: %.1f\n"
    "  global_frontier_filter_radius: %.2f m\n"
    "  min_frontier_spacing: %.2f m\n"
    "  save_map: %s\n"
    "  run_dir: '%s'",
    planner_frequency_, potential_scale_, gain_scale_, min_frontier_size_,
    local_frontier_filter_radius_, min_local_frontiers_, min_global_frontiers_,
    global_frontier_filter_radius_, min_frontier_spacing_,
    save_map_ ? "true" : "false", run_dir_.c_str());

  // Setup TF
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Setup Costmap client
  costmap_client_ = std::make_unique<Costmap2DClient>(
    this, tf_buffer_.get(), costmap_topic_, costmap_updates_topic_,
    robot_base_frame_, transform_tolerance_);

  // Setup Frontier search
  search_ = FrontierSearch(
    costmap_client_->getCostmap(), potential_scale_, gain_scale_,
    min_frontier_size_, min_frontier_spacing_);

  // Publishers
  termination_publisher_ = create_publisher<std_msgs::msg::Bool>(
    "explore/exploration_termination", 10);
  contract_termination_publisher_ = create_publisher<std_msgs::msg::Bool>(
    "exploration_termination", 10);

  if (visualize_) {
    marker_array_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "frontiers", 10);
  }

  // Nav2 Action Client
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

  last_progress_ = now();
  prev_goal_.x = std::numeric_limits<double>::infinity();
  prev_goal_.y = std::numeric_limits<double>::infinity();
  start();
}

Explore::~Explore()
{
  stop();
}

void Explore::start()
{
  if (!is_exploring_) {
    is_exploring_ = true;
    start_time_ = now();
    last_progress_ = now();
    consecutive_empty_frontiers_ = 0;
    auto period = std::chrono::duration<double>(1.0 / std::max(planner_frequency_, 0.01));
    exploring_timer_ = create_wall_timer(period, [this]() { makePlan(); });
    RCLCPP_INFO(get_logger(), "Exploration timer started.");
  }
}

void Explore::stop()
{
  if (is_exploring_) {
    is_exploring_ = false;
    if (exploring_timer_) {
      exploring_timer_->cancel();
    }
    if (current_goal_handle_) {
      nav_client_->async_cancel_goal(current_goal_handle_);
      current_goal_handle_ = nullptr;
    }
    goal_in_flight_ = false;
    if (save_map_) {
      saveMap();
    }
    RCLCPP_INFO(get_logger(), "Exploration stopped.");
  }
}

void Explore::visualizeFrontiers(const std::vector<Frontier> & frontiers)
{
  if (!visualize_ || !marker_array_publisher_) {
    return;
  }

  visualization_msgs::msg::MarkerArray markers_msg;
  auto & markers = markers_msg.markers;

  visualization_msgs::msg::Marker m;
  m.header.frame_id = costmap_client_->getGlobalFrameID();
  m.header.stamp = now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0.0f;
  m.color.g = 0.0f;
  m.color.b = 1.0f;
  m.color.a = 1.0f;
  m.lifetime = rclcpp::Duration(0, 0);
  m.frame_locked = true;

  double min_cost = frontiers.empty() ? 0.0 : frontiers.front().cost;
  size_t id = 0;

  for (const auto & frontier : frontiers) {
    // Frontier boundary points
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.id = static_cast<int>(id);
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;

    if (goalOnBlacklist(frontier.centroid)) {
      m.color.r = 1.0f;
      m.color.g = 0.0f;
      m.color.b = 0.0f;
      m.color.a = 1.0f;
    } else {
      m.color.r = 0.0f;
      m.color.g = 0.0f;
      m.color.b = 1.0f;
      m.color.a = 1.0f;
    }
    markers.push_back(m);
    ++id;

    // Frontier centroid sphere
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.id = static_cast<int>(id);
    m.pose.position = frontier.centroid;
    double scale = std::min(std::abs(min_cost * 0.4 / std::max(std::abs(frontier.cost), 1e-4)), 0.5);
    scale = std::max(scale, 0.1);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points.clear();
    m.color.r = 0.0f;
    m.color.g = 1.0f;
    m.color.b = 0.0f;
    m.color.a = 1.0f;
    markers.push_back(m);
    ++id;
  }

  size_t current_markers_count = markers.size();

  // Delete previous markers that are no longer used
  m.action = visualization_msgs::msg::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = static_cast<int>(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_->publish(markers_msg);
}

bool Explore::goalOnBlacklist(const geometry_msgs::msg::Point & goal)
{
  constexpr static size_t tolerance = 5;
  auto * costmap = costmap_client_->getCostmap();
  double res = costmap ? costmap->getResolution() : 0.05;

  for (const auto & frontier_goal : frontier_blacklist_) {
    double x_diff = std::abs(goal.x - frontier_goal.x);
    double y_diff = std::abs(goal.y - frontier_goal.y);

    if (x_diff < tolerance * res && y_diff < tolerance * res) {
      return true;
    }
  }
  return false;
}

void Explore::makePlan()
{
  std::lock_guard<std::mutex> lock(planning_mutex_);

  if (!is_exploring_) {
    return;
  }

  // Warm-up check: wait for SLAM to settle and publish initial scan
  if ((now() - start_time_).seconds() < 3.0) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Waiting for SLAM map and sensors to initialize (%.1f s elapsed)...",
      (now() - start_time_).seconds());
    return;
  }

  if (!costmap_client_->hasMap()) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Waiting for costmap to become available on topic '%s'...",
      costmap_topic_.c_str());
    return;
  }

  if (!nav_client_->action_server_is_ready()) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Waiting for 'navigate_to_pose' action server to become available...");
    return;
  }

  geometry_msgs::msg::Pose pose;
  if (!costmap_client_->getRobotPose(pose)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Could not get robot pose for frontier search.");
    return;
  }

  // If we already have an active goal being navigated by Nav2:
  if (current_goal_handle_ != nullptr) {
    double current_dist_to_goal = euclideanDistance(pose.position, prev_goal_);
    if (current_dist_to_goal < prev_distance_ - 0.05) {
      last_progress_ = now();
      prev_distance_ = current_dist_to_goal;
    }

    if ((now() - last_progress_).seconds() > progress_timeout_) {
      RCLCPP_WARN(
        get_logger(),
        "Progress timeout (%.1f s) exceeded for goal (%.2f, %.2f). Adding to blacklist.",
        progress_timeout_, prev_goal_.x, prev_goal_.y);
      frontier_blacklist_.push_back(prev_goal_);
      nav_client_->async_cancel_goal(current_goal_handle_);
      current_goal_handle_ = nullptr;
      prev_goal_.x = std::numeric_limits<double>::infinity();
      prev_goal_.y = std::numeric_limits<double>::infinity();
      last_progress_ = now();
    } else {
      // Robot is actively executing path towards prev_goal_. Let it drive smoothly!
      return;
    }
  }

  // Find all raw frontiers across whole map, sorted by cost
  auto all_frontiers = search_.searchFrom(pose.position);

  // Filter out frontiers too close to the robot (< min_candidate_distance_)
  // or positioned inside high costmap inflation (tight crevices / narrow gaps behind furniture)
  auto * costmap = costmap_client_->getCostmap();
  std::vector<Frontier> frontiers;
  for (const auto & f : all_frontiers) {
    if (euclideanDistance(pose.position, f.middle) < min_candidate_distance_) {
      continue;
    }
    if (costmap != nullptr) {
      unsigned int mx = 0;
      unsigned int my = 0;
      if (costmap->worldToMap(f.middle.x, f.middle.y, mx, my)) {
        unsigned char c = costmap->getCost(mx, my);
        // Skip frontiers with cost >= 180 (close to obstacle, < ~0.35m clearance) to prevent wedging
        if (c >= 180 && c != nav2_costmap_2d::NO_INFORMATION) {
          continue;
        }
      }
    }
    frontiers.push_back(f);
  }

  // Exploration termination condition:
  // Terminate ONLY if no frontiers remain AND sufficient time has elapsed (> 45s)
  // AND consecutive cycles confirm zero frontiers across the map.
  if (frontiers.size() < min_global_frontiers_) {
    consecutive_empty_frontiers_++;
    if (consecutive_empty_frontiers_ >= 15 && (now() - start_time_).seconds() > 45.0) {
      RCLCPP_INFO(
        get_logger(),
        "Exploration complete: remaining frontiers (%zu) < min_global_frontiers (%.1f) over %d cycles. Stopping.",
        frontiers.size(), min_global_frontiers_, consecutive_empty_frontiers_);

      std_msgs::msg::Bool term_msg;
      term_msg.data = true;
      termination_publisher_->publish(term_msg);
      contract_termination_publisher_->publish(term_msg);

      stop();

      if (save_map_) {
        saveMap();
      }
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Low or no frontiers detected (%zu); waiting for map updates (cycle %d/10)...",
        frontiers.size(), consecutive_empty_frontiers_);
    }
    return;
  }
  consecutive_empty_frontiers_ = 0;

  // Dynamic window filtering:
  // 1. Filter frontiers within local_frontier_filter_radius
  std::vector<Frontier> filtered_frontiers;
  double radius_ = local_frontier_filter_radius_;

  RCLCPP_DEBUG(get_logger(), "Found %zu frontiers from whole map", frontiers.size());
  for (const auto & f : frontiers) {
    if (euclideanDistance(pose.position, f.centroid) <= radius_) {
      filtered_frontiers.push_back(f);
    }
  }
  RCLCPP_DEBUG(
    get_logger(), "Filtered frontiers within local radius (%.2f m): %zu",
    radius_, filtered_frontiers.size());

  // 2. Window expansion: if local frontiers <= min_local_frontiers, expand to global
  if (filtered_frontiers.size() <= min_local_frontiers_) {
    radius_ = global_frontier_filter_radius_;
    filtered_frontiers = frontiers;  // Expand to whole map
    RCLCPP_INFO(
      get_logger(),
      "****Not enough frontiers within local filter radius (<= %.1f) || "
      "Expanding search to whole map (%.2f m)****",
      min_local_frontiers_, radius_);
  } else {
    RCLCPP_INFO(
      get_logger(),
      "****Picking a frontier within the filter radius (%.2f m, count: %zu)****",
      radius_, filtered_frontiers.size());
  }

  if (visualize_) {
    visualizeFrontiers(filtered_frontiers);
  }

  // Find first non-blacklisted frontier
  auto frontier_it = std::find_if_not(
    filtered_frontiers.begin(), filtered_frontiers.end(),
    [this](const Frontier & f) { return goalOnBlacklist(f.middle); });

  // If none found in local filtered set, fallback to search in all global frontiers
  if (frontier_it == filtered_frontiers.end() && radius_ != global_frontier_filter_radius_) {
    frontier_it = std::find_if_not(
      frontiers.begin(), frontiers.end(),
      [this](const Frontier & f) { return goalOnBlacklist(f.middle); });
  }

  if (frontier_it == filtered_frontiers.end() || frontier_it == frontiers.end()) {
    blacklisted_cycles_++;
    if (blacklisted_cycles_ >= 5) {
      RCLCPP_WARN(
        get_logger(),
        "All available frontiers (%zu) have remained blacklisted for %d cycles. Clearing blacklist to retry.",
        frontiers.size(), blacklisted_cycles_);
      frontier_blacklist_.clear();
      blacklisted_cycles_ = 0;
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "All current frontiers (%zu) are temporarily blacklisted; waiting for map update (cycle %d/5)...",
        frontiers.size(), blacklisted_cycles_);
    }
    return;
  }
  blacklisted_cycles_ = 0;

  geometry_msgs::msg::Point target_position = frontier_it->middle;

  if (goal_in_flight_) {
    return;
  }

  prev_goal_ = target_position;
  prev_distance_ = euclideanDistance(pose.position, target_position);
  last_progress_ = now();

  // Dispatch goal to NavigateToPose action server
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.frame_id = costmap_client_->getGlobalFrameID();
  goal_msg.pose.header.stamp = now();
  goal_msg.pose.pose.position = target_position;

  double dx = target_position.x - pose.position.x;
  double dy = target_position.y - pose.position.y;
  double yaw = std::atan2(dy, dx);
  goal_msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
  goal_msg.pose.pose.orientation.w = std::cos(yaw / 2.0);
  goal_start_robot_pos_ = pose.position;

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    [this, target_position](const GoalHandleNav::SharedPtr & goal_handle) {
      goal_in_flight_ = false;
      if (!goal_handle) {
        RCLCPP_WARN(
          get_logger(),
          "NavigateToPose goal rejected by action server for (%.2f, %.2f)",
          target_position.x, target_position.y);
        frontier_blacklist_.push_back(target_position);
        prev_goal_.x = std::numeric_limits<double>::infinity();
      } else {
        RCLCPP_INFO(
          get_logger(),
          "NavigateToPose goal accepted for target (%.2f, %.2f)",
          target_position.x, target_position.y);
        current_goal_handle_ = goal_handle;
      }
    };

  send_goal_options.result_callback =
    [this, target_position](const GoalHandleNav::WrappedResult & result) {
      goal_in_flight_ = false;
      reachedGoal(result, target_position);
    };

  RCLCPP_INFO(
    get_logger(), "Sending NavigateToPose goal to (%.2f, %.2f) yaw=%.2f rad",
    target_position.x, target_position.y, yaw);
  goal_in_flight_ = true;
  nav_client_->async_send_goal(goal_msg, send_goal_options);
}

void Explore::reachedGoal(
  const GoalHandleNav::WrappedResult & result,
  const geometry_msgs::msg::Point & frontier_goal)
{
  current_goal_handle_ = nullptr;
  goal_in_flight_ = false;

  if (result.code == rclcpp_action::ResultCode::ABORTED) {
    RCLCPP_WARN(
      get_logger(),
      "NavigateToPose goal (%.2f, %.2f) was ABORTED; adding to blacklist.",
      frontier_goal.x, frontier_goal.y);
    frontier_blacklist_.push_back(frontier_goal);
  } else if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    geometry_msgs::msg::Pose current_pose;
    double dist_moved = 1.0;
    if (costmap_client_->getRobotPose(current_pose)) {
      dist_moved = euclideanDistance(goal_start_robot_pos_, current_pose.position);
    }
    RCLCPP_INFO(
      get_logger(),
      "NavigateToPose goal (%.2f, %.2f) SUCCEEDED (dist moved: %.2f m).",
      frontier_goal.x, frontier_goal.y, dist_moved);
    if (dist_moved >= 0.15) {
      frontier_blacklist_.push_back(frontier_goal);
      if (save_map_) {
        saveMap();
      }
    }
  } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
    RCLCPP_INFO(
      get_logger(),
      "NavigateToPose goal (%.2f, %.2f) was CANCELED.",
      frontier_goal.x, frontier_goal.y);
  }

  prev_goal_.x = std::numeric_limits<double>::infinity();
  prev_goal_.y = std::numeric_limits<double>::infinity();
  last_progress_ = now();

  // Trigger immediate replan for next frontier
  if (is_exploring_) {
    makePlan();
  }
}

bool Explore::saveMap(const std::string & custom_dir)
{
  std::string target_dir;
  if (!custom_dir.empty()) {
    target_dir = custom_dir;
  } else if (!run_dir_.empty()) {
    target_dir = run_dir_;
  } else {
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    target_dir = data_dir_ + "/" + oss.str();
  }

  std::error_code ec;
  std::filesystem::create_directories(target_dir, ec);
  if (ec) {
    RCLCPP_ERROR(
      get_logger(), "Failed to create directory '%s': %s",
      target_dir.c_str(), ec.message().c_str());
    return false;
  }

  auto map_msg = costmap_client_->getLatestOccupancyGrid();
  if (!map_msg) {
    RCLCPP_WARN(get_logger(), "No occupancy grid map received; cannot save map.");
    return false;
  }

  // Try saving via nav2_map_server::saveMapToFile
  nav2_map_server::SaveParameters params;
  params.map_file_name = target_dir + "/map";
  params.image_format = "pgm";
  params.free_thresh = 0.25;
  params.occupied_thresh = 0.65;
  params.mode = nav2_map_server::MapMode::Trinary;

  bool ok = false;
  try {
    ok = nav2_map_server::saveMapToFile(*map_msg, params);
  } catch (const std::exception & ex) {
    RCLCPP_WARN(get_logger(), "saveMapToFile threw exception: %s", ex.what());
    ok = false;
  }

  std::string pgm_file = target_dir + "/map.pgm";
  std::string yaml_file = target_dir + "/map.yaml";

  if (!ok || !std::filesystem::exists(pgm_file)) {
    // Direct PGM / YAML fallback
    RCLCPP_INFO(get_logger(), "Writing map directly to %s and %s", pgm_file.c_str(), yaml_file.c_str());

    std::ofstream yaml_f(yaml_file);
    yaml_f << "image: map.pgm\n";
    yaml_f << std::fixed << std::setprecision(6);
    yaml_f << "resolution: " << map_msg->info.resolution << "\n";
    yaml_f << "origin: [" << map_msg->info.origin.position.x << ", "
           << map_msg->info.origin.position.y << ", 0.000000]\n";
    yaml_f << "negate: 0\n";
    yaml_f << "occupied_thresh: 0.65\n";
    yaml_f << "free_thresh: 0.25\n";
    yaml_f.close();

    std::ofstream pgm_f(pgm_file, std::ios::binary);
    pgm_f << "P5\n" << map_msg->info.width << " " << map_msg->info.height << "\n255\n";
    for (int y = static_cast<int>(map_msg->info.height) - 1; y >= 0; --y) {
      for (size_t x = 0; x < map_msg->info.width; ++x) {
        size_t idx = static_cast<size_t>(y) * map_msg->info.width + x;
        int8_t val = map_msg->data[idx];
        uint8_t pix = 205;  // Unknown
        if (val == 0) {
          pix = 254;        // Free
        } else if (val >= 65) {
          pix = 0;          // Occupied
        }
        pgm_f.put(static_cast<char>(pix));
      }
    }
    pgm_f.close();
  }

  RCLCPP_INFO(
    get_logger(),
    "Map successfully saved to '%s' and '%s'",
    pgm_file.c_str(), yaml_file.c_str());
  return true;
}

}  // namespace scoutiq_explore
