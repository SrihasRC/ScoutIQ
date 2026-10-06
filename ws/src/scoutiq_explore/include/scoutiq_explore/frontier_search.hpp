#ifndef SCOUTIQ_EXPLORE__FRONTIER_SEARCH_HPP_
#define SCOUTIQ_EXPLORE__FRONTIER_SEARCH_HPP_

#include <cstdint>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

namespace scoutiq_explore
{

/**
 * @brief Represents a frontier cluster
 */
struct Frontier
{
  std::uint32_t size{0};
  double min_distance{0.0};
  double cost{0.0};
  double width{0.0};
  geometry_msgs::msg::Point initial;
  geometry_msgs::msg::Point centroid;
  geometry_msgs::msg::Point middle;
  std::vector<geometry_msgs::msg::Point> points;
};

/**
 * @brief Thread-safe implementation of frontier-search task on a costmap
 */
class FrontierSearch
{
public:
  FrontierSearch();

  /**
   * @brief Constructor with parameters
   * @param costmap Pointer to costmap data to search
   * @param potential_scale Weight on distance in cost function
   * @param gain_scale Weight on size in cost function
   * @param min_frontier_size Minimum size of a frontier in meters
   * @param min_frontier_spacing Minimum distance between frontier centroids in meters
   */
  FrontierSearch(
    nav2_costmap_2d::Costmap2D * costmap,
    double potential_scale,
    double gain_scale,
    double min_frontier_size,
    double min_frontier_spacing);

  /**
   * @brief Runs search implementation outward from the start position
   * @param position Initial position to search from
   * @return List of frontiers sorted by cost ascending
   */
  std::vector<Frontier> searchFrom(geometry_msgs::msg::Point position);

  void setCostmap(nav2_costmap_2d::Costmap2D * costmap);

  void setParameters(
    double potential_scale,
    double gain_scale,
    double min_frontier_size,
    double min_frontier_spacing);

protected:
  Frontier buildNewFrontier(
    unsigned int initial_cell,
    unsigned int reference,
    std::vector<bool> & frontier_flag);

  bool isNewFrontierCell(
    unsigned int idx,
    const std::vector<bool> & frontier_flag);

  double frontierCost(const Frontier & frontier);

private:
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  const unsigned char * map_{nullptr};
  unsigned int size_x_{0};
  unsigned int size_y_{0};
  double potential_scale_{1e-3};
  double gain_scale_{1.0};
  double min_frontier_size_{0.5};
  double min_frontier_spacing_{2.0};
};

}  // namespace scoutiq_explore

#endif  // SCOUTIQ_EXPLORE__FRONTIER_SEARCH_HPP_
