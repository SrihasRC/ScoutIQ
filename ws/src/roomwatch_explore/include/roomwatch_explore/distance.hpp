#ifndef ROOMWATCH_EXPLORE__DISTANCE_HPP_
#define ROOMWATCH_EXPLORE__DISTANCE_HPP_

#include <cmath>
#include <geometry_msgs/msg/point.hpp>

namespace roomwatch_explore
{

inline double euclideanDistance(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2)
{
  return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

}  // namespace roomwatch_explore

#endif  // ROOMWATCH_EXPLORE__DISTANCE_HPP_
