#include "scoutiq_explore/frontier_search.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <queue>

#include "nav2_costmap_2d/cost_values.hpp"
#include "scoutiq_explore/costmap_tools.hpp"
#include "scoutiq_explore/distance.hpp"

namespace scoutiq_explore
{

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

FrontierSearch::FrontierSearch()
: costmap_(nullptr),
  map_(nullptr),
  size_x_(0),
  size_y_(0),
  potential_scale_(1e-3),
  gain_scale_(1.0),
  min_frontier_size_(0.5),
  min_frontier_spacing_(2.0)
{
}

FrontierSearch::FrontierSearch(
  nav2_costmap_2d::Costmap2D * costmap,
  double potential_scale,
  double gain_scale,
  double min_frontier_size,
  double min_frontier_spacing)
: costmap_(costmap),
  map_(nullptr),
  size_x_(0),
  size_y_(0),
  potential_scale_(potential_scale),
  gain_scale_(gain_scale),
  min_frontier_size_(min_frontier_size),
  min_frontier_spacing_(min_frontier_spacing)
{
}

void FrontierSearch::setCostmap(nav2_costmap_2d::Costmap2D * costmap)
{
  costmap_ = costmap;
}

void FrontierSearch::setParameters(
  double potential_scale,
  double gain_scale,
  double min_frontier_size,
  double min_frontier_spacing)
{
  potential_scale_ = potential_scale;
  gain_scale_ = gain_scale;
  min_frontier_size_ = min_frontier_size;
  min_frontier_spacing_ = min_frontier_spacing;
}

std::vector<Frontier> FrontierSearch::searchFrom(geometry_msgs::msg::Point position)
{
  std::vector<Frontier> frontier_list;

  if (costmap_ == nullptr) {
    return frontier_list;
  }

  // Sanity check that robot is inside costmap bounds before searching
  unsigned int mx = 0;
  unsigned int my = 0;
  if (!costmap_->worldToMap(position.x, position.y, mx, my)) {
    return frontier_list;
  }

  // Make sure map is consistent and locked for duration of search
  std::lock_guard<std::recursive_mutex> lock(*(costmap_->getMutex()));

  map_ = costmap_->getCharMap();
  size_x_ = costmap_->getSizeInCellsX();
  size_y_ = costmap_->getSizeInCellsY();

  if (size_x_ == 0 || size_y_ == 0 || map_ == nullptr) {
    return frontier_list;
  }

  // Initialize flag arrays to keep track of visited and frontier cells
  std::vector<bool> frontier_flag(size_x_ * size_y_, false);
  std::vector<bool> visited_flag(size_x_ * size_y_, false);

  // Initialize breadth first search
  std::queue<unsigned int> bfs;

  // Find closest clear cell to start search
  unsigned int clear = 0;
  unsigned int pos = costmap_->getIndex(mx, my);
  if (nearestCell(clear, pos, FREE_SPACE, *costmap_)) {
    bfs.push(clear);
  } else {
    bfs.push(pos);
  }
  visited_flag[bfs.front()] = true;

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // Iterate over 4-connected neighbourhood
    for (unsigned int nbr : nhood4(idx, *costmap_)) {
      // Add to queue all free, unvisited cells; descending search if initialized on non-free cell
      if (map_[nbr] <= map_[idx] && !visited_flag[nbr]) {
        visited_flag[nbr] = true;
        bfs.push(nbr);
      } else if (isNewFrontierCell(nbr, frontier_flag)) {
        frontier_flag[nbr] = true;
        Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag);

        if (new_frontier.size * costmap_->getResolution() >= min_frontier_size_) {
          bool too_close = false;
          for (const auto & frontier : frontier_list) {
            if (euclideanDistance(new_frontier.centroid, frontier.centroid) < min_frontier_spacing_) {
              too_close = true;
              break;
            }
          }
          if (!too_close) {
            frontier_list.push_back(new_frontier);
          }
        }
      }
    }
  }

  // Set costs of frontiers
  for (auto & frontier : frontier_list) {
    frontier.cost = frontierCost(frontier);
  }
  std::sort(
    frontier_list.begin(), frontier_list.end(),
    [](const Frontier & f1, const Frontier & f2) { return f1.cost < f2.cost; });

  return frontier_list;
}

Frontier FrontierSearch::buildNewFrontier(
  unsigned int initial_cell,
  unsigned int reference,
  std::vector<bool> & frontier_flag)
{
  Frontier output;
  output.size = 1;
  output.min_distance = std::numeric_limits<double>::infinity();

  // Record initial contact point for frontier
  unsigned int ix = 0;
  unsigned int iy = 0;
  costmap_->indexToCells(initial_cell, ix, iy);
  costmap_->mapToWorld(ix, iy, output.initial.x, output.initial.y);

  // Cache reference position in world coords
  unsigned int rx = 0;
  unsigned int ry = 0;
  double reference_x = 0.0;
  double reference_y = 0.0;
  costmap_->indexToCells(reference, rx, ry);
  costmap_->mapToWorld(rx, ry, reference_x, reference_y);

  // Seed with initial contact point
  output.points.push_back(output.initial);
  output.centroid.x = output.initial.x;
  output.centroid.y = output.initial.y;
  output.min_distance = std::hypot(reference_x - output.initial.x, reference_y - output.initial.y);
  output.middle = output.initial;

  // Push initial gridcell onto queue
  std::queue<unsigned int> bfs;
  bfs.push(initial_cell);

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // Try adding cells in 8-connected neighborhood to frontier
    for (unsigned int nbr : nhood8(idx, *costmap_)) {
      if (isNewFrontierCell(nbr, frontier_flag)) {
        frontier_flag[nbr] = true;
        unsigned int mx = 0;
        unsigned int my = 0;
        double wx = 0.0;
        double wy = 0.0;
        costmap_->indexToCells(nbr, mx, my);
        costmap_->mapToWorld(mx, my, wx, wy);

        geometry_msgs::msg::Point point;
        point.x = wx;
        point.y = wy;
        point.z = 0.0;
        output.points.push_back(point);

        output.size++;
        output.centroid.x += wx;
        output.centroid.y += wy;

        double distance = std::hypot(reference_x - wx, reference_y - wy);
        if (distance < output.min_distance) {
          output.min_distance = distance;
          output.middle.x = wx;
          output.middle.y = wy;
        }

        bfs.push(nbr);
      }
    }
  }

  output.centroid.x /= output.size;
  output.centroid.y /= output.size;
  return output;
}

bool FrontierSearch::isNewFrontierCell(
  unsigned int idx,
  const std::vector<bool> & frontier_flag)
{
  if (map_[idx] != NO_INFORMATION || frontier_flag[idx]) {
    return false;
  }

  for (unsigned int nbr : nhood4(idx, *costmap_)) {
    if (map_[nbr] == FREE_SPACE) {
      return true;
    }
  }

  return false;
}

double FrontierSearch::frontierCost(const Frontier & frontier)
{
  return (potential_scale_ * frontier.min_distance * costmap_->getResolution()) -
         (gain_scale_ * frontier.size * costmap_->getResolution());
}

}  // namespace scoutiq_explore
