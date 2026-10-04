#include <gtest/gtest.h>

#include <geometry_msgs/msg/point.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

#include "roomwatch_explore/costmap_tools.hpp"
#include "roomwatch_explore/distance.hpp"
#include "roomwatch_explore/frontier_search.hpp"

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using roomwatch_explore::euclideanDistance;
using roomwatch_explore::Frontier;
using roomwatch_explore::FrontierSearch;
using roomwatch_explore::nearestCell;
using roomwatch_explore::nhood4;
using roomwatch_explore::nhood8;

TEST(CostmapToolsTest, EuclideanDistance)
{
  geometry_msgs::msg::Point p1;
  p1.x = 0.0;
  p1.y = 0.0;

  geometry_msgs::msg::Point p2;
  p2.x = 3.0;
  p2.y = 4.0;

  EXPECT_DOUBLE_EQ(euclideanDistance(p1, p2), 5.0);
}

TEST(CostmapToolsTest, Neighborhood4And8)
{
  nav2_costmap_2d::Costmap2D costmap(10, 10, 0.1, 0.0, 0.0, FREE_SPACE);

  // Center cell (5, 5) -> idx = 55
  unsigned int idx = costmap.getIndex(5, 5);
  auto n4 = nhood4(idx, costmap);
  EXPECT_EQ(n4.size(), 4u);

  auto n8 = nhood8(idx, costmap);
  EXPECT_EQ(n8.size(), 8u);

  // Corner cell (0, 0)
  auto corner_n4 = nhood4(0, costmap);
  EXPECT_EQ(corner_n4.size(), 2u);

  auto corner_n8 = nhood8(0, costmap);
  EXPECT_EQ(corner_n8.size(), 3u);
}

TEST(CostmapToolsTest, NearestCell)
{
  nav2_costmap_2d::Costmap2D costmap(10, 10, 0.1, 0.0, 0.0, NO_INFORMATION);
  // Set cell (7, 7) as FREE_SPACE
  costmap.setCost(7, 7, FREE_SPACE);

  unsigned int result = 0;
  unsigned int start = costmap.getIndex(2, 2);
  bool found = nearestCell(result, start, FREE_SPACE, costmap);
  EXPECT_TRUE(found);

  unsigned int rx, ry;
  costmap.indexToCells(result, rx, ry);
  EXPECT_EQ(rx, 7u);
  EXPECT_EQ(ry, 7u);
}

TEST(FrontierSearchTest, AllUnknownMapHasNoFrontiers)
{
  // Map of 50x50 unknown cells
  nav2_costmap_2d::Costmap2D costmap(50, 50, 0.05, -1.25, -1.25, NO_INFORMATION);

  FrontierSearch search(&costmap, 1.0, 1.0, 0.2, 0.5);
  geometry_msgs::msg::Point pos;
  pos.x = 0.0;
  pos.y = 0.0;

  auto frontiers = search.searchFrom(pos);
  // Since there is no free space at all, no frontiers can be formed
  EXPECT_TRUE(frontiers.empty());
}

TEST(FrontierSearchTest, FreeSpacePatchDetectsFrontiers)
{
  // 100x100 map with resolution 0.05m -> 5.0m x 5.0m
  // Origin at (-2.5, -2.5)
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, -2.5, -2.5, NO_INFORMATION);

  // Make a square free region in the center: [-0.5, 0.5] in x and y (cells 40 to 60)
  for (unsigned int x = 40; x <= 60; ++x) {
    for (unsigned int y = 40; y <= 60; ++y) {
      costmap.setCost(x, y, FREE_SPACE);
    }
  }

  // Frontier length: square perimeter is ~80 cells. 80 * 0.05m = 4.0m.
  // min_frontier_size = 0.5m, spacing = 1.0m
  FrontierSearch search(&costmap, 5.0, 1.0, 0.5, 1.0);
  geometry_msgs::msg::Point pos;
  pos.x = 0.0;
  pos.y = 0.0;

  auto frontiers = search.searchFrom(pos);
  EXPECT_FALSE(frontiers.empty());

  for (const auto & f : frontiers) {
    EXPECT_GT(f.size, 10u);
    EXPECT_GT(f.points.size(), 0u);
    // Centroid should be within map bounds
    EXPECT_GE(f.centroid.x, -2.5);
    EXPECT_LE(f.centroid.x, 2.5);
    EXPECT_GE(f.centroid.y, -2.5);
    EXPECT_LE(f.centroid.y, 2.5);
  }
}

TEST(FrontierSearchTest, MinFrontierSizeFilter)
{
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, -2.5, -2.5, NO_INFORMATION);

  // Make a small free region: 3x3 cells
  for (unsigned int x = 48; x <= 50; ++x) {
    for (unsigned int y = 48; y <= 50; ++y) {
      costmap.setCost(x, y, FREE_SPACE);
    }
  }

  // Max frontier size here is ~10 cells = 0.5m
  // If min_frontier_size = 2.0m, it should be filtered out
  FrontierSearch search(&costmap, 5.0, 1.0, 2.0, 0.5);
  geometry_msgs::msg::Point pos;
  pos.x = 0.0;
  pos.y = 0.0;

  auto frontiers = search.searchFrom(pos);
  EXPECT_TRUE(frontiers.empty());

  // If min_frontier_size = 0.1m, it should be accepted
  search.setParameters(5.0, 1.0, 0.1, 0.5);
  frontiers = search.searchFrom(pos);
  EXPECT_FALSE(frontiers.empty());
}

TEST(FrontierSearchTest, FrontierCostSorting)
{
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, -2.5, -2.5, NO_INFORMATION);

  // Make a horizontal corridor of free space from x=30 to x=70 at y=50
  for (unsigned int x = 30; x <= 70; ++x) {
    costmap.setCost(x, 49, FREE_SPACE);
    costmap.setCost(x, 50, FREE_SPACE);
    costmap.setCost(x, 51, FREE_SPACE);
  }

  FrontierSearch search(&costmap, 5.0, 1.0, 0.2, 0.5);
  geometry_msgs::msg::Point pos;
  pos.x = 0.0;
  pos.y = 0.0;

  auto frontiers = search.searchFrom(pos);
  ASSERT_GE(frontiers.size(), 1u);

  // Check that frontiers are sorted by cost ascending
  for (size_t i = 1; i < frontiers.size(); ++i) {
    EXPECT_LE(frontiers[i - 1].cost, frontiers[i].cost);
  }
}
