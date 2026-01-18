#ifndef NAV2_CPP_SIMPLE_COMMANDER__FOOTPRINT_COLLISION_CHECKER_HPP_
#define NAV2_CPP_SIMPLE_COMMANDER__FOOTPRINT_COLLISION_CHECKER_HPP_

// Copyright 2021 Samsung Research America
// Copyright 2022 Afif Swaidan
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file footprint_collision_checker.hpp
 *
 * This is a C++ API for a Footprint Collision Checker.
 *
 * It provides the needed methods to manipulate the coordinates
 * and calculate the cost of a Footprint.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <memory>
#include <stdexcept>

#include "nav2_cpp_simple_commander/costmap_2d.hpp"
#include "nav2_cpp_simple_commander/line_iterator.hpp"

namespace nav2_simple_commander
{

// Cost values (same as Python example)
static constexpr uint8_t NO_INFORMATION = 255;
static constexpr uint8_t LETHAL_OBSTACLE = 254;
static constexpr uint8_t INSCRIBED_INFLATED_OBSTACLE = 253;
static constexpr uint8_t MAX_NON_OBSTACLE = 252;
static constexpr uint8_t FREE_SPACE = 0;

/**
 * @class FootprintCollisionChecker
 *
 * @brief FootprintCollisionChecker Class for getting the cost
 * and checking the collisions of a Footprint
 */
class FootprintCollisionChecker
{
public:
  /**
   * @brief Initialize the FootprintCollisionChecker Object.
   */
  FootprintCollisionChecker() = default;

  /**
   * @brief Specify which costmap to use.
   *
   * @param costmap Costmap2D instance to use in the object's methods
   */
  void setCostmap(const std::shared_ptr<Costmap2D> & costmap) { costmap_ = costmap; }

  /**
   * @brief Iterate over all the points in a footprint and check for collision.
   *
   * @param footprint The footprint to calculate the collision cost for
   * @return LETHAL_OBSTACLE if collision was found,
   *         otherwise the maximum cost found in the footprint points
   */
  double footprintCost(const geometry_msgs::msg::Polygon & footprint)
  {
    if (!costmap_) {
      throw std::runtime_error(
        "Costmap not specified, use setCostmap() to specify the costmap first");
    }

    if (footprint.points.empty()) {
      return LETHAL_OBSTACLE;
    }

    double footprint_cost = 0.0;

    auto start =
      costmap_->worldToMapValidated(footprint.points.front().x, footprint.points.front().y);

    if (start.first < 0 || start.second < 0) {
      return LETHAL_OBSTACLE;
    }

    int x0 = start.first;
    int y0 = start.second;
    int xstart = x0;
    int ystart = y0;

    int x1 = 0;
    int y1 = 0;

    for (size_t i = 0; i < footprint.points.size() - 1; ++i) {
      auto p = costmap_->worldToMapValidated(footprint.points[i + 1].x, footprint.points[i + 1].y);

      if (p.first < 0 || p.second < 0) {
        return LETHAL_OBSTACLE;
      }

      x1 = p.first;
      y1 = p.second;

      footprint_cost = std::max(lineCost(x0, x1, y0, y1), footprint_cost);

      x0 = x1;
      y0 = y1;

      if (footprint_cost == LETHAL_OBSTACLE) {
        return footprint_cost;
      }
    }

    return std::max(lineCost(xstart, x1, ystart, y1), footprint_cost);
  }

  /**
   * @brief Iterate over all the points along a line and check for collision.
   *
   * @param x0 Abscissa of the initial point in map coordinates
   * @param y0 Ordinate of the initial point in map coordinates
   * @param x1 Abscissa of the final point in map coordinates
   * @param y1 Ordinate of the final point in map coordinates
   * @param step_size Increments' resolution
   *
   * @return LETHAL_OBSTACLE if collision was found,
   *         otherwise the maximum cost found in the line points
   */
  double lineCost(double x0, double x1, double y0, double y1, double step_size = 0.5)
  {
    if (!costmap_) {
      throw std::runtime_error(
        "Costmap not specified, use setCostmap() to specify the costmap first");
    }

    double line_cost = 0.0;

    LineIterator it(x0, y0, x1, y1, step_size);

    while (it.isValid()) {
      uint8_t point_cost = costmap_->getCostXY(
        static_cast<unsigned int>(it.getX()), static_cast<unsigned int>(it.getY()));

      if (point_cost == LETHAL_OBSTACLE) {
        return point_cost;
      }

      line_cost = std::max(line_cost, static_cast<double>(point_cost));

      it.advance();
    }

    return line_cost;
  }

  /**
   * @brief Get the cost of a footprint at a specific Pose in map coordinates.
   *
   * @param x map coordinate X
   * @param y map coordinate Y
   * @param theta absolute rotation angle of the footprint
   * @param footprint the footprint to calculate its cost at the given Pose
   *
   * @return LETHAL_OBSTACLE if collision was found,
   *         otherwise the maximum cost found in the footprint points
   */
  double footprintCostAtPose(
    double x, double y, double theta, const geometry_msgs::msg::Polygon & footprint)
  {
    geometry_msgs::msg::Polygon oriented_footprint;

    const double cos_th = std::cos(theta);
    const double sin_th = std::sin(theta);

    for (const auto & pt : footprint.points) {
      geometry_msgs::msg::Point32 new_pt;
      new_pt.x = x + (pt.x * cos_th - pt.y * sin_th);
      new_pt.y = y + (pt.x * sin_th + pt.y * cos_th);
      oriented_footprint.points.push_back(new_pt);
    }

    return footprintCost(oriented_footprint);
  }

private:
  std::shared_ptr<Costmap2D> costmap_{nullptr};
};

}  // namespace nav2_simple_commander

#endif  // NAV2_CPP_SIMPLE_COMMANDER__FOOTPRINT_COLLISION_CHECKER_HPP_
