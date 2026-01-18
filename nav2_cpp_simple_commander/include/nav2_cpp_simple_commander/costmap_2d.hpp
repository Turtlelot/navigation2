#ifndef NAV2_CPP_SIMPLE_COMMANDER__COSTMAP_2D_HPP_
#define NAV2_CPP_SIMPLE_COMMANDER__COSTMAP_2D_HPP_

#include <cmath>
#include <cstdint>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/time.hpp>
#include <stdexcept>
#include <vector>

namespace nav2_simple_commander
{

/**
 * @class Costmap2D
 *
 * @brief Costmap C++ API for OccupancyGrids to populate from published messages
 *
 * This is a direct C++ equivalent of the Python PyCostmap2D class.
 */
class Costmap2D
{
public:
  /**
   * @brief Initialize costmap2D.
   *
   * @param occupancy_map 2D OccupancyGrid map
   */
  explicit Costmap2D(const nav_msgs::msg::OccupancyGrid & occupancy_map)
  {
    size_x_ = occupancy_map.info.width;
    size_y_ = occupancy_map.info.height;
    resolution_ = occupancy_map.info.resolution;
    origin_x_ = occupancy_map.info.origin.position.x;
    origin_y_ = occupancy_map.info.origin.position.y;
    global_frame_id_ = occupancy_map.header.frame_id;
    costmap_timestamp_ = occupancy_map.header.stamp;

    // Copy costmap data
    costmap_.resize(occupancy_map.data.size());
    for (size_t i = 0; i < occupancy_map.data.size(); ++i) {
      costmap_[i] = static_cast<uint8_t>(occupancy_map.data[i]);
    }
  }

  /// Get map width in cells.
  unsigned int getSizeInCellsX() const { return size_x_; }

  /// Get map height in cells.
  unsigned int getSizeInCellsY() const { return size_y_; }

  /// Get x axis map size in meters.
  double getSizeInMetersX() const
  {
    return (static_cast<double>(size_x_) - 1.0 + 0.5) * resolution_;
  }

  /// Get y axis map size in meters.
  double getSizeInMetersY() const
  {
    return (static_cast<double>(size_y_) - 1.0 + 0.5) * resolution_;
  }

  /// Get the origin x axis of the map [m].
  double getOriginX() const { return origin_x_; }

  /// Get the origin y axis of the map [m].
  double getOriginY() const { return origin_y_; }

  /// Get map resolution [m/cell].
  double getResolution() const { return resolution_; }

  /// Get global frame_id.
  const std::string & getGlobalFrameID() const { return global_frame_id_; }

  /// Get costmap timestamp.
  const rclcpp::Time & getCostmapTimestamp() const { return costmap_timestamp_; }

  /**
   * @brief Get the cost of a cell using map coordinates XY.
   */
  uint8_t getCostXY(unsigned int mx, unsigned int my) const { return costmap_[getIndex(mx, my)]; }

  /**
   * @brief Get the cost of a cell using an index.
   */
  uint8_t getCostIdx(unsigned int index) const { return costmap_[index]; }

  /**
   * @brief Set the cost of a cell using map coordinates XY.
   */
  void setCost(unsigned int mx, unsigned int my, uint8_t cost)
  {
    costmap_[getIndex(mx, my)] = cost;
  }

  /**
   * @brief Get the world coordinate XY using map coordinate XY.
   */
  std::pair<double, double> mapToWorld(unsigned int mx, unsigned int my) const
  {
    double wx = origin_x_ + (static_cast<double>(mx) + 0.5) * resolution_;
    double wy = origin_y_ + (static_cast<double>(my) + 0.5) * resolution_;
    return {wx, wy};
  }

  /**
   * @brief Get the map coordinate XY using world coordinate XY.
   *
   * @return {mx, my} if valid, {-1, -1} if invalid
   */
  std::pair<int, int> worldToMapValidated(double wx, double wy) const
  {
    if (wx < origin_x_ || wy < origin_y_) {
      return {-1, -1};
    }

    int mx = static_cast<int>((wx - origin_x_) / resolution_);
    int my = static_cast<int>((wy - origin_y_) / resolution_);

    if (mx < static_cast<int>(size_x_) && my < static_cast<int>(size_y_)) {
      return {mx, my};
    }

    return {-1, -1};
  }

  /**
   * @brief Get the index of the cell using map coordinate XY.
   */
  inline unsigned int getIndex(unsigned int mx, unsigned int my) const { return my * size_x_ + mx; }

private:
  unsigned int size_x_{0};
  unsigned int size_y_{0};
  double resolution_{0.0};
  double origin_x_{0.0};
  double origin_y_{0.0};

  std::string global_frame_id_;
  rclcpp::Time costmap_timestamp_;

  std::vector<uint8_t> costmap_;
};

}  // namespace nav2_simple_commander

#endif  // NAV2_CPP_SIMPLE_COMMANDER__COSTMAP_2D_HPP_
