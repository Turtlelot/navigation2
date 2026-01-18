#ifndef NAV2_CPP_SIMPLE_COMMANDER__LINE_ITERATOR_HPP_
#define NAV2_CPP_SIMPLE_COMMANDER__LINE_ITERATOR_HPP_

#include <cmath>
#include <stdexcept>

namespace nav2_simple_commander
{

/**
 * @class LineIterator
 *
 * @brief LineIterator C++ API for iterating along the points of a given line.
 *
 */
class LineIterator
{
public:
  /**
   * @brief Initialize the LineIterator.
   *
   * @param x0 Abscissa of the initial point
   * @param y0 Ordinate of the initial point
   * @param x1 Abscissa of the final point
   * @param y1 Ordinate of the final point
   * @param step_size Increments' resolution (must be > 0)
   *
   * @throws std::invalid_argument if step_size <= 0
   * @throws std::runtime_error if line has zero length
   */
  LineIterator(double x0, double y0, double x1, double y1, double step_size = 1.0)
  : x0_(x0), y0_(y0), x1_(x1), y1_(y1), x_(x0), y_(y0), step_size_(step_size)
  {
    if (step_size_ <= 0.0) {
      throw std::invalid_argument("step_size must be a positive number");
    }

    // General (non-vertical, non-horizontal) case
    if (x1_ != x0_ && y1_ != y0_) {
      valid_ = true;
      m_ = (y1_ - y0_) / (x1_ - x0_);
      b_ = y1_ - (m_ * x1_);
    }
    // Vertical line
    else if (x1_ == x0_ && y1_ != y0_) {
      valid_ = true;
    }
    // Horizontal line
    else if (y1_ == y0_ && x1_ != x0_) {
      valid_ = true;
      m_ = (y1_ - y0_) / (x1_ - x0_);
      b_ = y1_ - (m_ * x1_);
    }
    // Degenerate line
    else {
      valid_ = false;
      throw std::runtime_error("Line has zero length (All 4 points have same coordinates)");
    }
  }

  /**
   * @brief Check if line is valid.
   */
  bool isValid() const { return valid_; }

  /**
   * @brief Advance to the next point in the line.
   */
  void advance()
  {
    if (!valid_) {
      return;
    }

    if (x1_ > x0_) {
      if (x_ < x1_) {
        x_ = round5(clamp(x_ + step_size_, x0_, x1_));
        y_ = round5(m_ * x_ + b_);
      } else {
        valid_ = false;
      }
    } else if (x1_ < x0_) {
      if (x_ > x1_) {
        x_ = round5(clamp(x_ - step_size_, x1_, x0_));
        y_ = round5(m_ * x_ + b_);
      } else {
        valid_ = false;
      }
    } else {
      if (y1_ > y0_) {
        if (y_ < y1_) {
          y_ = round5(clamp(y_ + step_size_, y0_, y1_));
        } else {
          valid_ = false;
        }
      } else if (y1_ < y0_) {
        if (y_ > y1_) {
          y_ = round5(clamp(y_ - step_size_, y1_, y0_));
        } else {
          valid_ = false;
        }
      } else {
        valid_ = false;
      }
    }
  }

  /// Get the abscissa of the current point.
  double getX() const { return x_; }

  /// Get the ordinate of the current point.
  double getY() const { return y_; }

  /// Get the abscissa of the initial point.
  double getX0() const { return x0_; }

  /// Get the ordinate of the initial point.
  double getY0() const { return y0_; }

  /// Get the abscissa of the final point.
  double getX1() const { return x1_; }

  /// Get the ordinate of the final point.
  double getY1() const { return y1_; }

  /**
   * @brief Get the length of the line.
   */
  double getLineLength() const
  {
    return std::sqrt(std::pow(x1_ - x0_, 2) + std::pow(y1_ - y0_, 2));
  }

private:
  double clamp(double n, double min_n, double max_n) const
  {
    if (n < min_n) {
      return min_n;
    } else if (n > max_n) {
      return max_n;
    } else {
      return n;
    }
  }

  double round5(double v) const { return std::round(v * 100000.0) / 100000.0; }

  double x0_{0.0}, y0_{0.0};
  double x1_{0.0}, y1_{0.0};
  double x_{0.0}, y_{0.0};
  double step_size_{1.0};
  double m_{0.0}, b_{0.0};
  bool valid_{false};
};

}  // namespace nav2_simple_commander

#endif  // NAV2_CPP_SIMPLE_COMMANDER__LINE_ITERATOR_HPP_
