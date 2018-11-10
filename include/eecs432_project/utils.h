#include <list>

#include <geometry_msgs/Twist.h>
#include <opencv2/core/types.hpp>

namespace utils {

// If either the x or y component is greater than the threshold, return the
// input object. Otherwise, return zeroes.
cv::Point3f deadbandXY(const cv::Point3f& input, const float threshold);


// Simple class for tracking and serving a moving average, given input
// data and a maximum number of elements to use for the average.
class MovingAverageFilter {
 public:
  MovingAverageFilter(const size_t max_elements = 5);
  geometry_msgs::Twist average() const;
  geometry_msgs::Twist add(const geometry_msgs::Twist& input_twist);
  
 private:
  // Max number of elements to use for moving average calculation.
  size_t max_elements_;
  std::list<geometry_msgs::Twist> elements_;
};
} // namespace utils
