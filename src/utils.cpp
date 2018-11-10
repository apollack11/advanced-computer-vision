#include "eecs432_project/utils.h"

#include <cstdlib>
#include <list>

#include <geometry_msgs/Twist.h>
#include <opencv2/core/types.hpp>


namespace utils {


cv::Point3f deadbandXY(const cv::Point3f& input, const float threshold) {
  if (std::abs(input.x) < threshold && std::abs(input.y) < threshold) {
    return cv::Point3f(0.0, 0.0, 0.0);
  }
  return input;
}

MovingAverageFilter::MovingAverageFilter(const size_t max_elements)
  : max_elements_(max_elements) {}

geometry_msgs::Twist MovingAverageFilter::average() const {
  // Protect against divide-by-zero errors if there is no data.
  const size_t num_elements = elements_.size();
  if (num_elements == 0) {
    return geometry_msgs::Twist(); // not much else we can do here
  }

  // Loop over all elements and return the average.
  geometry_msgs::Twist output_twist;
  for (const auto& twist : elements_) {
    output_twist.linear.x += twist.linear.x / num_elements;
    output_twist.linear.y += twist.linear.y / num_elements;
    output_twist.linear.z += twist.linear.z / num_elements;
    output_twist.angular.x += twist.angular.x / num_elements;
    output_twist.angular.y += twist.angular.y / num_elements;
    output_twist.angular.z += twist.angular.z / num_elements;
  }
  return output_twist;
}

geometry_msgs::Twist
    MovingAverageFilter::add(const geometry_msgs::Twist& input_twist) {
  // If the number of elements is at its max when a new one is added,
  // remove the oldest element to make room for the new one.
  if (elements_.size() >= max_elements_) {
    elements_.pop_back();
  }

  // Add the new element and calculate/return the new average.
  elements_.push_front(input_twist);
  return average();
}


} // namespace utils
