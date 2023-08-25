#ifndef JOY_INTERACTICE_TRANSFORM_PUBLISHER_JOY_UTIL_HPP
#define JOY_INTERACTICE_TRANSFORM_PUBLISHER_JOY_UTIL_HPP

#include <ros/param.h>
#include <algorithm>

class JoyConfig {
 public:
  JoyConfig() {
    namespace rp = ros::param;
    axes_    = rp::param<std::vector<std::string>>("joy/keymap/axes", {});
    buttons_ = rp::param<std::vector<std::string>>("joy/keymap/buttons", {});
  }

  const uint8_t getAxesIndex(const std::string& axes_name) const {
    auto itr = std::find(axes_.begin(), axes_.end(), axes_name);
    return std::distance(axes_.begin(), itr);
  }

  const uint8_t getButtonIndex(const std::string& button_name) const {
    auto itr = std::find(buttons_.begin(), buttons_.end(), button_name);
    return std::distance(buttons_.begin(), itr);
  }

 private:
  std::vector<std::string> axes_, buttons_;
};

#endif // JOY_INTERACTICE_TRANSFORM_PUBLISHER_JOY_UTIL_HPP