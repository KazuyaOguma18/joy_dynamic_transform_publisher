#ifndef JOY_INTERACTICE_TRANSFORM_PUBLISHER_TF_UTIL_HPP
#define JOY_INTERACTICE_TRANSFORM_PUBLISHER_TF_UTIL_HPP

#include <geometry_msgs/TransformStamped.h>

geometry_msgs::TransformStamped makeTransformStamped(const std::string& world_frame, const std::string& child_frame, const geometry_msgs::Transform& transform) {
  geometry_msgs::TransformStamped t_stamped;
  t_stamped.header.frame_id = world_frame;
  t_stamped.header.stamp = ros::Time::now();
  t_stamped.child_frame_id = child_frame;
  t_stamped.transform = transform;

  return t_stamped;
}

#endif // JOY_INTERACTICE_TRANSFORM_PUBLISHER_TF_UTIL_HPP