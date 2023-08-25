#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <joy_interactive_transform_publisher/joy_util.hpp>
#include <joy_interactive_transform_publisher/tf_util.hpp>

class JoyInteractiveTransformPublisher {
 public:
  JoyInteractiveTransformPublisher() : joy_config_(JoyConfig()) {
    ros::NodeHandle nh;
    joy_sub_ = nh.subscribe<sensor_msgs::Joy>("joy", 1, &JoyInteractiveTransformPublisher::joyCallback, this);
    tf_pub_.reset(new tf2_ros::StaticTransformBroadcaster());


    namespace rp = ros::param;
    namespace rn = ros::names;
    world_frame_ = rp::param<std::string>("~world_frame", "world");
    child_frame_ = rp::param<std::string>("~child_frame", "child");

    yaw_scale_ = rp::param<double>("~yaw_scale", 1.);
    pitch_scale_ = rp::param<double>("~pitch_scale", 1.);

    current_pose_ = Eigen::Isometry3d();

    timer_ = nh.createTimer(ros::Duration(0.1), &JoyInteractiveTransformPublisher::timerCallback, this);
  }

  void joyCallback(const sensor_msgs::JoyConstPtr& joy) {
    // subtract right stick input
    double rsh, rsv;
    rsh = (double)joy->axes[joy_config_.getAxesIndex("rsh")];
    rsv = (double)joy->axes[joy_config_.getAxesIndex("rsv")];


    last_rsh_ = rsh;
    last_rsv_ = rsv;
  }

  void timerCallback(const ros::TimerEvent& e) {
    (void)e;

    ROS_INFO_STREAM("rsh = " << last_rsh_ << ", rsv = " << last_rsv_);

    // if the input from right horizontal stick, rotate along the yaw axis from current pose
    current_pose_.linear() *= (Eigen::AngleAxisd(last_rsh_ * yaw_scale_, Eigen::Vector3d::UnitZ()) * 
                               Eigen::AngleAxisd(last_rsv_ * pitch_scale_, Eigen::Vector3d::UnitY())).matrix();
    // if the input from right vertical stick, rotate along the pitch axis from current pose
    // send transfrom
    tf_pub_->sendTransform(makeTransformStamped(world_frame_, child_frame_, tf2::eigenToTransform(current_pose_).transform));
  }

 private:
  ros::Subscriber joy_sub_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_pub_;
  ros::Timer timer_;

  std::string world_frame_, child_frame_;
  double yaw_scale_, pitch_scale_;

  Eigen::Isometry3d current_pose_;
  double last_rsh_, last_rsv_;

  JoyConfig joy_config_;
};

int main (int argc, char** argv) {
  ros::init(argc, argv, "joy_interactive_transform_publisher");
  JoyInteractiveTransformPublisher node;
  ros::spin();
  return 0;
}