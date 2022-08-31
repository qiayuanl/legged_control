//
// Created by qiayuan on 2022/8/31.
//
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nav_msgs/Odometry.h>

namespace legged
{
class OdomCorrection
{
public:
  OdomCorrection();

private:
  void callback(const nav_msgs::Odometry::ConstPtr& msg);

  ros::Subscriber sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  tf2::Transform world2odom_;
  std::string frame_odom_, frame_guess_, frame_base_;
};

}  // namespace legged