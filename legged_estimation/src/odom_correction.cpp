//
// Created by qiayuan on 2022/8/31.
//

#include "legged_estimation/odom_correction.h"

namespace legged
{
OdomCorrection::OdomCorrection() : tf_listener_(tf_buffer_)
{
  ros::NodeHandle nh = ros::NodeHandle();
  ros::NodeHandle pnh = ros::NodeHandle("~");

  pnh.param("frame_odom", frame_odom_, std::string("odom_corrected"));
  pnh.param("frame_guess", frame_guess_, std::string("odom"));
  sub_ = nh.subscribe<nav_msgs::Odometry>("/tracking_camera/odom/sample", 10, &OdomCorrection::callback, this);
}

void OdomCorrection::callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  tf2::Transform guess2sensor;
  try
  {
    geometry_msgs::TransformStamped tf_msg;
    tf_msg = tf_buffer_.lookupTransform(frame_guess_, msg->child_frame_id, msg->header.stamp, ros::Duration(0.05));
    tf2::fromMsg(tf_msg.transform, guess2sensor);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }

  tf2::Transform world2sensor;
  world2sensor.setOrigin(tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
  world2sensor.setRotation(tf2::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                                           msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));

  if (isnan(world2odom_.getRotation().length()))
    world2odom_ = world2sensor * guess2sensor.inverse();

  tf2::Transform odom2guess = world2odom_.inverse() * world2sensor * guess2sensor.inverse();
  //  odom2guess.setRotation(tf2::Quaternion::getIdentity());

  geometry_msgs::TransformStamped odom2guess_msg;
  odom2guess_msg.header.frame_id = frame_odom_;
  odom2guess_msg.header.stamp = msg->header.stamp;
  odom2guess_msg.child_frame_id = frame_guess_;
  odom2guess_msg.transform = tf2::toMsg(odom2guess);
  tf_broadcaster_.sendTransform(odom2guess_msg);
}

}  // namespace legged

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "odom_correction");
  legged::OdomCorrection odom_correction;
  ros::spin();
  // Successful exit
  return 0;
}
