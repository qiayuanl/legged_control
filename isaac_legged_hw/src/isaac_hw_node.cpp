#include "isaac_legged_hw/IsaacHW.h"
#include <legged_hw/LeggedHWLoop.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "isaac_legged_hw");
  ros::NodeHandle nh;
  ros::NodeHandle robotHwNh("~");

  ros::AsyncSpinner spinner(3);
  spinner.start();

  try {
    std::shared_ptr<legged::IsaacHW> isaacHw = std::make_shared<legged::IsaacHW>();

    if (!isaacHw->init(nh, robotHwNh)) {
      ROS_FATAL("Failed to initialize the Isaac hardware interface.");
      return 1;
    }

    legged::LeggedHWLoop controlLoop(nh, isaacHw);

    ros::waitForShutdown();
  } catch (const ros::Exception& e) {
    ROS_FATAL_STREAM("Error in the Isaac hardware interface:\n"
                     << "\t" << e.what());
    return 1;
  }

  return 0;
}
