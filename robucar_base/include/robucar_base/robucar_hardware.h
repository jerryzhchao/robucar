#ifndef ROBUCAR_BASE_ROBUCAR_HARDWARE_H
#define ROBUCAR_BASE_ROBUCAR_HARDWARE_H

#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <string>
#include <fstream>

#include "robucar_communication/pure_client.h"

namespace robucar_base
{

  /**
  * Class representing Robucar hardware, allows for ros_control to modify internal state via joint interfaces
  */
  class RobucarHardware : public hardware_interface::RobotHW
  {
  public:
    RobucarHardware(ros::NodeHandle nh, ros::NodeHandle private_nh, double target_control_freq);

    ~RobucarHardware();

    void updateJointsFromHardware();

    void writeCommandsToHardware(ros::Duration &dt);

  private:

    void resetTravelOffset();

    void registerControlInterfaces();

    double linearToAngular(const double &travel) const;

    double angularToLinear(const double &angle) const;

    void limitDifferentialSpeed(double &travel_speed_left, double &travel_speed_right);

    ros::NodeHandle nh_, private_nh_;

    // PURE
    std::string robot_ip_;
    robucar_communication::PureClient& pure_client_;
    robucar_communication::NotificationEntry* notifier_;
    const uint16_t pure_target_;


    // ROS Control interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;

    // ROS Parameters
    double wheel_diameter_;

    /**
    * Joint structure that is hooked to ros_control's InterfaceManager, to allow control via ros_controllers
    */
    struct Joint
    {
      double position;
      double position_offset;
      double velocity;
      double effort;
      double velocity_command;

      Joint() : position(0), velocity(0), effort(0), velocity_command(0) { }
    } joints_[4];

    struct SteeringJoint
    {
      double position;
      double velocity;
      double effort;
      double position_command;

      SteeringJoint() : position(0), velocity(0), effort(0), position_command(0) { }
    } steering_joints_[2];

    std::fstream debug_guidance_;
  };

}  // namespace robucar_base
#endif  // ROBUCAR_BASE_ROBUCAR_HARDWARE_H
