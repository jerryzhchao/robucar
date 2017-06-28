#include "robucar_base/robucar_hardware.h"
#include <boost/assign/list_of.hpp>

#include "communication_p/framepure.hpp"

using namespace robucar_communication;

namespace {
  const float TICKS_PER_METER = 345; //345 based upon the diameter of the wheel including the track, 452 based upon the diamater of the wheel excluding the track. 345 works best inside the lab, 452 works best on the carpet outside
  const uint ENCODER_MIN = 0;
  const uint ENCODER_MAX = 32767;
  const uint PULSES_PER_REVOLUTION = 185;//190; // for speed encoder
}

namespace robucar_base {

  /**
  * Initialize Robucar hardware
  */
  RobucarHardware::RobucarHardware(ros::NodeHandle nh, ros::NodeHandle private_nh, double target_control_freq)
      : nh_(nh),
        private_nh_(private_nh),
        robot_ip_("192.168.1.2"),
        pure_client_(PureClient::getInstance(robot_ip_,60000)),
        notifier_(0),
        pure_target_(2),
        wheel_radius_(0.28)
  {
    int port;
    private_nh_.param<int>("port", port, 60000);

    resetTravelOffset();
    registerControlInterfaces();

    /// TODO replace it by dynamic reconfigure param
    if(!nh_.getParam("wheel_radius", wheel_radius_))
    {
      ROS_WARN("The wheel_radius parameters must be loaded");
    }
    ROS_INFO_STREAM("Robucar hw config wheel_radius_ : "<<wheel_radius_);
#ifndef NDEBUG
    debug_guidance_.open(string("robufast.dat").c_str(), fstream::in|fstream::out|fstream::trunc);
    if(debug_guidance_.is_open())
    {
      debug_guidance_ <<" time, "
                      <<" FL, "
                      <<" FR, "
                      <<" RL, "
                      <<" RR, "
                      <<" FS, "
                      <<" RS, "
                      <<" FL_cmd, "
                      <<" FR_cmd, "
                      <<" RL_cmd, "
                      <<" RR_cmd, "
                      <<" FS_cmd, "
                      <<" RS_cmd, "
                      <<"\n";
    }
#endif
  }

  RobucarHardware::~RobucarHardware()
  {
    // send deactivation command
    CommandPureDrive command;
    uint8_t send_buf[512];
    pure_client_.sendNotification(pure_target_, send_buf, command.toBuffer(send_buf));

    // disable notification
    pure_client_.requestNotificationStop(pure_target_);
  }

  /**
  * Get current encoder travel offsets from MCU and bias future encoder readings against them
  */
  void RobucarHardware::resetTravelOffset()
  {
    if(!notifier_)
    {
      // make a GET  on the service to determine number of drives
      if (!pure_client_.sendRequest(22, Pure::ACTION_GET, pure_target_))
      {
        ROS_WARN("[Vehicle] Cannot send data request to Robucar.");
      }
      ResponsePtr response = pure_client_.findResponse(22, Pure::ACTION_GET, pure_target_);
      if (!response)
      {
        ROS_WARN("[Vehicle] Robucar has not sent answer.");
      }

      // register, no call-back
      notifier_ = pure_client_.requestNotification (pure_target_, 10, 1);
      if (!notifier_)
      {
        ROS_ERROR_STREAM(std::string("[Vehicle] Cannot communicate with Robucar. Try to execute this command to check for network problem: ping ") + robot_ip_);
      }
      //Send null command to be sure that no previous command are kept
      CommandPureDrive command;
      uint8_t send_buf[512];
      pure_client_.sendNotification(pure_target_, send_buf, command.toBuffer(send_buf));
    }
  }


  /**
  * Register interfaces with the RobotHW interface manager, allowing ros_control operation
  */
  void RobucarHardware::registerControlInterfaces()
  {
    ros::V_string joint_names = boost::assign::list_of("front_left_wheel")("front_right_wheel")
                                                      ("rear_left_wheel")("rear_right_wheel");
    for (unsigned int i = 0; i < joint_names.size(); i++)
    {
      hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
                                                              &joints_[i].position, &joints_[i].velocity, &joints_[i].effort);
      joint_state_interface_.registerHandle(joint_state_handle);

      hardware_interface::JointHandle joint_handle(
          joint_state_handle, &joints_[i].velocity_command);
      velocity_joint_interface_.registerHandle(joint_handle);
    }

    ros::V_string steering_joint_names = boost::assign::list_of("front_left_steering_joint")("front_right_steering_joint")
                                                               ("rear_left_steering_joint")("rear_right_steering_joint");

    for (unsigned int i = 0; i < steering_joint_names.size(); i++)
    {
      hardware_interface::JointStateHandle joint_state_handle(steering_joint_names[i],
                                                              &steering_joints_[i].position, &steering_joints_[i].velocity, &steering_joints_[i].effort);
      joint_state_interface_.registerHandle(joint_state_handle);

      hardware_interface::JointHandle joint_handle(
          joint_state_handle, &steering_joints_[i].position_command);
      position_joint_interface_.registerHandle(joint_handle);
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&position_joint_interface_);
  }

  /**
  * Pull latest speed and travel measurements from MCU, and store in joint structure for ros_control
  */
  void RobucarHardware::updateJointsFromHardware(ros::Duration &dt)
  {
    if(notifier_)
    {
      OutboundNotificationPtr message = notifier_->pop(1000);
      if(message)
      {
        if(message->dataLength == 0)
        {
          ROS_WARN("[Vehicle] Robucar answered inconsistent message... ignored.");
        }

        //std::vector<char> data(message->dataBuffer, message->dataBuffer + message->dataLength);
        FramePureDrive frame_pure(message->dataBuffer, message->dataLength);
        frame_pure.decode();
        double angle_front = -frame_pure.getMotorState(FramePureDrive::FS)->getPosition();
        double angle_rear = frame_pure.getMotorState(FramePureDrive::RS)->getPosition(); /// + pour Robucar, - pour Aroco; pour l'angle de braquage arrière

//        angle_front_ -= OFFSET_STEERING_FRONT_;
//        angle_rear_ -= OFFSET_STEERING_REAR_;
        joints_[0].velocity = linearToAngular(frame_pure.getMotorState(FramePureDrive::FL)->getSpeed());
        joints_[1].velocity = linearToAngular(frame_pure.getMotorState(FramePureDrive::FR)->getSpeed());
        joints_[2].velocity = linearToAngular(frame_pure.getMotorState(FramePureDrive::RL)->getSpeed());
        joints_[3].velocity = linearToAngular(frame_pure.getMotorState(FramePureDrive::RR)->getSpeed());

        for(int i=0; i<4; ++i)
        {
          joints_[i].position += joints_[i].velocity*dt.toSec();
        }

        steering_joints_[0].position = angle_front;
        steering_joints_[1].position = angle_front;
        steering_joints_[2].position = angle_rear;
        steering_joints_[3].position = angle_rear;
#ifndef NDEBUG
        if(debug_guidance_.is_open())
        {
          debug_guidance_ << std::setprecision(10)
                          <<" "<<ros::Time::now().toSec()
                          <<" "<<frame_pure.getMotorState(FramePureDrive::FL)->getSpeed()
                          <<" "<<frame_pure.getMotorState(FramePureDrive::FR)->getSpeed()
                          <<" "<<frame_pure.getMotorState(FramePureDrive::RL)->getSpeed()
                          <<" "<<frame_pure.getMotorState(FramePureDrive::RR)->getSpeed()
                          <<" "<<frame_pure.getMotorState(FramePureDrive::FS)->getPosition()
                          <<" "<<frame_pure.getMotorState(FramePureDrive::RS)->getPosition()  
                          <<" "<<frame_pure.getMotorState(FramePureDrive::FL)->getCommandValue()
                          <<" "<<frame_pure.getMotorState(FramePureDrive::FR)->getCommandValue()
                          <<" "<<frame_pure.getMotorState(FramePureDrive::RL)->getCommandValue()
                          <<" "<<frame_pure.getMotorState(FramePureDrive::RR)->getCommandValue()
                          <<" "<<frame_pure.getMotorState(FramePureDrive::FS)->getCommandValue()
                          <<" "<<frame_pure.getMotorState(FramePureDrive::RS)->getCommandValue()
                          <<"\n";
        }
#endif
      }
      else
      {
        ROS_WARN("[Vehicle] Robucar timeout.");
      }
    }
  }

  /**
  * Get latest velocity commands from ros_control via joint structure, and send to MCU
  */
  void RobucarHardware::writeCommandsToHardware(ros::Duration& dt)
  {
    CommandPureDrive drive(DRIVE_ENABLE);
    drive.setSpeedFL(angularToLinear(joints_[0].velocity_command));
    drive.setSpeedFR(angularToLinear(joints_[1].velocity_command));
    drive.setSpeedRL(angularToLinear(joints_[2].velocity_command));
    drive.setSpeedRR(angularToLinear(joints_[3].velocity_command));
    const double front_steering_command = -(steering_joints_[0].position_command + steering_joints_[1].position_command)/2.0;
    const double rear_steering_command = (steering_joints_[2].position_command + steering_joints_[3].position_command)/2.0;
    drive.setFrontSteering(front_steering_command);
    drive.setRearSteering(rear_steering_command);
    //drive.setRearSteering(); // + pour Robucar, - pour Aroco

    float measured_speed = (joints_[0].velocity+joints_[1].velocity+joints_[2].velocity+joints_[3].velocity)/4.0 *wheel_radius_;
    float cmd_speed = (joints_[0].velocity_command+joints_[1].velocity_command+joints_[2].velocity_command+joints_[3].velocity_command)/4.0 *wheel_radius_;
    float front_measured_steering = (steering_joints_[0].position + steering_joints_[1].position)/2.0;
    float rear_measured_steering = (steering_joints_[2].position + steering_joints_[3].position)/2.0;
    if(fabs(measured_speed) < 0.1 && fabs(cmd_speed) < 0.03
       && fabs(front_measured_steering) < 0.03  && fabs(front_steering_command) < 0.03
       && fabs(rear_measured_steering) < 0.03  && fabs(rear_steering_command) < 0.03)
    {
      drive.setDriveStatus(DRIVE_DISABLE);
    }
    uint8_t buffer[512];
    // Drive (4 drive motors)
    size_t len = drive.toBuffer(buffer);
    if(pure_client_.sendNotification(pure_target_, static_cast<uint8_t*>(buffer), len)<=0)
    {
      ROS_WARN("[Vehicle] Cannot send message to Robucar through the pure client");
    }
  }

  /**
  * Robucar reports travel in metres, need radians for ros_control RobotHW
  */
  double RobucarHardware::linearToAngular(const double &travel) const
  {
    return travel / wheel_radius_;
  }

  /**
  * RobotHW provides velocity command in rad/s, Robucar needs m/s,
  */
  double RobucarHardware::angularToLinear(const double &angle) const
  {
    return angle * wheel_radius_;
  }


}  // namespace robucar_base
