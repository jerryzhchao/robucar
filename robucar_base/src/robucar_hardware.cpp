#include "robucar_base/robucar_hardware.h"
#include <boost/assign/list_of.hpp>

#include "communication_p/framepure.hpp"

using namespace DrRobot_MotionSensorDriver;
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
        pure_target_(2)
  {
    private_nh_.param<double>("wheel_diameter", wheel_diameter_, 0.27); // or 0.3555?
    private_nh_.param<double>("max_accel", max_accel_, 5.0);
    private_nh_.param<double>("max_speed", max_speed_, 2.0);
    private_nh_.param<double>("polling_timeout_", polling_timeout_, 10.0);

    std::string port;
    private_nh_.param<int>("port", port, 60000);

    robot_config_.commMethod = DrRobot_MotionSensorDriver::Network;
    robot_config_.boardType = DrRobot_MotionSensorDriver::Jaguar;
    robot_config_.portNum = 10001;
    strncpy(robot_config_.robotIP, "172.16.51.52", sizeof(robot_config_.robotIP) - 1);
    drrobot_motion_driver_.setDrRobotMotionDriverConfig(&robot_config_);

    pid_controller_left_.init(ros::NodeHandle(private_nh_, "pid_parameters"));
    pid_controller_left_.reset();
    pid_controller_right_.init(ros::NodeHandle(private_nh_, "pid_parameters"));
    pid_controller_right_.reset();

    init_gains_ = pid_controller_left_.getGains();
    resetTravelOffset();
    registerControlInterfaces();

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
        ROS_WARN("[Vehicle] Cannot send data request to Robufast.");
      }
      ResponsePtr response = pure_client_.findResponse(22, Pure::ACTION_GET, pure_target_);
      if (!response)
      {
        ROS_WARN("[Vehicle] Robufast has not sent answer.");
      }

      // register, no call-back
      notifier_ = pure_client_.requestNotification (pure_target_, 10, 1);
      if (!notifier_)
      {
        ROS_ERROR(std::string("[Vehicle] Cannot communicate with Robufast. Try to execute this command to check for network problem: ping ") + robot_ip_);
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
    ros::V_string joint_names = boost::assign::list_of("rear_left_wheel")("rear_right_wheel")("front_left_wheel")
                                ("front_right_wheel");
    for (unsigned int i = 0; i < joint_names.size(); i++)
    {
      hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
                                                              &joints_[i].position, &joints_[i].velocity, &joints_[i].effort);
      joint_state_interface_.registerHandle(joint_state_handle);

      hardware_interface::JointHandle joint_handle(
          joint_state_handle, &joints_[i].velocity_command);
      velocity_joint_interface_.registerHandle(joint_handle);
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);

    ros::V_string steering_joint_names = boost::assign::list_of("front_left_steering_joint")("front_right_steering_joint");
    /// TODO register steering interface
  }

  /**
  * Pull latest speed and travel measurements from MCU, and store in joint structure for ros_control
  */
  void RobucarHardware::updateJointsFromHardware()
  {
    if(notifier_)
    {
      OutboundNotificationPtr message = notifier_->pop(1000);
      if(message)
      {
        //frame datation
        const effibox::date_time acquisition_date = effibox::live_clock::current_time();
        PerceptionDate perception_date(acquisition_date - latency.sensor_latency() - latency.communication_latency(),
                                       latency.uncertainty());

        if(message->dataLength == 0)
        {
          ROS_WARN("[Vehicle] Robufast answered inconsistent message... ignored.");
        }

        std::vector<char> data(message->dataBuffer, message->dataBuffer + message->dataLength);
      }
      else
      {
        ROS_WARN("[Vehicle] Robufast timeout.");
      }
    }


    if(drrobot_motion_driver_.portOpen())
    {
      drrobot_motion_driver_.readMotorSensorData(&motor_sensor_data_);

      // Translate from driver data to ROS data
      //motors used are 3 & 4
      //3 is left motor
      //4 is right motor
      for (uint j = 0 ; j < 4; ++j)
      {
        uint i = j;
        if(j > 1)
          i = j+1;
//        if( j == 3)
//          ROS_DEBUG_STREAM(" motorSensorEncoderPos[i] "<<motor_sensor_data_.motorSensorEncoderPos[j]
//                           <<" motorSensorEncoderVel[i] "<<motor_sensor_data_.motorSensorEncoderVel[j]
//                           <<" motorSensorEncoderDir[i] "<<motor_sensor_data_.motorSensorEncoderDir[j]);
        double delta = double(motor_sensor_data_.motorSensorEncoderPos[i])/double(PULSES_PER_REVOLUTION)*2.0*M_PI;
        if(i == 1 || i == 4)
          delta = -delta;
        delta += - joints_[j].position_offset - joints_[j].position;

        // detect suspiciously large readings, possibly from encoder rollover
        if (std::abs(delta) < 3.0)
        {
          joints_[j].position += delta;
        }
        else
        {
          // suspicious! drop this measurement and update the offset for subsequent readings
          joints_[j].position_offset += delta;
        }

        double velocity = double(motor_sensor_data_.motorSensorEncoderVel[i])/double(PULSES_PER_REVOLUTION)*2.0*M_PI;
        if(motor_sensor_data_.motorSensorEncoderDir[i] == 0)
          velocity = -velocity;

        joints_[j].velocity = velocity ;
        if(j == 2)
          ROS_DEBUG_STREAM(i<<" delta "<<angularToLinear(delta)<<" m velocity "<<angularToLinear(velocity)<<" m/s");
        if(j == 3)
          ROS_DEBUG_STREAM(i<<" delta "<<angularToLinear(delta)<<" m velocity "<<angularToLinear(velocity)<<" m/s");
      }
    }

  }

  /**
  * Get latest velocity commands from ros_control via joint structure, and send to MCU
  */
  void RobucarHardware::writeCommandsToHardware(ros::Duration& dt)
  {
    double error_left = joints_[0].velocity_command - (joints_[0].velocity+joints_[2].velocity)/2;
    double error_right = joints_[1].velocity_command - (joints_[1].velocity+joints_[3].velocity)/2;

    control_toolbox::Pid::Gains new_gains_left = init_gains_;
    control_toolbox::Pid::Gains new_gains_right = init_gains_;
    if(fabs(joints_[0].velocity_command) < 0.1)
    {
      new_gains_left.i_gain_ = 0;
      pid_controller_left_.reset();
    }
    if(fabs(joints_[1].velocity_command) < 0.1)
    {
      new_gains_right.i_gain_ = 0;
      pid_controller_right_.reset();
    }

    if(fabs(error_left) < 0.1)
      new_gains_left.p_gain_ = 0;
    if(fabs(error_right) < 0.1)
      new_gains_right.p_gain_ = 0;

    pid_controller_left_.setGains(new_gains_left);
    pid_controller_right_.setGains(new_gains_right);

    double diff_speed_left = pid_controller_left_.computeCommand(error_left, dt);
    double diff_speed_right = pid_controller_right_.computeCommand(error_right, dt);
    control_toolbox::Pid::Gains gain = pid_controller_left_.getGains();
    ROS_DEBUG_STREAM("diff_speed_left_rear "<<diff_speed_left <<
                    " joints_[0].velocity "<<joints_[0].velocity<<
                    " joints_[0].velocity_command "<<joints_[0].velocity_command<<
                    " error_left "<<error_left<<
                    " i left gain "<<gain.i_gain_);
    diff_speed_left = angularToLinear(diff_speed_left);
    diff_speed_right = angularToLinear(diff_speed_right);

    // roue décollée
    // -1   => 3m/s
    // -0.8 => 2.2m/s
    // -0.5 => 1m/s
    // -0.2 => 0.3m/s
//    double diff_speed_left = angularToLinear(joints_[0].velocity_command);
//    double diff_speed_right = angularToLinear(joints_[1].velocity_command);

    limitDifferentialSpeed(diff_speed_left, diff_speed_right);

    double linear_speed = (diff_speed_left + diff_speed_right) * 0.5;
    double differential_speed = diff_speed_left - diff_speed_right;
    int forwardPWM = -linear_speed * 16384 + 16384;
    int turnPWM = differential_speed * 16384 + 16384;
    if (forwardPWM > ENCODER_MAX) forwardPWM = ENCODER_MAX;
    if (forwardPWM < 0) forwardPWM = 0;
    if (turnPWM > ENCODER_MAX) turnPWM = ENCODER_MAX;
    if (turnPWM < 0) turnPWM = 0;
    drrobot_motion_driver_.sendMotorCtrlAllCmd(PWM,NOCONTROL,NOCONTROL,NOCONTROL,forwardPWM,turnPWM, NOCONTROL);
  }

  /**
  * Scale left and right speed outputs to maintain ros_control's desired trajectory without saturating the outputs
  */
  void RobucarHardware::limitDifferentialSpeed(double &diff_speed_left, double &diff_speed_right)
  {
    double large_speed = std::max(std::abs(diff_speed_left), std::abs(diff_speed_right));
    float max_speed = 3.0;
    if (large_speed > max_speed)
    {
      diff_speed_left *= max_speed / large_speed;
      diff_speed_right *= max_speed / large_speed;
    }
  }

  /**
  * Robucar reports travel in metres, need radians for ros_control RobotHW
  */
  double RobucarHardware::linearToAngular(const double &travel) const
  {
    return travel / wheel_diameter_ * 2;
  }

  /**
  * RobotHW provides velocity command in rad/s, Robucar needs m/s,
  */
  double RobucarHardware::angularToLinear(const double &angle) const
  {
    return angle * wheel_diameter_ / 2;
  }


}  // namespace robucar_base
