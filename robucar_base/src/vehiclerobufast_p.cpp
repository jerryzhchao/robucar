#include "vehiclerobufast_p.hpp"

//boost
#include <boost/shared_ptr.hpp>

//effibox
#include <effibox/clock/clock.hpp>
#include <effibox/clock/date_time.hpp>
#include <effibox/clock/perceptiondate.hpp>
#include <effibox/interface/frameerror.hpp>
#include <effibox/interface/frameraw.hpp>

//local
#include "vehicle/frame/fromVehicle/all.hpp"
#include "vehicle/frame/toVehicle/all.hpp"
#include "vehicle/event.hpp"
#include "vehiclerobufast_configuration.hpp"

namespace effibox {
namespace vehicle {

using namespace robufast_communication;
using namespace robufast;

//=============================================================================
//Factory Reference
static bool const ref = FactoryAbstractVehicle::Instance().reference<Robufast>(VehicleRobufastParameter::GetID());
//=============================================================================

Robufast::Robufast(AbstractVehicleParameterPtr const & params)
  :p_(boost::static_pointer_cast<const VehicleRobufastParameter>(params)),
    send_mutex_(),
    pure_client_(PureClient::getInstance(p_->networkParameter.ip_address.to_string(), p_->networkParameter.port)),
    notifier_(0),
    pure_target_(2),
    angle_front_(0.0),
    angle_rear_(0.0),
	measured_speed_(0.0)
{
}
//=============================================================================

Robufast::~Robufast()
{
}
//=============================================================================

void Robufast::start()
{
  if(!notifier_)
  {
    // make a GET  on the service to determine number of drives
    if (!pure_client_.sendRequest(22, Pure::ACTION_GET, pure_target_))
    {
      EFFIBOX_WARNING_MESSAGE("[Vehicle] Cannot send data request to Robufast.");
    }
    ResponsePtr response = pure_client_.findResponse(22, Pure::ACTION_GET, pure_target_);
    if (!response)
    {
      EFFIBOX_WARNING_MESSAGE("[Vehicle] Robufast has not sent answer.");
    }

    // register, no call-back
    notifier_ = pure_client_.requestNotification (pure_target_, 10, 1);
    if (!notifier_)
    {
      EFFIBOX_ERROR_MESSAGE(std::string("[Vehicle] Cannot communicate with Robufast. Try to execute this command to check for network problem: ping ") + p_->networkParameter.ip_address.to_string());
    }
    //Send null command to be sure that no previous command are kept
    CommandPureDrive command;
    uint8_t send_buf[512];
    pure_client_.sendNotification(pure_target_, send_buf, command.toBuffer(send_buf));
  }
}
//=============================================================================

void Robufast::stop()
{
  // send deactivation command
  CommandPureDrive command;
  uint8_t send_buf[512];
  pure_client_.sendNotification(pure_target_, send_buf, command.toBuffer(send_buf));

  // disable notification
  pure_client_.requestNotificationStop(pure_target_);
}
//=============================================================================

FramePtr Robufast::receive(latencies::LatencyValues const & latency)
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
        EFFIBOX_WARNING_MESSAGE("[Vehicle] Robufast answered inconsistent message... ignored.");
        return FramePtr();
      }

      std::vector<char> data(message->dataBuffer, message->dataBuffer + message->dataLength);
      return FrameRawPtr(new FrameRaw(acquisition_date, perception_date, data));
    }
    else
    {
      EFFIBOX_WARNING_MESSAGE("[Vehicle] Robufast timeout.");
    }
  }
  return FramePtr();
}
//=============================================================================

void Robufast::lockOffThread()
{
}
//=============================================================================

CommandPureDrive Robufast::process_command(double speed, double front_steering, double rear_steering = 0)
{
  float velocities[4];
  double dThetaDesire;

  if(speed > MAX_SPEED)  speed =  MAX_SPEED;
  if(speed <-MAX_SPEED)  speed = -MAX_SPEED;

  dThetaDesire = speed * (tan(angle_front_) -tan(angle_rear_))/ WHEELBASE;

  // Limites trains AVANT et ARRIERE
  if(front_steering> MAX_STEERING_ANGLE) front_steering =  MAX_STEERING_ANGLE;
  if(front_steering<-MAX_STEERING_ANGLE) front_steering = -MAX_STEERING_ANGLE;
  if(rear_steering> MAX_STEERING_ANGLE) rear_steering =  MAX_STEERING_ANGLE;
  if(rear_steering<-MAX_STEERING_ANGLE) rear_steering = -MAX_STEERING_ANGLE;

  CommandPureDrive drive(DRIVE_ENABLE);
  if (fabs(speed) < 0.1 )
  {
    velocities[0] = velocities[1] = velocities[2] = velocities[3] = 0;
  }
  else
  {
    velocities[0]=speed + dThetaDesire*TRACK/2 ; // Roue arriere
    velocities[1]=speed - dThetaDesire*TRACK/2 ; // Roue arriere
    velocities[2]=(fabs(speed)/speed) *
        sqrt((pow((speed + dThetaDesire*TRACK/2),2)+pow(WHEELBASE*dThetaDesire,2))) ; // Roue Avant
    velocities[3]=(fabs(speed)/speed) *
        sqrt((pow((speed - dThetaDesire*TRACK/2),2)+pow(WHEELBASE*dThetaDesire,2))) ; // Roue Avant
  }
  // Si la vitesse est inférieure à une vitesse faible, il vaut mieux désactiver les drives pour un arrêt plus rapide et fiable
  if(fabs(measured_speed_) < 0.1 && fabs(speed) < 0.01
     && fabs(angle_front_) < (0.03 - OFFSET_STEERING_FRONT_) && fabs(front_steering) < 0.01
     && fabs(angle_rear_) < (0.03 - OFFSET_STEERING_REAR_) && fabs(rear_steering) < 0.01)
  {
    drive.setDriveStatus(DRIVE_DISABLE);
  }
  //std::cout<<"speed : "<<speed<<"; dThetaDesire : "<<dThetaDesire<<"; RL : "<<velocities[1]<<"; RR : "<<velocities[0]<<std::endl;

  drive.setSpeedFL(velocities[3]);
  drive.setSpeedFR(velocities[2]);
  drive.setSpeedRL(velocities[1]);
  drive.setSpeedRR(velocities[0]);

  front_steering += OFFSET_STEERING_FRONT_;
  rear_steering += OFFSET_STEERING_REAR_;

  drive.setFrontSteering(-front_steering);
  drive.setRearSteering(rear_steering); // + pour RobuFAST, - pour Aroco

  return drive;
}
//=============================================================================

void Robufast::send(const FramePtr & frame)
{
  FrameVehicleControl2SteeringAnglePtr f_two_steering = boost::dynamic_pointer_cast<FrameVehicleControl2SteeringAngle>(frame);
  FrameVehicleControlBicycleModelPtr f_one_steering = boost::dynamic_pointer_cast<FrameVehicleControlBicycleModel>(frame);
  FrameVehicleControl2SteeringAngle4WheelSpeedPtr f_four_wheel_speed = boost::dynamic_pointer_cast<FrameVehicleControl2SteeringAngle4WheelSpeed>(frame);

  if(f_two_steering)
  {
    CommandPureDrive drive = process_command(f_two_steering->linearSpeed(),
                                             f_two_steering->frontSteeringAngle(),
                                             f_two_steering->rearSteeringAngle());

    uint8_t buffer[512];
    // Drive (4 drive motors)
    size_t len = drive.toBuffer(buffer);
    boost::mutex::scoped_lock scoped_lock(send_mutex_);
    if(pure_client_.sendNotification(pure_target_, static_cast<uint8_t*>(buffer), len)<=0)
    {
      EFFIBOX_WARNING_MESSAGE("[Vehicle] Cannot send message to Robufast through the pure client");
    }
  }
  else if(f_one_steering)
  {
    CommandPureDrive drive = process_command(f_one_steering->linear_speed(),
                                             f_one_steering->steering_angle());
    uint8_t buffer[512];
    // Drive (4 drive motors)
    size_t len = drive.toBuffer(buffer);
    boost::mutex::scoped_lock scoped_lock(send_mutex_);
    if(pure_client_.sendNotification(pure_target_, static_cast<uint8_t*>(buffer), len)<=0)
    {
      EFFIBOX_WARNING_MESSAGE("[Vehicle] Cannot send message to Robufast through the pure client");
    }
  }
  else if(f_four_wheel_speed)
  {
    CommandPureDrive drive(DRIVE_ENABLE);
    drive.setSpeedFL(f_four_wheel_speed->frontLeftWheelSpeed());
    drive.setSpeedFR(f_four_wheel_speed->frontRightWheelSpeed());
    drive.setSpeedRL(f_four_wheel_speed->rearLeftWheelSpeed());
    drive.setSpeedRR(f_four_wheel_speed->rearRightWheelSpeed());
    drive.setFrontSteering(-f_four_wheel_speed->frontSteeringAngle());
    drive.setRearSteering(f_four_wheel_speed->rearSteeringAngle()); // + pour RobuFAST, - pour Aroco

    uint8_t buffer[512];
    // Drive (4 drive motors)
    size_t len = drive.toBuffer(buffer);
    boost::mutex::scoped_lock scoped_lock(send_mutex_);
    if(pure_client_.sendNotification(pure_target_, static_cast<uint8_t*>(buffer), len)<=0)
    {
      EFFIBOX_WARNING_MESSAGE("[Vehicle] Cannot send message to Robufast through the pure client");
    }
  }
  else
  {
    EFFIBOX_WARNING_MESSAGE("[Vehicle] Robufast can not be controled by this frame type. Please send a FrameVehicleControl2SteeringAngle, FrameVehicleControlBicycleModel or FrameVehicleControl2SteeringAngle4WheelSpeed");
  }
}
//=============================================================================

void Robufast::decodeRawFrame(effibox::FrameRawPtr const& frameraw,
                              std::list<FramePtr> & outFramesList,
                              latencies::LatencyValues const & /*latency*/)
{
  if(frameraw->getBuffer().empty())
    return;

  FramePureDrive frame_pure(frameraw->getAcquisitionDate(),
                            frameraw->getPerceptionDate(),
                            reinterpret_cast<uint8_t const*>(&frameraw->getBuffer().front()),
                            frameraw->getBuffer().size());
  frame_pure.decode();

  //create odometry frame from odometry datas
  FrameOdometryPtr frame_odo = FrameOdometryPtr(new FrameOdometry(frame_pure.getAcquisitionDate(),
                                                                  frame_pure.getPerceptionDate()));
  angle_front_ = -frame_pure.getMotorState(FramePureDrive::FS)->getPosition();
  angle_rear_ = frame_pure.getMotorState(FramePureDrive::RS)->getPosition(); /// + pour RobuFAST, - pour Aroco; pour l'angle de braquage arriÃ¨re

  angle_front_ -= OFFSET_STEERING_FRONT_;
  angle_rear_ -= OFFSET_STEERING_REAR_;
  measured_speed_ = (frame_pure.getMotorState(FramePureDrive::FL)->getSpeed() +
                    frame_pure.getMotorState(FramePureDrive::FR)->getSpeed() +
                    frame_pure.getMotorState(FramePureDrive::RL)->getSpeed() +
                    frame_pure.getMotorState(FramePureDrive::RR)->getSpeed()) / 4;

  frame_odo->addWheelSteering(WheelSteering(frame_pure.getMotorState(FramePureDrive::FL)->getSpeed(),
                                       frame_pure.getMotorState(FramePureDrive::FR)->getSpeed(),
                                       angle_front_));
  frame_odo->addWheelSteering(WheelSteering(frame_pure.getMotorState(FramePureDrive::RL)->getSpeed(),
                                       frame_pure.getMotorState(FramePureDrive::RR)->getSpeed(),
                                       angle_rear_));

  if(frame_odo)
  {
#ifndef NDEBUG
    const effibox::date_time acquisition_date = frame_pure.getAcquisitionDate();
    to_display_++;
    if(to_display_ >= 40)
    {
      printf(" Time; cmFL; spFL; torFL; s; m; cmFR; spFR; torFR; s; m;cmdRL;speRL; torRL; s; m;cmdRR;speRR; torRR; s; m;\n");//cmdFS;posFS; torFS; s; m;\n);
      to_display_ = 0;
    }
    printf("%d; %.2f; %.2f; %.3f; %d; %d; %.2f; %.2f; %.3f; %d; %d; %.2f; %.2f; %.3f; %d; %d; %.2f; %.2f; %.3f; %d; %d; \n" /*%.2f; %.2f; %d; %d; %.2f; %.2f; %d; %d;\n*/,
           static_cast<int>(acquisition_date.total_milliseconds()),
           frame_pure.getMotorState(FramePureDrive::FL)->getCommandValue(),
           frame_pure.getMotorState(FramePureDrive::FL)->getSpeed(),
           frame_pure.getMotorState(FramePureDrive::FL)->getTorque(),
           frame_pure.getMotorState(FramePureDrive::FL)->getStatus(),
           frame_pure.getMotorState(FramePureDrive::FL)->getMode(),
           frame_pure.getMotorState(FramePureDrive::FR)->getCommandValue(),
           frame_pure.getMotorState(FramePureDrive::FR)->getSpeed(),
           frame_pure.getMotorState(FramePureDrive::FR)->getTorque(),
           frame_pure.getMotorState(FramePureDrive::FR)->getStatus(),
           frame_pure.getMotorState(FramePureDrive::FR)->getMode(),
           frame_pure.getMotorState(FramePureDrive::RL)->getCommandValue(),
           frame_pure.getMotorState(FramePureDrive::RL)->getSpeed(),
           frame_pure.getMotorState(FramePureDrive::RL)->getTorque(),
           frame_pure.getMotorState(FramePureDrive::RL)->getStatus(),
           frame_pure.getMotorState(FramePureDrive::RL)->getMode(),
           frame_pure.getMotorState(FramePureDrive::RR)->getCommandValue(),
           frame_pure.getMotorState(FramePureDrive::RR)->getSpeed(),
           frame_pure.getMotorState(FramePureDrive::RR)->getTorque(),
           frame_pure.getMotorState(FramePureDrive::RR)->getStatus(),
           frame_pure.getMotorState(FramePureDrive::RR)->getMode()/*,
           frame_pure.getMotorState(FramePureDrive::FS)->getCommandValue(),
           frame_pure.getMotorState(FramePureDrive::FS)->getPosition(),
           frame_pure.getMotorState(FramePureDrive::FS)->getStatus(),
           frame_pure.getMotorState(FramePureDrive::FS)->getMode(),
           frame_pure.getMotorState(FramePureDrive::RS)->getCommandValue(),
           frame_pure.getMotorState(FramePureDrive::RS)->getPosition(),
           frame_pure.getMotorState(FramePureDrive::RS)->getStatus(),
           frame_pure.getMotorState(FramePureDrive::RS)->getMode()*/);
#endif
    outFramesList.push_back(frame_odo);
  }
}
//=============================================================================

void Robufast::getPendingFrames(std::list<FramePtr> &/*frames*/)
{
}
//=============================================================================

} //namespace vehicle
} //namespace effibox
