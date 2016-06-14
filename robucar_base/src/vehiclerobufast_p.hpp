#ifndef _effibox_vehicle_Robufast_hpp_
#define _effibox_vehicle_Robufast_hpp_

//stl
#include <string>

//boost
#include <boost/shared_ptr.hpp>

//effidence drivers
//#include <effidriver/network/factory.hpp>
#include <effidriver/network/asio/parameter.hpp>

//effibox
#include <effibox/recorder/rawrecorder.hpp>

//local
#include "vehicle/vehicle/abstractvehicle_p.hpp"
#include "communication_p/all.hpp"

namespace effibox {
namespace vehicle {

//==============================================================================
//Specific Robufast vehicle parameter
struct VehicleRobufastParameter: public AbstractVehicleParameter
{
  VehicleRobufastParameter(std::string const & ipAddress,
                           unsigned short ipPort)
    :AbstractVehicleParameter(ipAddress, GetID(), RecorderPtr(new RawRecorder)),
    networkParameter(ipAddress, ipPort, effibox::milliseconds(3000), true){}

  static std::string const & GetID(){static std::string s("vehicle/Robufast"); return s;}

  const effidriver::network::AsioParameter networkParameter;
};
//=============================================================================

class Robufast : public AbstractVehicle
{
public:
  Robufast(AbstractVehicleParameterPtr const& params);
  virtual ~Robufast();

  //methods for live communication
  virtual void start();
  virtual void stop();
  virtual FramePtr receive(latencies::LatencyValues const & latency =
                           latencies::LatencyValues());
  robufast_communication::CommandPureDrive process_command(double speed, double front_steering, double rear_steering);
  virtual void send(const FramePtr & frame);
  virtual void lockOffThread();

  //method called in live and replay mode when raw recorder is used
  virtual void decodeRawFrame(effibox::FrameRawPtr const& frameraw,
                              std::list<FramePtr> & outFramesList,
                              latencies::LatencyValues const & latency);

  virtual void getPendingFrames(std::list<FramePtr> & frames);

private:
  //vehicle parameters
  boost::shared_ptr<VehicleRobufastParameter const> p_;
  boost::mutex send_mutex_;

  // PURE
  robufast_communication::PureClient& pure_client_;
  robufast_communication::NotificationEntry* notifier_;
  const uint16_t pure_target_;

  //Command
  double angle_front_, angle_rear_; // Mesure orientation train avant, arriere
  float measured_speed_;

};
//=============================================================================

}
}
#endif
