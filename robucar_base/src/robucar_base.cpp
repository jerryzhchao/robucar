#include "robucar_base/robucar_hardware.h"
#include "controller_manager/controller_manager.h"
#include "ros/callback_queue.h"

/**
* Control loop for Robucar, not realtime safe
*/
void controlLoop(robucar_base::RobucarHardware &robucar,
                 controller_manager::ControllerManager &cm,
                 ros::Time &last_time)
{

  // Calculate monotonic time difference
  ros::Time this_time = ros::Time::now();
  ros::Duration elapsed = this_time - last_time;
  last_time = this_time;

  // Process control loop
  robucar.updateJointsFromHardware();
  cm.update(ros::Time::now(), elapsed);
  robucar.writeCommandsToHardware(elapsed);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "robucar_base");
  ros::NodeHandle nh, private_nh("~");

  double control_frequency;
  private_nh.param<double>("control_frequency", control_frequency, 10.0);

  // Initialize robot hardware and link to controller manager
  robucar_base::RobucarHardware robucar(nh, private_nh, control_frequency);
  controller_manager::ControllerManager cm(&robucar, nh);

  // Setup separate queue and single-threaded spinner to process timer callbacks
  // that interface with Robucar hardware - libhorizon_legacy not threadsafe. This
  // avoids having to lock around hardware access, but precludes realtime safety
  // in the control loop.
  ros::CallbackQueue robucar_queue;
  ros::AsyncSpinner robucar_spinner(1, &robucar_queue);

  ros::Time last_time = ros::Time::now();
  ros::TimerOptions control_timer(
      ros::Duration(1 / control_frequency),
      boost::bind(controlLoop, boost::ref(robucar), boost::ref(cm), boost::ref(last_time)),
      &robucar_queue);
  ros::Timer control_loop = nh.createTimer(control_timer);

  robucar_spinner.start();

  // Process remainder of ROS callbacks separately, mainly ControlManager related
  ros::spin();

  return 0;
}
