// -*- mode:c++; fill-column: 100; -*-

#ifndef VESC_DRIVER_VESC_DRIVER_H_
#define VESC_DRIVER_VESC_DRIVER_H_

#include <string>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <boost/optional.hpp>

#include "vesc_driver/vesc_interface.h"
#include "vesc_driver/vesc_packet.h"

namespace vesc_driver
{

class VescDriver
{
public:

  VescDriver(ros::NodeHandle nh,
             ros::NodeHandle private_nh);

private:
  // interface to the VESC
  VescInterface vesc_;
  void vescPacketCallback(const boost::shared_ptr<VescPacket const>& packet);
  void vescErrorCallback(const std::string& error);

  // limits on VESC commands
  struct CommandLimit
  {
    CommandLimit(const ros::NodeHandle& nh, const std::string& str,
                 const boost::optional<double>& min_lower = boost::optional<double>(),
                 const boost::optional<double>& max_upper = boost::optional<double>());
    double clip(double value);
    std::string name;
    boost::optional<double> lower;
    boost::optional<double> upper;
  };

  CommandLimit duty_cycle_limit_;
  CommandLimit current_limit_;
  CommandLimit brake_limit_;
  CommandLimit speed_limit_;
  CommandLimit position_limit_;
  CommandLimit servo_limit_;

  // ROS services
  ros::Timer timer_;
  std::string frame_id_;
  ros::Publisher state_pub_;
  ros::Publisher feedback_pub_;
  ros::Subscriber duty_cycle_sub_;
  ros::Subscriber current_sub_;
  ros::Subscriber brake_sub_;
  ros::Subscriber speed_sub_;
  ros::Subscriber position_sub_;
  ros::Subscriber servo_sub_;

  // driver modes (possible states)
  typedef enum {
    DRIVER_MODE_INITIALIZING,
    DRIVER_MODE_OPERATING
  } driver_mode_t;

  // control modes (possible states)
  typedef enum {
    CONTROL_MODE_DUTY_CYCLE,
    CONTROL_MODE_CURRENT,
    CONTROL_MODE_BRAKE,
    CONTROL_MODE_SPEED,
    CONTROL_MODE_POSITION
  } control_mode_t;

  // feedback modes (possible states)
  typedef enum {
    FEEDBACK_MODE_DUTY_CYCLE,
    FEEDBACK_MODE_CURRENT,
    FEEDBACK_MODE_SPEED,
    FEEDBACK_MODE_POSITION,
    FEEDBACK_MODE_NONE
  } feedback_mode_t;

  // servo modes (possible states)
  typedef enum {
    SERVO_MODE_ON,
    SERVO_MODE_OFF
  } servo_mode_t;

  // other variables
  driver_mode_t driver_mode_;           ///< driver state machine mode (state)
  control_mode_t control_mode_;
  feedback_mode_t feedback_mode_;
  servo_mode_t servo_mode_;
  int fw_version_major_;                ///< firmware major version reported by vesc
  int fw_version_minor_;                ///< firmware minor version reported by vesc

  // ROS callbacks
  void timerCallback(const ros::TimerEvent& event);
  void dutyCycleCallback(const std_msgs::Float64::ConstPtr& duty_cycle);
  void currentCallback(const std_msgs::Float64::ConstPtr& current);
  void brakeCallback(const std_msgs::Float64::ConstPtr& brake);
  void speedCallback(const std_msgs::Float64::ConstPtr& speed);
  void positionCallback(const std_msgs::Float64::ConstPtr& position);
  void servoCallback(const std_msgs::Float64::ConstPtr& servo);
};

} // namespace vesc_driver

#endif // VESC_DRIVER_VESC_DRIVER_H_
