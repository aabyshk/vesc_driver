// -*- mode:c++; fill-column: 100; -*-

#include "vesc_driver/vesc_driver.h"

#include <cassert>
#include <cmath>
#include <sstream>

#include<boost/bind.hpp>
#include<vesc_driver/VescStateStamped.h>
#include<vesc_driver/VescFeedbackStamped.h>

namespace vesc_driver
{

VescDriver::VescDriver(ros::NodeHandle nh, ros::NodeHandle private_nh):
  vesc_(std::string(),
        boost::bind(&VescDriver::vescPacketCallback, this, _1),
        boost::bind(&VescDriver::vescErrorCallback, this, _1)),
  duty_cycle_limit_(private_nh, "duty_cycle", -1.0, 1.0),
  current_limit_(private_nh, "current"),
  brake_limit_(private_nh, "brake"),
  speed_limit_(private_nh, "speed"),
  position_limit_(private_nh, "position"),
  servo_limit_(private_nh, "servo", 0.0, 1.0),
  driver_mode_(DRIVER_MODE_INITIALIZING),
  control_mode_(CONTROL_MODE_DUTY_CYCLE),
  feedback_mode_(FEEDBACK_MODE_DUTY_CYCLE),
  servo_mode_(SERVO_MODE_OFF),
  fw_version_major_(-1),
  fw_version_minor_(-1)
{
  // get vesc serial port address
  std::string port;
  if (!private_nh.getParam("port", port)) {
    ROS_FATAL("VESC communication port parameter required.");
    ros::shutdown();
    return;
  }

  // attempt to connect to the serial port
  try {
    vesc_.connect(port);
  }
  catch (SerialException e) {
    ROS_FATAL("Failed to connect to the VESC, %s.", e.what());
    ros::shutdown();
    return;
  }

  // get motor control mode
  std::string control_mode;
  if (!private_nh.getParam("control_mode", control_mode)) {
    ROS_FATAL("VESC control mode parameter required.");
    ros::shutdown();
    return;
  }

  // create command subscribers based on control mode
  if (control_mode.compare("duty_cycle") == 0) {
      control_mode_ = CONTROL_MODE_DUTY_CYCLE;
      duty_cycle_sub_ = nh.subscribe("commands/motor/duty_cycle", 10, &VescDriver::dutyCycleCallback, this);
    } else if (control_mode.compare("current") == 0) {
      control_mode_ = CONTROL_MODE_CURRENT;
      current_sub_ = nh.subscribe("commands/motor/current", 10, &VescDriver::currentCallback, this);
    } else if (control_mode.compare("brake") == 0) {
      control_mode_ = CONTROL_MODE_BRAKE;
      brake_sub_ = nh.subscribe("commands/motor/brake", 10, &VescDriver::brakeCallback, this);
    } else if (control_mode.compare("speed") == 0) {
      control_mode_ = CONTROL_MODE_SPEED;
      speed_sub_ = nh.subscribe("commands/motor/speed", 10, &VescDriver::speedCallback, this);
    } else if (control_mode.compare("position") == 0) {
      control_mode_ = CONTROL_MODE_POSITION;
      position_sub_ = nh.subscribe("commands/motor/position", 10, &VescDriver::positionCallback, this);
    } else {
      ROS_WARN("Invalid control mode parameter, defaulting to duty cycle");
      control_mode_ = CONTROL_MODE_DUTY_CYCLE;
      duty_cycle_sub_ = nh.subscribe("commands/motor/duty_cycle", 10, &VescDriver::dutyCycleCallback, this);
    }


  // get feedback mode
  std::string feedback_mode;
  if (!private_nh.getParam("feedback_mode", feedback_mode)) {
    ROS_FATAL("VESC feedback mode parameter required.");
    ros::shutdown();
    return;
  }
  // create command subscribers based on control mode
  if (feedback_mode.compare("duty_cycle") == 0) {
      feedback_mode_ = FEEDBACK_MODE_DUTY_CYCLE;
      feedback_pub_ = nh.advertise<vesc_driver::VescFeedbackStamped>("feedback/motor/duty_cycle", 10);
    } else if (feedback_mode.compare("current") == 0) {
      feedback_mode_ = FEEDBACK_MODE_CURRENT;
      feedback_pub_ = nh.advertise<vesc_driver::VescFeedbackStamped>("feedback/motor/current", 10);
    } else if (feedback_mode.compare("speed") == 0) {
      feedback_mode_ = FEEDBACK_MODE_SPEED;
      feedback_pub_ = nh.advertise<vesc_driver::VescFeedbackStamped>("feedback/motor/speed", 10);
    } else if (feedback_mode.compare("pid_position") == 0) {
      feedback_mode_ = FEEDBACK_MODE_PID_POSITION;
      feedback_pub_ = nh.advertise<vesc_driver::VescFeedbackStamped>("feedback/motor/pid_position", 10);
    } else if (feedback_mode.compare("encoder_position") == 0) {
      feedback_mode_ = FEEDBACK_MODE_ENCODER_POSITION;
      feedback_pub_ = nh.advertise<vesc_driver::VescFeedbackStamped>("feedback/motor/encoder_position", 10);
    } else if (feedback_mode.compare("none") == 0) {
      feedback_mode_ = FEEDBACK_MODE_NONE;
    } else {
      ROS_WARN("Invalid feedback mode parameter, defaulting to none");
      feedback_mode_ = FEEDBACK_MODE_NONE;
    }

  // create vesc state (telemetry) publisher
  state_pub_ = nh.advertise<vesc_driver::VescStateStamped>("sensors/core", 10);

  // get servo mode
  std::string servo_mode;
  if (!private_nh.getParam("servo_mode", servo_mode)) {
    ROS_FATAL("VESC servo mode parameter required.");
    ros::shutdown();
    return;
  }
  // subscribe to servo command topic
  if (servo_mode.compare("on") == 0) {
      servo_mode_ = SERVO_MODE_ON;
      servo_sub_ = nh.subscribe("commands/servo/position", 10, &VescDriver::servoCallback, this);
    } else if (servo_mode.compare("off") == 0) {
      servo_mode_ = SERVO_MODE_OFF;
    } else {
      ROS_WARN("Invalid servo mode parameter, defaulting to off");
      servo_mode_ = SERVO_MODE_OFF;
    }

  // get frame_id
  if (!private_nh.getParam("frame_id", frame_id_)) {
    ROS_FATAL("VESC feedback frame id parameter required.");
    ros::shutdown();
    return;
  }

  // create a 50Hz timer, used for state machine & polling VESC telemetry
  timer_ = nh.createTimer(ros::Duration(1.0/50.0), &VescDriver::timerCallback, this);
}

void VescDriver::timerCallback(const ros::TimerEvent& event)
{
  // VESC interface should not unexpectedly disconnect, but test for it anyway
  if (!vesc_.isConnected()) {
    ROS_FATAL("Unexpectedly disconnected from serial port.");
    timer_.stop();
    ros::shutdown();
    return;
  }

  /*
   * Driver state machine, modes:
   *  INITIALIZING - request and wait for vesc version
   *  OPERATING - receiving commands from subscriber topics
   */
  if (driver_mode_ == DRIVER_MODE_INITIALIZING) {
    // request version number, return packet will update the internal version numbers
    vesc_.requestFWVersion();
    if (fw_version_major_ >= 0 && fw_version_minor_ >= 0) {
      ROS_INFO("Connected to VESC with firmware version %d.%d", fw_version_major_, fw_version_minor_);
      driver_mode_ = DRIVER_MODE_OPERATING;
    }
    if (feedback_mode_ == FEEDBACK_MODE_PID_POSITION) {
      vesc_.setDetect(DISP_POS_MODE_PID_POS);
    }
    if (feedback_mode_ == FEEDBACK_MODE_ENCODER_POSITION) {
      vesc_.setDetect(DISP_POS_MODE_ENCODER);
    }
  } else if (driver_mode_ == DRIVER_MODE_OPERATING) {
    // poll for vesc state (telemetry)
    vesc_.requestState();
    if (feedback_mode_ == FEEDBACK_MODE_PID_POSITION || feedback_mode_ == FEEDBACK_MODE_ENCODER_POSITION) {
      vesc_.requestRotorPos();
    }
  } else {
    // unknown mode, how did that happen?
    assert(false && "unknown driver mode");
  }
}

void VescDriver::vescPacketCallback(const boost::shared_ptr<VescPacket const>& packet)
{
  if (packet->name() == "FWVersion") {
    boost::shared_ptr<VescPacketFWVersion const> fw_version =
      boost::dynamic_pointer_cast<VescPacketFWVersion const>(packet);
    // todo: might need lock here
    fw_version_major_ = fw_version->fwMajor();
    fw_version_minor_ = fw_version->fwMinor();
  }

  vesc_driver::VescFeedbackStamped::Ptr feedback_msg(new vesc_driver::VescFeedbackStamped);

  if (packet->name() == "Values") {
    boost::shared_ptr<VescPacketValues const> values =
      boost::dynamic_pointer_cast<VescPacketValues const>(packet);

    vesc_driver::VescStateStamped::Ptr state_msg(new vesc_driver::VescStateStamped);
    state_msg->header.stamp = ros::Time::now();
    state_msg->header.frame_id = frame_id_;
    state_msg->voltage_input = values->v_in();
    state_msg->temperature_pcb = values->temp_pcb();
    state_msg->current_motor = values->current_motor();
    state_msg->current_input = values->current_in();
    state_msg->speed = values->rpm();
    state_msg->duty_cycle = values->duty_now();
    state_msg->charge_drawn = values->amp_hours();
    state_msg->charge_regen = values->amp_hours_charged();
    state_msg->energy_drawn = values->watt_hours();
    state_msg->energy_regen = values->watt_hours_charged();
    state_msg->displacement = values->tachometer();
    state_msg->distance_traveled = values->tachometer_abs();
    state_msg->fault_code = values->fault_code();

    state_pub_.publish(state_msg);

    if (feedback_mode_ == FEEDBACK_MODE_DUTY_CYCLE) {
        feedback_msg->header.stamp = ros::Time::now();
        feedback_msg->header.frame_id = frame_id_;
        feedback_msg->feedback = values->duty_now();
        feedback_pub_.publish(feedback_msg);
      } else if (feedback_mode_ == FEEDBACK_MODE_CURRENT) {
        feedback_msg->header.stamp = ros::Time::now();
        feedback_msg->header.frame_id = frame_id_;
        feedback_msg->feedback = values->current_motor();
        feedback_pub_.publish(feedback_msg);
      } else if (feedback_mode_ == FEEDBACK_MODE_SPEED) {
        feedback_msg->header.stamp = ros::Time::now();
        feedback_msg->header.frame_id = frame_id_;
        feedback_msg->feedback = values->rpm();
        feedback_pub_.publish(feedback_msg);
      }
  }

  if (packet->name() == "RotorPos") {
    if (feedback_mode_ == FEEDBACK_MODE_PID_POSITION || feedback_mode_ == FEEDBACK_MODE_ENCODER_POSITION) {
      boost::shared_ptr<VescPacketRotorPos const> rotor_pos =
        boost::dynamic_pointer_cast<VescPacketRotorPos const>(packet);

        feedback_msg->header.stamp = ros::Time::now();
        feedback_msg->header.frame_id = frame_id_;
        feedback_msg->feedback = rotor_pos->rotor_pos();
        feedback_pub_.publish(feedback_msg);
      }
    }

}

void VescDriver::vescErrorCallback(const std::string& error)
{
  ROS_ERROR("%s", error.c_str());
}

/**
 * @param duty_cycle Commanded VESC duty cycle. Valid range for this driver is -1 to +1. However,
 *                   note that the VESC may impose a more restrictive bounds on the range depending
 *                   on its configuration, e.g. absolute value is between 0.05 and 0.95.
 */
void VescDriver::dutyCycleCallback(const std_msgs::Float64::ConstPtr& duty_cycle)
{
  if (driver_mode_ == DRIVER_MODE_OPERATING && control_mode_ == CONTROL_MODE_DUTY_CYCLE) {
    vesc_.setDutyCycle(duty_cycle_limit_.clip(duty_cycle->data));
  }
}

/**
 * @param current Commanded VESC current in Amps. Any value is accepted by this driver. However,
 *                note that the VESC may impose a more restrictive bounds on the range depending on
 *                its configuration.
 */
void VescDriver::currentCallback(const std_msgs::Float64::ConstPtr& current)
{
  if (driver_mode_ == DRIVER_MODE_OPERATING && control_mode_ == CONTROL_MODE_CURRENT) {
    vesc_.setCurrent(current_limit_.clip(current->data));
  }
}

/**
 * @param brake Commanded VESC braking current in Amps. Any value is accepted by this driver.
 *              However, note that the VESC may impose a more restrictive bounds on the range
 *              depending on its configuration.
 */
void VescDriver::brakeCallback(const std_msgs::Float64::ConstPtr& brake)
{
  if (driver_mode_ == DRIVER_MODE_OPERATING && control_mode_ == CONTROL_MODE_BRAKE) {
    vesc_.setBrake(brake_limit_.clip(brake->data));
  }
}

/**
 * @param speed Commanded VESC speed in electrical RPM. Electrical RPM is the mechanical RPM
 *              multiplied by the number of motor poles. Any value is accepted by this
 *              driver. However, note that the VESC may impose a more restrictive bounds on the
 *              range depending on its configuration.
 */
void VescDriver::speedCallback(const std_msgs::Float64::ConstPtr& speed)
{
  if (driver_mode_ == DRIVER_MODE_OPERATING && control_mode_ == CONTROL_MODE_SPEED) {
    vesc_.setSpeed(speed_limit_.clip(speed->data));
  }
}

/**
 * @param position Commanded VESC motor position in radians. Any value is accepted by this driver.
 *                 Note that the VESC must be in encoder mode for this command to have an effect.
 */
void VescDriver::positionCallback(const std_msgs::Float64::ConstPtr& position)
{
  if (driver_mode_ == DRIVER_MODE_OPERATING && control_mode_ == CONTROL_MODE_POSITION) {
    // ROS uses radians but VESC seems to use degrees. Convert to degrees.
    double position_deg = position_limit_.clip(position->data) * 180.0 / M_PI;
    vesc_.setPosition(position_deg);
  }
}

/**
 * @param servo Commanded VESC servo output position. Valid range is 0 to 1.
 */
void VescDriver::servoCallback(const std_msgs::Float64::ConstPtr& servo)
{
  if (driver_mode_ == DRIVER_MODE_OPERATING  && servo_mode_ == SERVO_MODE_ON) {
    double servo_clipped(servo_limit_.clip(servo->data));
    vesc_.setServo(servo_clipped);
    // publish clipped servo value as a "sensor"
    std_msgs::Float64::Ptr servo_sensor_msg(new std_msgs::Float64);
    servo_sensor_msg->data = servo_clipped;
  }
}

VescDriver::CommandLimit::CommandLimit(const ros::NodeHandle& nh, const std::string& str,
                                       const boost::optional<double>& min_lower,
                                       const boost::optional<double>& max_upper): name(str)
{
  // check if user's minimum value is outside of the range min_lower to max_upper
  double param_min;
  if (nh.getParam(name + "_min", param_min)) {
    if (min_lower && param_min < *min_lower) {
      lower = *min_lower;
      ROS_WARN_STREAM("Parameter " << name << "_min (" << param_min <<
                      ") is less than the feasible minimum (" << *min_lower << ").");
    }
    else if (max_upper && param_min > *max_upper) {
      lower = *max_upper;
      ROS_WARN_STREAM("Parameter " << name << "_min (" << param_min <<
                      ") is greater than the feasible maximum (" << *max_upper << ").");
    }
    else {
      lower = param_min;
    }
  }
  else if (min_lower) {
    lower = *min_lower;
  }

  // check if the uers' maximum value is outside of the range min_lower to max_upper
  double param_max;
  if (nh.getParam(name + "_max", param_max)) {
    if (min_lower && param_max < *min_lower) {
      upper = *min_lower;
      ROS_WARN_STREAM("Parameter " << name << "_max (" << param_max <<
                      ") is less than the feasible minimum (" << *min_lower << ").");
    }
    else if (max_upper && param_max > *max_upper) {
      upper = *max_upper;
      ROS_WARN_STREAM("Parameter " << name << "_max (" << param_max <<
                      ") is greater than the feasible maximum (" << *max_upper << ").");
    }
    else {
      upper = param_max;
    }
  }
  else if (max_upper) {
    upper = *max_upper;
  }

  // check for min > max
  if (upper && lower && *lower > *upper) {
    ROS_WARN_STREAM("Parameter " << name << "_max (" << *upper
                    << ") is less than parameter " << name << "_min (" << *lower << ").");
    double temp(*lower);
    lower = *upper;
    upper = temp;
  }

  std::ostringstream oss;
  oss << "  " << name << " limit: ";
  if (lower) oss << *lower << " "; else oss << "(none) ";
  if (upper) oss << *upper; else oss << "(none)";
  ROS_DEBUG_STREAM(oss.str());
}

double VescDriver::CommandLimit::clip(double value)
{
  if (lower && value < lower) {
    ROS_INFO_THROTTLE(10, "%s command value (%f) below minimum limit (%f), clipping.",
                      name.c_str(), value, *lower);
    return *lower;
  }
  if (upper && value > upper) {
    ROS_INFO_THROTTLE(10, "%s command value (%f) above maximum limit (%f), clipping.",
                      name.c_str(), value, *upper);
    return *upper;
  }
  return value;
}


} // namespace vesc_driver
