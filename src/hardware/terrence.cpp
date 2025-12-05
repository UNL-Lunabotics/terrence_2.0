// src/hardware/terrence.cpp
//
// Open-loop hardware_interface for Terrence:
// - Two diff-drive wheels (velocity command).
// - One scoop joint (position command).
// - No encoders: we integrate wheel positions from commanded velocities
//   and treat scoop position as equal to its commanded value.
// - Serial is used only to send "m <left> <right> <scoop>" to the Teensy;
//   we do NOT send "e" or read encoder values.

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "serial/serial.h"

namespace terrence_2
{

class Terrence : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Terrence)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override
  {
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Required hardware parameters from ros2_control.xacro:
    //  - device
    //  - baud_rate
    //  - timeout
    //  - loop_rate
    try {
      device_ = info.hardware_parameters.at("device");
      baud_rate_ = std::stoul(info.hardware_parameters.at("baud_rate"));
      timeout_ms_ = std::stoul(info.hardware_parameters.at("timeout"));
      loop_rate_ = std::stod(info.hardware_parameters.at("loop_rate"));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("TerrenceHW"),
        "Missing/invalid hardware parameter: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Locate indices for the joints we care about
    for (size_t i = 0; i < info.joints.size(); ++i) {
      const auto & j = info.joints[i];
      if (j.name == "DS_Joint") {
        left_index_ = static_cast<int>(i);
      } else if (j.name == "PS_Joint") {
        right_index_ = static_cast<int>(i);
      } else if (j.name == "LoaderJoint") {
        scoop_index_ = static_cast<int>(i);
      }
    }

    if (left_index_ < 0 || right_index_ < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("TerrenceHW"),
        "Could not find DS_Joint / PS_Joint in hardware info");
      return hardware_interface::CallbackReturn::ERROR;
    }

    const size_t n_joints = info.joints.size();
    hw_positions_.assign(n_joints, 0.0);
    hw_velocities_.assign(n_joints, 0.0);
    hw_commands_.assign(n_joints, 0.0);

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> states;
    states.reserve(hw_positions_.size() * 2);

    for (size_t i = 0; i < info_.joints.size(); ++i) {
      const auto & name = info_.joints[i].name;
      states.emplace_back(hardware_interface::StateInterface(
        name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
      states.emplace_back(hardware_interface::StateInterface(
        name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    }

    return states;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> cmds;
    cmds.reserve(hw_commands_.size());

    for (size_t i = 0; i < info_.joints.size(); ++i) {
      const auto & name = info_.joints[i].name;

      if (static_cast<int>(i) == left_index_ || static_cast<int>(i) == right_index_) {
        // Wheels: velocity command
        cmds.emplace_back(hardware_interface::CommandInterface(
          name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
      } else if (static_cast<int>(i) == scoop_index_) {
        // Scoop: position command
        cmds.emplace_back(hardware_interface::CommandInterface(
          name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
      } else {
        // Any other joints are read-only (no command interface)
      }
    }

    return cmds;
  }

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    try {
      serial_.setPort(device_);
      serial_.setBaudrate(baud_rate_);
      serial_.setTimeout(serial::Timeout::simpleTimeout(timeout_ms_));
      serial_.open();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("TerrenceHW"),
        "Failed to open serial port %s: %s", device_.c_str(), e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!serial_.isOpen()) {
      RCLCPP_ERROR(rclcpp::get_logger("TerrenceHW"),
        "Serial port %s is not open", device_.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    last_update_time_ = rclcpp::Clock().now();

    RCLCPP_INFO(rclcpp::get_logger("TerrenceHW"),
      "Terrence hardware configured (open-loop, no encoders).");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    if (serial_.isOpen()) {
      serial_.close();
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Open-loop read:
  //  - Integrate wheel positions from commanded velocities.
  //  - Set wheel velocities equal to commanded velocities.
  //  - Set scoop position equal to commanded position (instantaneous),
  //    and scoop velocity to (pos_new - pos_old) / dt.
  hardware_interface::return_type read(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & period) override
  {
    const double dt = (period.seconds() > 0.0) ? period.seconds() : (1.0 / loop_rate_);

    auto integrate_wheel = [&](int idx)
    {
      if (idx < 0) return;
      const double cmd_vel = hw_commands_[idx];
      hw_positions_[idx] += cmd_vel * dt;
      hw_velocities_[idx] = cmd_vel;
    };

    // Wheels: DS_Joint and PS_Joint
    integrate_wheel(left_index_);
    integrate_wheel(right_index_);

    // Scoop: LoaderJoint – command is position
    if (scoop_index_ >= 0) {
      const double old_pos = hw_positions_[scoop_index_];
      const double new_pos = hw_commands_[scoop_index_];
      hw_positions_[scoop_index_] = new_pos;

      const double vel = (new_pos - old_pos) / dt;
      hw_velocities_[scoop_index_] = vel;
    }

    last_update_time_ = rclcpp::Clock().now();
    return hardware_interface::return_type::OK;
  }

  // Write: send commands to Teensy via "m <left> <right> <scoop>\n"
  hardware_interface::return_type write(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/) override
  {
    if (!serial_.isOpen()) {
      return hardware_interface::return_type::ERROR;
    }

    double left_cmd = 0.0;
    double right_cmd = 0.0;
    double scoop_cmd = 0.0;

    if (left_index_ >= 0) {
      left_cmd = hw_commands_[left_index_];
    }
    if (right_index_ >= 0) {
      right_cmd = hw_commands_[right_index_];
    }
    if (scoop_index_ >= 0) {
      // hw_commands_[scoop_index_] is desired scoop position.
      // Teensy side expects a "velocity-like" number; we just pass the
      // position command through for now and interpret it however we want
      // in the firmware (e.g., clamp, treat as normalized up/down).
      scoop_cmd = hw_commands_[scoop_index_];
    }

    std::ostringstream oss;
    oss << "m " << left_cmd << " " << right_cmd << " " << scoop_cmd << "\n";

    try {
      serial_.write(oss.str());
    } catch (const std::exception & e) {
      RCLCPP_WARN(rclcpp::get_logger("TerrenceHW"),
        "Error during write(): %s", e.what());
      return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
  }

private:
  // Serial config
  std::string device_;
  unsigned long baud_rate_{57600};
  unsigned long timeout_ms_{1000};
  double loop_rate_{30.0};

  // State
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;

  int left_index_{-1};
  int right_index_{-1};
  int scoop_index_{-1};

  rclcpp::Time last_update_time_;

  serial::Serial serial_;
};

}  // namespace terrence_2

PLUGINLIB_EXPORT_CLASS(
  terrence_2::Terrence,
  hardware_interface::SystemInterface)
