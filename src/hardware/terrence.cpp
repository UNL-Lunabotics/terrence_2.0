// src/hardware/diffdrive_arduino_with_scoop.cpp

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

// this is William's serial library, same one Josh uses
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

    // Parameters from ros2_control.xacro
    try {
      device_ = info.hardware_parameters.at("device");
      baud_rate_ = std::stoul(info.hardware_parameters.at("baud_rate"));
      timeout_ms_ = std::stoul(info.hardware_parameters.at("timeout"));
      loop_rate_ = std::stod(info.hardware_parameters.at("loop_rate"));
      enc_counts_per_rev_ = std::stod(info.hardware_parameters.at("enc_counts_per_rev"));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("Terrence"),
        "Missing/invalid hardware parameter: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Joint indices: two drive joints + one scoop joint
    // We identify them by name so URDF can change ordering safely
    for (size_t i = 0; i < info.joints.size(); ++i) {
      const auto & j = info.joints[i];
      if (j.name == "DS_Joint") {
        left_index_ = static_cast<int>(i);
      } else if (j.name == "PS_Joint") {
        right_index_ = static_cast<int>(i);
      } else if (j.name == "LoaderJoint") {   // treat Loader as scoop
        scoop_index_ = static_cast<int>(i);
      }
    }

    if (left_index_ < 0 || right_index_ < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("Terrence"),
        "Could not find DS_Joint/PS_Joint in hardware info");
      return hardware_interface::CallbackReturn::ERROR;
    }

    size_t n_joints = info.joints.size();
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
        // Scoop: position command (up/down angle)
        cmds.emplace_back(hardware_interface::CommandInterface(
          name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
      } else {
        // joints that are read-only (if any) – skip
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
      RCLCPP_ERROR(rclcpp::get_logger("Terrence"),
        "Failed to open serial port %s: %s", device_.c_str(), e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!serial_.isOpen()) {
      RCLCPP_ERROR(rclcpp::get_logger("Terrence"),
        "Serial port %s is not open", device_.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    last_read_time_ = rclcpp::Clock().now();
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

  hardware_interface::return_type read(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/) override
  {
    if (!serial_.isOpen()) {
      return hardware_interface::return_type::ERROR;
    }

    // Request encoder values: "e\n"
    try {
      serial_.write("e\n");
      std::string line = serial_.readline(128, "\n");

      // Expect: e <left> <right> <scoop>\n
      std::istringstream iss(line);
      char prefix;
      long enc_left = 0, enc_right = 0, enc_scoop = 0;
      iss >> prefix >> enc_left >> enc_right >> enc_scoop;

      if (prefix != 'e') {
        return hardware_interface::return_type::OK; // ignore garbage
      }

      // Convert encoder counts to radians
      const double counts_to_rad = 2.0 * M_PI / enc_counts_per_rev_;

      rclcpp::Time now = rclcpp::Clock().now();
      double dt = (now - last_read_time_).seconds();
      if (dt <= 0.0) {
        dt = 1.0 / loop_rate_;
      }
      last_read_time_ = now;

      // We keep very simple state: position from counts, velocity from finite diff.
      auto update_joint = [&](int idx, long counts_now, long & counts_prev)
      {
        if (idx < 0) return;

        double pos = counts_now * counts_to_rad;
        double vel = (counts_now - counts_prev) * counts_to_rad / dt;

        hw_velocities_[idx] = vel;
        hw_positions_[idx] = pos;

        counts_prev = counts_now;
      };

      update_joint(left_index_, enc_left, prev_enc_left_);
      update_joint(right_index_, enc_right, prev_enc_right_);
      update_joint(scoop_index_, enc_scoop, prev_enc_scoop_);

    } catch (const std::exception & e) {
      RCLCPP_WARN(rclcpp::get_logger("Terrence"),
        "Error during read(): %s", e.what());
      return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/) override
  {
    if (!serial_.isOpen()) {
      return hardware_interface::return_type::ERROR;
    }

    // Commands:
    // wheels: velocity [rad/s]
    // scoop: position [rad] – we map to a simple proportional "velocity" command
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
      // foolishly-simple P control on position error, unit-less
      double pos_error = hw_commands_[scoop_index_] - hw_positions_[scoop_index_];
      double k_p = 1.0; // tune this on the real robot
      scoop_cmd = k_p * pos_error;
    }

    std::ostringstream oss;
    // "m <left> <right> <scoop>\n"
    oss << "m " << left_cmd << " " << right_cmd << " " << scoop_cmd << "\n";

    try {
      serial_.write(oss.str());
    } catch (const std::exception & e) {
      RCLCPP_WARN(rclcpp::get_logger("Terrence"),
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
  double enc_counts_per_rev_{3436.0};

  // State
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;

  int left_index_{-1};
  int right_index_{-1};
  int scoop_index_{-1};

  long prev_enc_left_{0};
  long prev_enc_right_{0};
  long prev_enc_scoop_{0};
  rclcpp::Time last_read_time_;

  serial::Serial serial_;
};

}  // namespace terrence_2

PLUGINLIB_EXPORT_CLASS(
  terrence_2::Terrence,
  hardware_interface::SystemInterface)
