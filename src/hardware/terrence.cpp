// src/hardware/terrence.cpp
//
// Terrence hardware_interface using standalone Asio for serial I/O.
// - Two diff-drive wheels (DS_Joint, PS_Joint) with velocity commands.
// - One scoop joint (LoaderJoint) with position command.
// - Open-loop: no encoders; we integrate wheel positions from commanded velocities.
// - Scoop position is set directly from command.
//
// Serial protocol (to Teensy/Arduino):
//   write: "m <left> <right> <scoop>\n"
//   (no reads, no "e" encoder requests)
//
// Requires standalone Asio headers (libasio-dev on Ubuntu) and C++17.

#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

// Standalone Asio (not Boost.Asio)
// Make sure ASIO_STANDALONE is defined before including <asio.hpp>.
#ifndef ASIO_STANDALONE
#define ASIO_STANDALONE
#endif
#include <asio.hpp>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

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

    // Parameters from ros2_control.xacro:
    // - device (e.g. "/dev/ttyACM0")
    // - baud_rate (e.g. "57600")
    // - timeout (ms, currently unused but kept for compatibility)
    // - loop_rate (for open-loop integration)
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

    // Find indices of the joints of interest by name
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
        // Other joints: read-only
      }
    }

    return cmds;
  }

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    // Open serial with Asio in synchronous mode
    asio::error_code ec;
    serial_port_ = std::make_unique<asio::serial_port>(io_context_);

    serial_port_->open(device_, ec);
    if (ec) {
      RCLCPP_ERROR(rclcpp::get_logger("TerrenceHW"),
        "Failed to open serial port %s: %s",
        device_.c_str(), ec.message().c_str());
      serial_port_.reset();
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Configure baudrate & basic port options
    serial_port_->set_option(asio::serial_port_base::baud_rate(baud_rate_), ec);
    if (ec) {
      RCLCPP_ERROR(rclcpp::get_logger("TerrenceHW"),
        "Failed to set baud rate %lu on %s: %s",
        baud_rate_, device_.c_str(), ec.message().c_str());
      serial_port_.reset();
      return hardware_interface::CallbackReturn::ERROR;
    }

    serial_port_->set_option(asio::serial_port_base::character_size(8), ec);
    serial_port_->set_option(asio::serial_port_base::parity(
      asio::serial_port_base::parity::none), ec);
    serial_port_->set_option(asio::serial_port_base::stop_bits(
      asio::serial_port_base::stop_bits::one), ec);
    serial_port_->set_option(asio::serial_port_base::flow_control(
      asio::serial_port_base::flow_control::none), ec);

    if (ec) {
      RCLCPP_ERROR(rclcpp::get_logger("TerrenceHW"),
        "Failed to fully configure serial port %s: %s",
        device_.c_str(), ec.message().c_str());
      serial_port_.reset();
      return hardware_interface::CallbackReturn::ERROR;
    }

    last_update_time_ = rclcpp::Clock().now();

    RCLCPP_INFO(rclcpp::get_logger("TerrenceHW"),
      "Terrence hardware configured (open-loop, Asio serial, no encoders).");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    if (serial_port_ && serial_port_->is_open()) {
      asio::error_code ec;
      serial_port_->close(ec);
      if (ec) {
        RCLCPP_WARN(rclcpp::get_logger("TerrenceHW"),
          "Error closing serial port %s: %s",
          device_.c_str(), ec.message().c_str());
      }
    }
    serial_port_.reset();
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Open-loop read:
  //  - Wheels: integrate position from commanded velocity, set velocity = command.
  //  - Scoop: position = command, velocity = finite difference.
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

    // Wheels
    integrate_wheel(left_index_);
    integrate_wheel(right_index_);

    // Scoop joint (LoaderJoint)
    if (scoop_index_ >= 0) {
      const double old_pos = hw_positions_[scoop_index_];
      const double new_pos = hw_commands_[scoop_index_];  // commanded position
      hw_positions_[scoop_index_] = new_pos;
      hw_velocities_[scoop_index_] = (new_pos - old_pos) / dt;
    }

    last_update_time_ = rclcpp::Clock().now();
    return hardware_interface::return_type::OK;
  }

  // Write: send "m <left> <right> <scoop>\n" via Asio
  hardware_interface::return_type write(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/) override
  {
    if (!serial_port_ || !serial_port_->is_open()) {
      RCLCPP_ERROR_THROTTLE(
        rclcpp::get_logger("TerrenceHW"), rclcpp::Clock(), 2000,
        "Serial port not open in write()");
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
      scoop_cmd = hw_commands_[scoop_index_];
    }

    std::ostringstream oss;
    oss << "m " << left_cmd << " " << right_cmd << " " << scoop_cmd << "\n";
    const std::string out = oss.str();

    asio::error_code ec;
    asio::write(*serial_port_, asio::buffer(out), ec);
    if (ec) {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("TerrenceHW"), rclcpp::Clock(), 2000,
        "Error writing to serial port %s: %s",
        device_.c_str(), ec.message().c_str());
      return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
  }

private:
  // Params
  std::string device_;
  unsigned long baud_rate_{57600};
  unsigned long timeout_ms_{1000};  // currently unused
  double loop_rate_{30.0};

  // ROS2-control state
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;

  int left_index_{-1};
  int right_index_{-1};
  int scoop_index_{-1};

  rclcpp::Time last_update_time_;

  // Asio context & serial
  asio::io_context io_context_;
  std::unique_ptr<asio::serial_port> serial_port_;
};

}  // namespace terrence_2

PLUGINLIB_EXPORT_CLASS(
  terrence_2::Terrence,
  hardware_interface::SystemInterface)
