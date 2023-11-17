// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SMC_ROS2_ARDUINO_HW_INTERFACE_SYSTEM_HPP_
#define SMC_ROS2_ARDUINO_HW_INTERFACE_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "smc_ros2_arduino_hw_interface/visibility_control.h"
#include "smc_ros2_arduino_hw_interface/arduino_libserial_comm.hpp"
#include "smc_ros2_arduino_hw_interface/motor.hpp"




namespace smc_ros2_arduino_hw_interface
{
class SMCArduinoHardwareInterface : public hardware_interface::SystemInterface
{

struct Config
{
  std::string motorA_wheel_name = "";
  std::string motorB_wheel_name = "";
  std::string port = "";
};


public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SMCArduinoHardwareInterface);

  SMC_ROS2_ARDUINO_HW_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  SMC_ROS2_ARDUINO_HW_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  SMC_ROS2_ARDUINO_HW_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  SMC_ROS2_ARDUINO_HW_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  SMC_ROS2_ARDUINO_HW_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;


  SMC_ROS2_ARDUINO_HW_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  SMC_ROS2_ARDUINO_HW_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  SMC_ROS2_ARDUINO_HW_INTERFACE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  SMC_ROS2_ARDUINO_HW_INTERFACE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  SMCArduinoLibserialComm smc_;
  Config cfg_;
  Motor motorA_;
  Motor motorB_;
};

}  // namespace smc_ros2_arduino_hw_interface

#endif  // SMC_ROS2_ARDUINO_HW_INTERFACE_SYSTEM_HPP_
