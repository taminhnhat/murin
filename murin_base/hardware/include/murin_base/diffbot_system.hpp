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

#ifndef MURIN_BASE__DIFFBOT_SYSTEM_HPP_
#define MURIN_BASE__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "controller_interface/controller_interface.hpp"
#include "semantic_components/imu_sensor.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "murin_base/visibility_control.h"

#include "murin_base/arduino_comms.hpp"
#include "murin_base/wheel.hpp"
#include "murin_base/named_pipe.hpp"

namespace murin_base
{
  class murin_base_hardware : public hardware_interface::SystemInterface
  {

    struct Config
    {
      std::string rear_left_wheel_name = "";
      std::string rear_right_wheel_name = "";
      std::string front_left_wheel_name = "";
      std::string front_right_wheel_name = "";
      float loop_rate = 0.0;
      std::string device = "";
      int baud_rate = 0;
      int timeout_ms = 0;
      int reconnect_timeout_ms = 5000;
      int enc_counts_per_rev = 0;
      int pid_p = 0;
      int pid_d = 0;
      int pid_i = 0;
      int pid_o = 0;
    };

  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(murin_base_hardware);

    MURIN_BASE_PUBLIC
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;

    MURIN_BASE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    MURIN_BASE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    MURIN_BASE_PUBLIC
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    MURIN_BASE_PUBLIC
    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &previous_state) override;

    MURIN_BASE_PUBLIC
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    MURIN_BASE_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    MURIN_BASE_PUBLIC
    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    MURIN_BASE_PUBLIC
    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    ArduinoComms comms_;
    NamedPipe pipe_;
    Config cfg_;
    Wheel wheel_rear_l_;
    Wheel wheel_rear_r_;
    Wheel wheel_front_l_;
    Wheel wheel_front_r_;
    double battery;

    const size_t size_ = 10;
    const std::string sensor_name_ = "mpu6050";
    std::array<double, 4> orientation_values_ = {1.1, 2.2, 3.3, 4.4};
    std::array<double, 3> angular_velocity_values_ = {4.4, 5.5, 6.6};
    std::array<double, 3> linear_acceleration_values_ = {4.4, 5.5, 6.6};
    // std::unique_ptr<TestableIMUSensor> imu_sensor_;
    const std::vector<std::string> imu_interface_names_ = {
        "orientation.x", "orientation.y", "orientation.z", "orientation.w",
        "angular_velocity.x", "angular_velocity.y", "angular_velocity.z", "linear_acceleration.x",
        "linear_acceleration.y", "linear_acceleration.z"};
  };

} // namespace murin_base

#endif // MURIN_BASE__DIFFBOT_SYSTEM_HPP_
