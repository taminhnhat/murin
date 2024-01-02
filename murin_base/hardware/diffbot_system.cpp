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

#include "murin_base/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <jsoncpp/json/json.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace murin_base
{
  hardware_interface::CallbackReturn murin_base_hardware::on_init(const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    // if (hardware_interface::SensorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    // {
    //   return hardware_interface::CallbackReturn::ERROR;
    // }

    cfg_.rear_right_wheel_name = info_.hardware_parameters["rear_right_wheel_name"];
    cfg_.rear_left_wheel_name = info_.hardware_parameters["rear_left_wheel_name"];
    cfg_.front_right_wheel_name = info_.hardware_parameters["front_right_wheel_name"];
    cfg_.front_left_wheel_name = info_.hardware_parameters["front_left_wheel_name"];
    cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
    cfg_.device = info_.hardware_parameters["device"];
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
    if (info_.hardware_parameters.count("pid_p") > 0)
    {
      cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
      cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
      cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
      cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("murin_base_hardware"), "PID values not supplied, using defaults.");
    }

    wheel_rear_l_.setup(cfg_.rear_left_wheel_name);
    wheel_front_l_.setup(cfg_.front_left_wheel_name);
    wheel_rear_r_.setup(cfg_.rear_right_wheel_name);
    wheel_front_r_.setup(cfg_.front_right_wheel_name);

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // DiffBotSystem has exactly two states and one command interface on each joint
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("murin_base_hardware"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("murin_base_hardware"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("murin_base_hardware"),
            "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("murin_base_hardware"),
            "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("murin_base_hardware"),
            "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> murin_base_hardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // rear left wheel
    state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_rear_l_.name, hardware_interface::HW_IF_POSITION, &wheel_rear_l_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_rear_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rear_l_.vel));
    // rear right wheel
    state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_rear_r_.name, hardware_interface::HW_IF_POSITION, &wheel_rear_r_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_rear_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rear_r_.vel));
    // front left wheel
    state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_front_l_.name, hardware_interface::HW_IF_POSITION, &wheel_front_l_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_front_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_front_l_.vel));
    // front right wheel
    state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_front_r_.name, hardware_interface::HW_IF_POSITION, &wheel_front_r_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_front_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_front_r_.vel));
    // assign values to orientation
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name_, imu_interface_names_[0], &orientation_values_[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name_, imu_interface_names_[1], &orientation_values_[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name_, imu_interface_names_[2], &orientation_values_[2]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name_, imu_interface_names_[3], &orientation_values_[3]));
    // assign values to angular velocity
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name_, imu_interface_names_[4], &angular_velocity_values_[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name_, imu_interface_names_[5], &angular_velocity_values_[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name_, imu_interface_names_[6], &angular_velocity_values_[2]));
    // assign values to linear acceleration
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name_, imu_interface_names_[7], &linear_acceleration_values_[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name_, imu_interface_names_[8], &linear_acceleration_values_[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name_, imu_interface_names_[9], &linear_acceleration_values_[2]));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> murin_base_hardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(wheel_rear_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rear_l_.cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(wheel_front_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_front_l_.cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(wheel_rear_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rear_r_.cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(wheel_front_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_front_r_.cmd));

    return command_interfaces;
  }

  hardware_interface::CallbackReturn murin_base_hardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("murin_base_hardware"), "Configuring ...please wait...");
    if (comms_.connected())
    {
      comms_.disconnect();
    }
    comms_.init(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
    comms_.connect();
    if (!comms_.connected())
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("murin_base_hardware"), "Successfully configured!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn murin_base_hardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("murin_base_hardware"), "Cleaning up ...please wait...");
    if (comms_.connected())
    {
      comms_.disconnect();
    }
    RCLCPP_INFO(rclcpp::get_logger("murin_base_hardware"), "Successfully cleaned up!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn murin_base_hardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("murin_base_hardware"), "Activating ...please wait...");
    if (!comms_.connected())
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("murin_base_hardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn murin_base_hardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("murin_base_hardware"), "Deactivating ...please wait...");
    RCLCPP_INFO(rclcpp::get_logger("murin_base_hardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type murin_base_hardware::read(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
  {
    if (!comms_.connected())
    {
      return hardware_interface::return_type::ERROR;
    }
    std::string read_str = "";
    if (comms_.read_hardware_states(read_str, false))
    {
      RCLCPP_DEBUG(rclcpp::get_logger("murin_base_hardware"), "<<< %s", read_str.c_str());
      Json::Value root;
      Json::Reader reader;
      bool parsingSuccessful = reader.parse(read_str, root);
      if (!parsingSuccessful)
      {
        RCLCPP_DEBUG(rclcpp::get_logger("murin_base_hardware"), "Error parsing the string from serial");
        return hardware_interface::return_type::OK;
      }

      // const int battery = root["battery"].asDouble();
      if (pipe_.writeLine(read_str, false) == -1)
      {
        RCLCPP_DEBUG(rclcpp::get_logger("murin_base_hardware"), "Fail writing to pipe! Closed pipe.");
        return hardware_interface::return_type::OK;
      }
      const auto velocity = root["vel"];
      wheel_front_r_.vel = velocity[0].asDouble();
      wheel_rear_r_.vel = velocity[1].asDouble();
      wheel_rear_l_.vel = velocity[2].asDouble();
      wheel_front_l_.vel = velocity[3].asDouble();

      const auto position = root["pos"];
      wheel_front_r_.pos = position[0].asDouble();
      wheel_rear_r_.pos = position[1].asDouble();
      wheel_rear_l_.pos = position[2].asDouble();
      wheel_front_l_.pos = position[3].asDouble();

      const auto orientation = root["ori"];
      orientation_values_[0] = orientation[0].asDouble();
      orientation_values_[1] = orientation[1].asDouble();
      orientation_values_[2] = orientation[2].asDouble();
      orientation_values_[3] = orientation[3].asDouble();

      const auto gyroscope = root["gyr"];
      angular_velocity_values_[0] = gyroscope[0].asDouble();
      angular_velocity_values_[1] = gyroscope[1].asDouble();
      angular_velocity_values_[2] = gyroscope[2].asDouble();

      const auto acceleration = root["acc"];
      linear_acceleration_values_[0] = acceleration[0].asDouble();
      linear_acceleration_values_[1] = acceleration[1].asDouble();
      linear_acceleration_values_[2] = acceleration[2].asDouble();
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("murin_base_hardware"), "Fail writing to serial!");
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type murin_base ::murin_base_hardware::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
  {
    if (!comms_.connected())
    {
      return hardware_interface::return_type::ERROR;
    }

    double front_right_vel = wheel_front_r_.cmd;
    double rear_right_vel = wheel_rear_r_.cmd;
    double front_left_vel = wheel_front_l_.cmd;
    double rear_left_vel = wheel_rear_l_.cmd;
    char cmd[100];
    sprintf(cmd, "{\"topic\":\"ros2_control\",\"velocity\":[%.2f,%.2f,%.2f,%.2f]}", front_right_vel, rear_right_vel, rear_left_vel, front_left_vel);
    std::string msg = cmd;
    if (!comms_.write_hardware_command(msg, false))
      RCLCPP_DEBUG(rclcpp::get_logger("murin_base_hardware"), "Fail writing to serial!");
    else
      RCLCPP_DEBUG(rclcpp::get_logger("murin_base_hardware"), ">>> %s", msg.c_str());
    return hardware_interface::return_type::OK;
  }

} // namespace murin_base

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    murin_base::murin_base_hardware,
    hardware_interface::SystemInterface)
