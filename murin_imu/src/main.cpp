// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <libserial/SerialPort.h>
#include <jsoncpp/json/json.h>
#include <zlib.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
        : Node("minimal_publisher"), count_(0)
    {
        _publisherImu = this->create_publisher<sensor_msgs::msg::Imu>("bno085/imu", 10);
        _publisherMagneticField = this->create_publisher<sensor_msgs::msg::MagneticField>("bno085/mag", 10);
        timer_ = this->create_wall_timer(
            20ms, std::bind(&MinimalPublisher::timer_callback, this));
        serial_conn_.Open("/dev/robot_imu");
        serial_conn_.SetBaudRate(LibSerial::BaudRate::BAUD_460800);
        serial_conn_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serial_conn_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_SOFTWARE);
        serial_conn_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        serial_conn_.SetParity(LibSerial::Parity::PARITY_NONE);
    }

private:
    LibSerial::SerialPort serial_conn_;
    std::array<double, 4> orientation_values_ = {0, 0, 0, 1};
    std::array<double, 3> angular_velocity_values_ = {0, 0, 0};
    std::array<double, 3> linear_acceleration_values_ = {0, 0, 0};
    std::array<double, 3> magnetic_field_values_ = {0, 0, 0};

    void timer_callback()
    {
        std::string _read_str = "";
        std::string _send_str = "150088878{\"topic\":\"ros2_state\"}\r\n";
        serial_conn_.Write(_send_str);
        serial_conn_.DrainWriteBuffer();

        try
        {
            serial_conn_.ReadLine(_read_str, '\n', 500);
        }
        catch (const LibSerial::ReadTimeout &)
        {
            std::cerr << "The ReadByte() call has timed out." << std::endl;
        }
        RCLCPP_DEBUG(rclcpp::get_logger("murin_imu"), "==> %s", _send_str.c_str());
        RCLCPP_DEBUG(rclcpp::get_logger("murin_imu"), "<== %s", _read_str.c_str());

        std::size_t startIndex = _read_str.find_first_of("{");
        std::size_t stopIndex = _read_str.find_last_of("}");

        if (startIndex != std::string::npos && stopIndex != std::string::npos)
        {
            ulong crc_value = 0;
            std::string crc_str = _read_str.substr(0, startIndex);
            try
            {
                crc_value = std::stoul(crc_str);
            }
            catch (const std::exception &e)
            {
                std::cerr << "sub string error " << e.what() << '\n';
                return;
            }

            std::string str_to_parse = _read_str.substr(startIndex, stopIndex - startIndex + 1);
            ulong crc_cal = crc32(0L, Z_NULL, 0);

            unsigned char bytes[200];
            std::memcpy(bytes, str_to_parse.data(), str_to_parse.length());

            crc_cal = crc32(crc_cal, (const Bytef *)bytes, str_to_parse.length());

            if (crc_value == crc_cal)
            {
                Json::Value root;
                Json::Reader reader;
                bool parsingSuccessful = reader.parse(str_to_parse, root);
                if (!parsingSuccessful)
                {
                    RCLCPP_DEBUG(rclcpp::get_logger("murin_base_hardware"), "Error parsing the string from serial");
                }

                const auto orientation = root["qua"];
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

                const auto magnetic = root["mag"];
                magnetic_field_values_[0] = magnetic[0].asDouble();
                magnetic_field_values_[1] = magnetic[1].asDouble();
                magnetic_field_values_[2] = magnetic[2].asDouble();

                auto messageImu = sensor_msgs::msg::Imu();
                messageImu.header.frame_id = "imu_link";
                messageImu.header.stamp = rclcpp::Clock().now();

                messageImu.orientation.x = orientation_values_[0];
                messageImu.orientation.y = orientation_values_[1];
                messageImu.orientation.z = orientation_values_[2];
                messageImu.orientation.w = orientation_values_[3];

                messageImu.linear_acceleration.x = linear_acceleration_values_[0];
                messageImu.linear_acceleration.y = linear_acceleration_values_[1];
                messageImu.linear_acceleration.z = linear_acceleration_values_[2];

                messageImu.angular_velocity.x = angular_velocity_values_[0];
                messageImu.angular_velocity.y = angular_velocity_values_[1];
                messageImu.angular_velocity.z = angular_velocity_values_[2];
                _publisherImu->publish(messageImu);

                auto messageMag = sensor_msgs::msg::MagneticField();
                messageMag.header.frame_id = "imu_link";
                messageMag.header.stamp = messageImu.header.stamp;

                messageMag.magnetic_field.x = magnetic_field_values_[0];
                messageMag.magnetic_field.y = magnetic_field_values_[1];
                messageMag.magnetic_field.z = magnetic_field_values_[2];

                _publisherMagneticField->publish(messageMag);
            }
        }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _publisherImu;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr _publisherMagneticField;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
