#ifndef MURIN_BASE_ARDUINO_COMMS_HPP
#define MURIN_BASE_ARDUINO_COMMS_HPP

#include <sstream>
#include <bitset>
#include <cstddef>
#include <libserial/SerialPort.h>
#include <iostream>
#include <cstring>
#include <jsoncpp/json/json.h>

#include <zlib.h>

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
  case 1200:
    return LibSerial::BaudRate::BAUD_1200;
  case 1800:
    return LibSerial::BaudRate::BAUD_1800;
  case 2400:
    return LibSerial::BaudRate::BAUD_2400;
  case 4800:
    return LibSerial::BaudRate::BAUD_4800;
  case 9600:
    return LibSerial::BaudRate::BAUD_9600;
  case 19200:
    return LibSerial::BaudRate::BAUD_19200;
  case 38400:
    return LibSerial::BaudRate::BAUD_38400;
  case 57600:
    return LibSerial::BaudRate::BAUD_57600;
  case 115200:
    return LibSerial::BaudRate::BAUD_115200;
  case 230400:
    return LibSerial::BaudRate::BAUD_230400;
  case 460800:
    return LibSerial::BaudRate::BAUD_460800;
  default:
    std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
    return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoComms
{

public:
  ArduinoComms() = default;

  void init(const std::string &serial_device, int32_t baud_rate = 112500, int32_t timeout_ms = 1000, int32_t reconnect_timeout_ms = 5000)
  {
    this->timeout_ms_ = timeout_ms;
    this->baud_rate_ = baud_rate;
    this->serial_device_ = serial_device;
    this->reconnect_timeout_ms_ = reconnect_timeout_ms;
  }

  void connect()
  {
    serial_conn_.Open(this->serial_device_);
    serial_conn_.SetBaudRate(convert_baud_rate(this->baud_rate_));
    serial_conn_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_conn_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_SOFTWARE);
    serial_conn_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    serial_conn_.SetParity(LibSerial::Parity::PARITY_NONE);
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }

  bool read_hardware_states(std::string &str_out, bool print_output = false)
  {
    // serial_conn_.FlushIOBuffers();
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
      if (print_output)
        std::cerr << "The ReadByte() call has timed out." << std::endl;
      return false;
    }
    if (print_output)
    {
      std::cout << "==> " << _send_str;
      std::cout << "<== " << _read_str;
    }
    std::size_t startIndex = _read_str.find_first_of("{");
    std::size_t stopIndex = _read_str.find_last_of("}");
    // std::cout << _read_str.length() << " - " << startIndex << " - " << stopIndex << std::endl;

    if (startIndex != std::string::npos && stopIndex != std::string::npos)
    {
      uLong crc_value = 0;
      std::string crc_str = _read_str.substr(0, startIndex);
      try
      {
        crc_value = std::stoul(crc_str);
      }
      catch (const std::exception &e)
      {
        if (print_output)
          std::cerr << "sub string error " << e.what() << '\n';
        return false;
      }

      std::string str_to_parse = _read_str.substr(startIndex, stopIndex - startIndex + 1);
      // std::cout << crc_str << " - " << str_to_parse << std::endl;
      uLong crc_cal = crc32(0L, Z_NULL, 0);

      unsigned char bytes[str_to_parse.length()];
      std::memcpy(bytes, str_to_parse.data(), str_to_parse.length());

      crc_cal = crc32(crc_cal, (const Bytef *)bytes, sizeof(bytes));
      // std::cout << crc_value << " - " << crc_cal << std::endl;
      // std::cout << "string: " << _read_str << " - " << _read_str.length() << std::endl;
      // std::cout << "parsing: " << str_to_parse << " - " << str_to_parse.length() << std::endl;
      if (crc_value == crc_cal)
      {
        str_out = str_to_parse;
        return true;
      }
      else
        return false;
    }
    else
      return false;
  }

  bool write_hardware_command(const std::string &msg_to_send, bool print_output = false)
  {

    unsigned char bytes[msg_to_send.length()];
    std::memcpy(bytes, msg_to_send.data(), msg_to_send.length());

    uLong crc_cal = crc32(0L, Z_NULL, 0);
    crc_cal = crc32(crc_cal, (const Bytef *)bytes, sizeof(bytes));
    const std::string msg_to_serial = std::to_string(crc_cal) + msg_to_send + "\r\n";
    // const std::string msg_to_serial = "326728755{\"topic\":\"ros2_control\",\"velocity\":[0.00,0.00,0.00,0.00]}\r\n";

    serial_conn_.Write(msg_to_serial);
    serial_conn_.DrainWriteBuffer();

    std::string response = "";
    try
    {
      serial_conn_.ReadLine(response, '\n', 50);
    }
    catch (const LibSerial::ReadTimeout &)
    {
      if (print_output)
        std::cerr << "The ReadByte() call has timed out." << std::endl;
      return false;
    }

    if (print_output)
    {
      std::cout << "==> " << msg_to_serial;
      std::cout << "<== " << response;
    }

    return true;
  }

private:
  LibSerial::SerialPort serial_conn_;
  std::string serial_device_ = "";
  int32_t baud_rate_;
  int32_t timeout_ms_;
  int32_t reconnect_timeout_ms_;
};

#endif // MURIN_BASE_ARDUINO_COMMS_HPP