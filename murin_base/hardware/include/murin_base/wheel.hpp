#ifndef MURIN_BASE_WHEEL_HPP
#define MURIN_BASE_WHEEL_HPP

#include <string>
#include <cmath>

class Wheel
{
public:
  std::string name = "";
  int enc = 0;
  double cmd = 0;
  double pos = 0;
  double vel = 0;

  Wheel() = default;

  Wheel(const std::string &wheel_name)
  {
    setup(wheel_name);
  }

  void setup(const std::string &wheel_name)
  {
    name = wheel_name;
  }
};

#endif // murin_base_WHEEL_HPP
