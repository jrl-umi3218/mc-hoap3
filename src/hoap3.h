#pragma once

#include <RBDyn/parsers/urdf.h>
#include <mc_rbdyn/RobotModuleMacros.h>
#include <mc_robots/api.h>
#include <mc_rtc/logging.h>

namespace mc_robots
{

enum class Base
{
  Floating,
  Fixed
};

struct MC_ROBOTS_DLLAPI HOAP3RobotModule : public mc_rbdyn::RobotModule
{
  HOAP3RobotModule();

 protected:
  Base base_;
  bool canonical_ = true;
};

}  // namespace mc_robots

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string>& names)
  {
    names = {
        "HOAP3",
        "HOAP3::Fixed"};
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule* ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule* create(const std::string& n)
  {
    ROBOT_MODULE_CHECK_VERSION("HOAP3")
    if (n == "HOAP3")
    {
      return new mc_robots::HOAP3RobotModule(mc_robots::Base::Floating, false);
    }
    else if (n == "HOAP3::Fixed")
    {
      return new mc_robots::HOAP3RobotModule(mc_robots::Base::Fixed, false);
    }
  }
}
