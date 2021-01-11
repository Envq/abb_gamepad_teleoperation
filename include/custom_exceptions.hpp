#pragma once

#include "colors.hpp"
#include <iostream>
#include <stdexcept>
#include <string>


namespace auxiliary_interfaces {
class AUXException : public std::exception {
  protected:
    const std::string msg_;
    const Colors color_;  // default is white

  public:
    AUXException(const std::string msg, const Colors &color = FG_WHITE)
        : std::exception(), msg_(msg), color_(color){};

    const char *what() const noexcept {
        return msg_.c_str();
    }

    const std::string getInfo() const noexcept {
        return colorize(msg_, color_);
    }
};
}  // namespace auxiliary_interfaces



namespace simple_interface {
class EGMErrorException : public auxiliary_interfaces::AUXException {
  public:
    EGMErrorException(const std::string msg)
        : auxiliary_interfaces::AUXException(msg, Colors::FG_RED){};
};

class EGMWarnException : public auxiliary_interfaces::AUXException {
  public:
    EGMWarnException(const std::string msg)
        : auxiliary_interfaces::AUXException(msg, Colors::FG_YELLOW){};
};
}  // namespace simple_interface



namespace joystick_interface {
class JoyErrorException : public auxiliary_interfaces::AUXException {
  public:
    JoyErrorException(const std::string msg)
        : auxiliary_interfaces::AUXException(msg, Colors::FG_RED){};
};
class JoyWarnException : public auxiliary_interfaces::AUXException {
  public:
    JoyWarnException(const std::string msg)
        : auxiliary_interfaces::AUXException(msg, Colors::FG_YELLOW){};
};
}  // namespace joystick_interface