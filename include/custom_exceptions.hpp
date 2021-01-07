#pragma once

#include <iostream>
#include <stdexcept>
#include <string>


namespace auxiliary_interfaces {
class AUXException : public std::exception {
  protected:
    const std::string msg_;
    const int color_;  // default is white

  public:
    AUXException(const std::string msg, const int color = 37)
        : std::exception(), msg_(msg), color_(color){};

    const char *what() const noexcept {
        return msg_.c_str();
    }

    const std::string getInfo() const noexcept {
        std::string colorful_msg = "\033[1;";
        colorful_msg.append(std::to_string(color_));
        colorful_msg.append("m");
        colorful_msg.append(msg_);
        colorful_msg.append("\033[0m");
        return colorful_msg;
    }
};
}  // namespace auxiliary_interfaces



namespace simple_interface {
class EGMErrorException : public auxiliary_interfaces::AUXException {
  public:
    EGMErrorException(const std::string msg)
        : auxiliary_interfaces::AUXException(msg, 31){};
};

class EGMWarnException : public auxiliary_interfaces::AUXException {
  public:
    EGMWarnException(const std::string msg)
        : auxiliary_interfaces::AUXException(msg, 33){};
};
}  // namespace simple_interface



namespace joystick_interface {
class JoyErrorException : public auxiliary_interfaces::AUXException {
  public:
    JoyErrorException(const std::string msg)
        : auxiliary_interfaces::AUXException(msg, 31){};
};
class JoyWarnException : public auxiliary_interfaces::AUXException {
  public:
    JoyWarnException(const std::string msg)
        : auxiliary_interfaces::AUXException(msg, 33){};
};
}  // namespace joystick_interface