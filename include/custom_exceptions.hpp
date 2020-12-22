#pragma once

#include <iostream>
#include <stdexcept>
#include <string>


namespace simple_interface {


class EGMException : public std::exception {
  protected:
    const std::string msg_;
    const int color_;  // default is white

  public:
    EGMException(const std::string msg, const int color = 37)
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

class EGMErrorException : public EGMException {
  public:
    EGMErrorException(const std::string msg) : EGMException(msg, 31){};
};

class EGMWarnException : public EGMException {
  public:
    EGMWarnException(const std::string msg) : EGMException(msg, 33){};
};


}  // namespace simple_interface
