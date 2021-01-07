#pragma once
#include "custom_exceptions.hpp"
#include <SDL2/SDL.h>                  // Joystick
#include <boost/algorithm/string.hpp>  //split
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <iostream>


namespace joystick_interface {
struct JoystickStatus {
    int axis0 = 0;  // Left x
    int axis1 = 0;  // Left y
    int axis2 = 0;  // Right x
    int axis3 = 0;  // Right y
    bool quit = false;

    std::string to_string();
    static JoystickStatus parse(std::string &sample);
};

class JoystickInterface {
  private:
    JoystickStatus status_;
    void handle_joystick();

  public:
    JoystickInterface(boost::thread_group &thread_group);
    JoystickStatus read();
    ~JoystickInterface();
};

}  // namespace joystick_interface
