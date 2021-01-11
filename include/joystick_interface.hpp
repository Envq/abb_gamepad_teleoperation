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
    int axis0;  // Left x
    int axis1;  // Left y
    int axis2;  // Right x
    int axis3;  // Right y
    bool quit;

    // Constructors

    JoystickStatus(const int axis0_, const int axis1_, const int axis2_, const int axis3_,
                   const bool quit_)
        : axis0(axis0_), axis1(axis1_), axis2(axis2_), axis3(axis3_), quit(quit_){};

    JoystickStatus() : JoystickStatus(0, 0, 0, 0, false){};

    JoystickStatus(const JoystickStatus &status)
        : JoystickStatus(status.axis0, status.axis1, status.axis2, status.axis3,
                         status.quit){};

    std::string to_string();
    static JoystickStatus parse(std::string &sample);
};

class JoystickInterface {
  private:
    const double maxVal_ = 32768.0;  // Range: -32768 to 32767
    boost::mutex mutex_;
    JoystickStatus status_;
    void handle_joystick();

  public:
    JoystickInterface(boost::thread_group &thread_group);
    ~JoystickInterface();
    JoystickStatus read();
    double getJoystickMaxVal();
};

}  // namespace joystick_interface
