#pragma once
#include "custom_exceptions.hpp"
#include "custom_utils.hpp"
#include <SDL2/SDL.h>                  // Joystick dep
#include <boost/algorithm/string.hpp>  // split
#include <boost/thread.hpp>
#include <iostream>


namespace joystick_interface {
namespace config {
struct LogitechF310 {  // Direct-input Mode
    static const int JOY_LEFT_HORIZONTAL = 0;
    static const int JOY_LEFT_VERTICAL = 1;
    static const int JOY_RIGHT_HORIZONTAL = 2;
    static const int JOY_RIGHT_VERTICAL = 3;
    static const int A = 1;
    static const int X = 0;
    static const int B = 2;
    static const int Y = 3;
    static const int LB = 4;
    static const int RB = 5;
    static const int RT = 7;
    static const int LT = 6;
    static const int START = 9;
    static const int SELECT = 8;
    static const int JOY_LEFT = 10;
    static const int JOY_RIGHT = 11;
    static const int AXIS0_ORIGIN = 128;
    static const int AXIS1_ORIGIN = 129;
    static const int AXIS2_ORIGIN = 129;
};  // namespace LogitechF310

const int AXIS_X = LogitechF310::JOY_LEFT_HORIZONTAL;
const int AXIS_Y = LogitechF310::JOY_LEFT_VERTICAL;
const int AXIS_Z = LogitechF310::JOY_RIGHT_VERTICAL;
const int INCREMENT = LogitechF310::B;
const int DECREMENT = LogitechF310::X;
const int SET_ORIGIN = LogitechF310::RB;
const int QUIT = LogitechF310::START;
const int AXIS_X_ORIGIN = LogitechF310::AXIS0_ORIGIN;
const int AXIS_Y_ORIGIN = LogitechF310::AXIS1_ORIGIN;
const int AXIS_Z_ORIGIN = LogitechF310::AXIS2_ORIGIN;
const int GRANULARITY = 5;       // [mm]
const int DEFAULT_STRETCH = 30;  // [mm]
}  // namespace config


struct JoystickInfo {
    Pose pose;
    int stretch;
    bool setOrigin;
    bool quit;

    // Constructors
    JoystickInfo(const Pose pose_, const int stretch_, const bool setOrigin_,
                 const bool quit_)
        : pose(pose_), stretch(stretch_), setOrigin(setOrigin_), quit(quit_){};

    JoystickInfo() : JoystickInfo(Pose(), config::DEFAULT_STRETCH, false, false){};

    JoystickInfo(const JoystickInfo &state)
        : JoystickInfo(state.pose, state.stretch, state.setOrigin, state.quit){};

    // Functions
    std::string toString();
};

struct JoystickState {
    int axisX;
    int axisY;
    int axisZ;
    int stretchRequest;
    bool setOrigin;
    bool quit;

    // Constructors
    JoystickState(const int axisX_, const int axisY_, const int axisZ_,
                  const int stretchRequest_, const bool setOrigin_, const bool quit_)
        : axisX(axisX_), axisY(axisY_), axisZ(axisZ_), stretchRequest(stretchRequest_),
          setOrigin(setOrigin_), quit(quit_){};

    JoystickState() : JoystickState(0, 0, 0, 0, false, false){};

    JoystickState(const JoystickState &state)
        : JoystickState(state.axisX, state.axisY, state.axisZ, state.stretchRequest,
                        state.setOrigin, state.quit){};
    // Functions
    std::string toString();
};

class JoystickInterface {
  private:
    const double maxVal_ = 32768.0;          // Range: -32768 to 32767
    boost::mutex mutex_;                     // for avoid race condition
    JoystickState state_;                    // joystick state
    int stretch_ = config::DEFAULT_STRETCH;  // current stretch
    bool setOrigin_last_ = false;            // for avoid repeated setting
    int stretchRequest_last_ = 0;            // for avoid repeated setting
    void handle_joystick();                  // thread function

  public:
    JoystickInterface(boost::thread_group &thread_group);
    ~JoystickInterface();

    JoystickState readState();
    JoystickInfo readInfo();
};

}  // namespace joystick_interface
