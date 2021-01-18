#include <joystick_interface.hpp>
#include <sstream>


namespace joystick_interface {
// JoystickInfo =========================================================================
std::string JoystickInfo::toString() {
    std::ostringstream stream;
    stream << "[(" << pose.x << ", ";
    stream << pose.y << ", ";
    stream << pose.z << ", ";
    stream << pose.roll << ", ";
    stream << pose.pitch << ", ";
    stream << pose.yaw << "), ";
    stream << stretch << ", ";
    stream << setOrigin << ", ";
    stream << quit << "]";
    return stream.str();
}


// JoystickState ========================================================================
std::string JoystickState::toString() {
    std::ostringstream stream;
    stream << "[" << axisX << ", ";
    stream << axisY << ", ";
    stream << axisZ << ", ";
    stream << stretchRequest << ", ";
    stream << setOrigin << ", ";
    stream << quit << "]";
    return stream.str();
}


// JoystickInterface ====================================================================
JoystickInterface::JoystickInterface(boost::thread_group &thread_group, const int stretch)
    : stretch_(stretch) {
    thread_group.create_thread(boost::bind(&JoystickInterface::handle_joystick, this));
}

JoystickInterface::~JoystickInterface() {
    // std::cout << "DELETE JOYSTICK INTERFACE" << std::endl;
    SDL_Quit();
}

void JoystickInterface::handle_joystick() {
    if (SDL_Init(SDL_INIT_JOYSTICK) < 0)
        throw JoyErrorException("Couldn't initialize SDL");

    int num_joysticks = SDL_NumJoysticks();
    std::cout << num_joysticks << " joysticks were found." << std::endl;
    if (num_joysticks == 0) {
        SDL_Quit();
        throw JoyErrorException("0 Joysticks were found");
    }

    SDL_JoystickEventState(SDL_ENABLE);
    auto joy = SDL_JoystickOpen(0);

    bool done = false;
    bool quit = false;
    while (!done && !quit) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT)
                done = true;
        }

        // Get state
        auto joy_state =
            JoystickState{SDL_JoystickGetAxis(joy, config::AXIS_X),
                          -SDL_JoystickGetAxis(joy, config::AXIS_Y),
                          -SDL_JoystickGetAxis(joy, config::AXIS_Z),
                          (SDL_JoystickGetButton(joy, config::INCREMENT) -
                           SDL_JoystickGetButton(joy, config::DECREMENT)),
                          (SDL_JoystickGetButton(joy, config::SET_ORIGIN) != 0),
                          quit = (SDL_JoystickGetButton(joy, config::QUIT) != 0)};

        // Adjust strength request. Set it to zero if the axis are different from Origin.
        if ((joy_state.axisX < (config::AXIS_X_ORIGIN - config::AXIS_THRESHOLD)) ||
            (joy_state.axisX > (config::AXIS_X_ORIGIN + config::AXIS_THRESHOLD)) ||
            (joy_state.axisY < (config::AXIS_Y_ORIGIN - config::AXIS_THRESHOLD)) ||
            (joy_state.axisY > (config::AXIS_Y_ORIGIN + config::AXIS_THRESHOLD)) ||
            (joy_state.axisZ < (config::AXIS_Z_ORIGIN - config::AXIS_THRESHOLD)) ||
            (joy_state.axisZ > (config::AXIS_Z_ORIGIN + config::AXIS_THRESHOLD)))
            joy_state.stretchRequest = 0;

        // Update state
        mutex_.lock();
        state_ = joy_state;
        mutex_.unlock();

        // DEBUG PRINT
        // std::cout << joy_state.toString() << std::endl;
        // fflush(stdout);
        // boost::this_thread::sleep(boost::posix_time::millisec(100));
    }

    // Shutdown
    SDL_JoystickClose(joy);
    SDL_Quit();
}


JoystickState JoystickInterface::readState() {
    mutex_.lock();  // LOCK -------------------------------
    auto joy_state = state_;
    mutex_.unlock();  // UNLOCK ---------------------------
    return joy_state;
}

JoystickInfo JoystickInterface::readInfo() {
    JoystickInfo info;
    mutex_.lock();  // LOCK ------------------------------
    // Prevents the origin from being reset continuously if the key is pressed
    if (state_.stretchRequest != stretchRequest_last_)
        stretch_ += state_.stretchRequest * config::GRANULARITY;
    stretchRequest_last_ = state_.stretchRequest;  // Update last_
    info.stretch = stretch_;

    info.pose.x = (state_.axisX / static_cast<double>(config::AXIS_MAX_VALUE)) * stretch_;
    info.pose.y = (state_.axisY / static_cast<double>(config::AXIS_MAX_VALUE)) * stretch_;
    info.pose.z = (state_.axisZ / static_cast<double>(config::AXIS_MAX_VALUE)) * stretch_;
    info.pose.roll = 0.0;
    info.pose.pitch = 0.0;
    info.pose.yaw = 0.0;

    // Prevents the origin from being reset continuously if the key is pressed
    info.setOrigin = (state_.setOrigin && !setOrigin_last_);
    setOrigin_last_ = state_.setOrigin;  // Update last_

    info.quit = state_.quit;
    mutex_.unlock();  // UNLOCK ---------------------------
    return info;
}

}  // namespace joystick_interface