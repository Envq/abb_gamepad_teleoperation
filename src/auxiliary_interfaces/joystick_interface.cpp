#include <joystick_interface.hpp>



namespace joystick_interface {


// JoystickStatus =======================================================================
std::string JoystickStatus::to_string() {
    std::string ret = std::to_string(axis0);
    ret.append("/");
    ret.append(std::to_string(axis1));
    ret.append("/");
    ret.append(std::to_string(axis2));
    ret.append("/");
    ret.append(std::to_string(axis3));
    ret.append("/");
    ret.append(std::to_string(quit));
    return ret;
}

JoystickStatus JoystickStatus::parse(std::string &sample) {
    std::vector<std::string> info;
    boost::split(info, sample, boost::is_any_of("/"));
    return JoystickStatus{std::stoi(info[0]), std::stoi(info[1]), std::stoi(info[2]),
                          std::stoi(info[3]), static_cast<bool>(std::stoi(info[4]))};
}



// JoystickInterface ====================================================================
JoystickInterface::JoystickInterface(boost::thread_group &thread_group) {
    thread_group.create_thread(boost::bind(&JoystickInterface::handle_joystick, this));
    // joystick_thread_ =
    //     new boost::thread(boost::bind(&JoystickInterface::handle_joystick, this));
}

JoystickInterface::~JoystickInterface() {
    // std::cout << "DELETE JOYSTICK INTERFACE" << std::endl;
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

        status_ =
            JoystickStatus{SDL_JoystickGetAxis(joy, 0), -SDL_JoystickGetAxis(joy, 1),
                           SDL_JoystickGetAxis(joy, 2), -SDL_JoystickGetAxis(joy, 3),
                           quit = (SDL_JoystickGetButton(joy, 9) != 0)};
        // std::cout << status_.to_string() << std::endl;
        // fflush(stdout);
    }

    // Shutdown
    SDL_JoystickClose(joy);
    SDL_Quit();
}


JoystickStatus JoystickInterface::read() {
    return status_;
}

}  // namespace joystick_interface