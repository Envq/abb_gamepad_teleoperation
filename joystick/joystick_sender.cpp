#include <SDL2/SDL.h>
#include <boost/algorithm/string.hpp>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <thread>

#define IPADDRESS "127.0.0.1"
#define UDP_PORT 6511

using boost::asio::ip::address;
using boost::asio::ip::udp;

void Sender(std::string in) {
    boost::asio::io_service io_service;
    udp::socket socket(io_service);
    udp::endpoint remote_endpoint =
        udp::endpoint(address::from_string(IPADDRESS), UDP_PORT);
    socket.open(udp::v4());

    boost::system::error_code err;
    auto sent = socket.send_to(boost::asio::buffer(in), remote_endpoint, 0, err);
    socket.close();
}

struct JoystickInfo {
    int axis;
    int value;
    bool quit;

    std::string to_string() {
        std::string ret = std::to_string(axis);
        ret.append("/");
        ret.append(std::to_string(value));
        ret.append("/");
        ret.append(std::to_string(quit));
        return ret;
    }

    static JoystickInfo parse(std::string &sample) {
        std::vector<std::string> info;
        boost::split(info, sample, boost::is_any_of("/"));
        return JoystickInfo{std::stoi(info[0]), std::stoi(info[1]),
                            static_cast<bool>(std::stoi(info[2]))};
    }
};

int main() {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK) < 0) {
        std::cerr << "Couldn't initialize SDL: " << SDL_GetError() << std::endl;
        exit(1);
    }

    int num_joysticks = SDL_NumJoysticks();
    std::cout << num_joysticks << " joysticks were found." << std::endl;
    if (num_joysticks == 0) {
        SDL_Quit();
        return 0;
    }


    SDL_Joystick *joystick;
    SDL_JoystickEventState(SDL_ENABLE);
    joystick = SDL_JoystickOpen(0);

    SDL_Event event;
    while (true) {
        while (SDL_PollEvent(&event)) {
            switch (event.type) {
            case SDL_QUIT:
                SDL_JoystickClose(joystick);
                SDL_Quit();
                return 0;

            case SDL_JOYBUTTONDOWN:
                std::cout << "Pressed button: " << (int)event.jbutton.button << std::endl;
                if (event.jbutton.button == 9) {
                    Sender(JoystickInfo{0, 0, true}.to_string());
                    SDL_JoystickClose(joystick);
                    SDL_Quit();
                    return 0;
                }

                break;

            case SDL_JOYAXISMOTION:
                std::cout << "Axis" << (int)event.jaxis.axis << ": " << event.jaxis.value
                          << std::endl;
                Sender(
                    JoystickInfo{event.jaxis.axis, event.jaxis.value, false}.to_string());

                break;
            }

            fflush(stdout);
        }
    }

    // Shutdown
    SDL_JoystickClose(joystick);
    SDL_Quit();
    return 0;
}