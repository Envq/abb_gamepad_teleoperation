#include <SDL2/SDL.h>
#include <boost/algorithm/string.hpp>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <iostream>

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

struct JoystickStatus {
    int axis0;  // Left x
    int axis1;  // Left y
    int axis2;  // Right x
    int axis3;  // Right y
    bool quit;

    std::string to_string() {
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

    static JoystickStatus parse(std::string &sample) {
        std::vector<std::string> info;
        boost::split(info, sample, boost::is_any_of("/"));
        return JoystickStatus{std::stoi(info[0]), std::stoi(info[1]), std::stoi(info[2]),
                              std::stoi(info[3]), static_cast<bool>(std::stoi(info[4]))};
    }
};

int main() {
    if (SDL_Init(SDL_INIT_JOYSTICK) < 0) {
        std::cerr << "Couldn't initialize SDL: " << SDL_GetError() << std::endl;
        exit(1);
    }

    int num_joysticks = SDL_NumJoysticks();
    std::cout << num_joysticks << " joysticks were found." << std::endl;
    if (num_joysticks == 0) {
        SDL_Quit();
        return 0;
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

        auto status =
            JoystickStatus{SDL_JoystickGetAxis(joy, 0), SDL_JoystickGetAxis(joy, 1),
                           SDL_JoystickGetAxis(joy, 2), SDL_JoystickGetAxis(joy, 3),
                           quit = (SDL_JoystickGetButton(joy, 9) != 0)};

        Sender(status.to_string());
        std::cout << status.to_string() << std::endl;
        fflush(stdout);
    }


    // Shutdown
    SDL_JoystickClose(joy);
    SDL_Quit();
    return 0;
}