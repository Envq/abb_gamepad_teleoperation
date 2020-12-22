#include "simple_interface.hpp"
#include <iostream>
#include <termios.h>
#include <unistd.h>



// CONFIGS ===============================================================================
const int PORT = 6510;
const double EGM_RATE = 250.0;



// AUXILIARY FUNCTIONS ===================================================================
namespace keyboard {
const int X_POS = 'd';
const int X_NEG = 'a';
const int Y_POS = 'w';
const int Y_NEG = 's';
const int EXIT = 'q';
int command;

int getch() {
    int ch;
    struct termios oldt;
    struct termios newt;
    tcgetattr(STDIN_FILENO, &oldt); /*store old settings */
    newt = oldt;                    /* copy old settings to new settings */
    newt.c_lflag &=
        ~(ICANON | ECHO); /* make one change to old settings in new settings */
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); /*apply the new settings immediatly */
    ch = getchar();                          /* standard getchar call */
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); /*reapply the old settings */
    return ch;                               /*return received char */
}

void catcher() {
    while (true) {
        command = keyboard::getch();
    }
}
}  // namespace keyboard



// MAIN ==================================================================================
int main(int argc, char **argv) {
    std::cout << "========== Keyboard controller ==========" << std::endl;
    // Boost components for managing asynchronous UDP socket(s).
    boost::asio::io_service io_service;
    boost::thread_group thread_group;
    auto keyboard_catcher = new boost::thread(&keyboard::catcher);

    try {
        std::cout << "1: Initialize..." << std::endl;
        simple_interface::EGMInterface egm = simple_interface::EGMInterface(
            io_service, thread_group, PORT, EGM_RATE, abb_robots::IRB_1100);

        std::cout << "2: Wait for an EGM communication session to start..." << std::endl;
        auto initial_pose = egm.waitConnection();
        auto current_pose = initial_pose;

        std::cout << "3: Launch keyboard reader thread..." << std::endl;
        thread_group.add_thread(keyboard_catcher);

        std::cout << "4: Start control loop..." << std::endl;
        double offset = 10;  // [mm].
        int timeout = 400;   //[ms].

        while (true) {
            // Get current pose
            auto pose = egm.waitForPose(timeout);
            std::cout << pose << std::endl;

            // Perform new pose
            if (keyboard::command == keyboard::X_POS)
                pose.x += offset;
            else if (keyboard::command == keyboard::X_NEG)
                pose.x -= offset;
            else if (keyboard::command == keyboard::Y_POS)
                pose.y += offset;
            else if (keyboard::command == keyboard::Y_NEG)
                pose.y -= offset;
            else if (keyboard::command == keyboard::EXIT)
                break;

            // Send new pose
            egm.sendPose(pose);
        }

    } catch (simple_interface::EGMException &err) {  // catch timeout and port error
        std::cerr << "\t" << err.getInfo() << std::endl;
        thread_group.remove_thread(keyboard_catcher);
    }

    // Clean Shutdown
    io_service.stop();
    thread_group.join_all();
    return 0;
}
