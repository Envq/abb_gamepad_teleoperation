#include "colors.hpp"
#include "joystick_interface.hpp"
#include "simple_interface.hpp"
#include <iostream>



// CONFIGS ===============================================================================
const int EGM_PORT = 6510;
const int JOY_PORT = 6511;
const double EGM_RATE = 250.0;  // [hz]
int WS_RANGE = 50;              // [mm]
const int EGM_TIMEOUT = 400;    // [ms]
const Colors NOTIFY_COLOR = Colors::FG_CYAN;



// MAIN
// ==================================================================================
int main(int argc, char **argv) {
    std::cout << "========== Simple Controller ==========" << std::endl;
    std::cout << "1: Initialize..." << std::endl;
    // Boost components for managing asynchronous UDP socket(s).
    boost::asio::io_service io_service;
    boost::thread_group thread_group;
    boost::shared_ptr<simple_interface::EGMInterface> egm_ptr;
    boost::shared_ptr<joystick_interface::JoystickInterface> joy_ptr;
    try {
        WS_RANGE = (argc > 1) ? std::stoi(argv[1]) : WS_RANGE;
        std::cout << "Workspace cube range: " << WS_RANGE << std::endl;

    } catch (std::invalid_argument &err) {
        std::cout << "INVALID ARGUMENT -> abort" << std::endl;
        io_service.stop();
        thread_group.join_all();
        return 0;
    }


    try {
        egm_ptr.reset(new simple_interface::EGMInterface(
            io_service, thread_group, EGM_PORT, EGM_RATE, abb_robots::IRB_1100));

        joy_ptr.reset(new joystick_interface::JoystickInterface(thread_group));

        std::cout << "2: Wait for an EGM communication session to start..." << std::endl;
        auto origin = egm_ptr->waitConnection();
        egm_ptr->setWorkspace(origin, WS_RANGE);

        std::cout << "3: Start control loop..." << std::endl;
        while (true) {
            try {
                // Get current pose (implicit sync to EGM rate)
                auto current_pose = egm_ptr->waitForPose(EGM_TIMEOUT);
                std::cout << colorize("Current Pose: ", NOTIFY_COLOR) << std::endl;
                std::cout << current_pose << std::endl;

                // Get Joystick info
                auto joy_info = joy_ptr->readInfo();
                // std::cout << joy_info.toString() << std::endl;

                // Check if quit
                if (joy_info.quit)
                    break;

                // Reset Origin
                if (joy_info.setOrigin) {
                    origin = current_pose;
                    egm_ptr->setWorkspace(origin, WS_RANGE);
                }
                std::cout << colorize("Origin Pose: ", NOTIFY_COLOR) << std::endl;
                std::cout << origin << std::endl;

                // Notify Stretch
                std::cout << colorize("Stretch: ", NOTIFY_COLOR) << joy_info.stretch
                          << std::endl;

                // Perform and update current target pose
                auto target = origin + joy_info.pose;

                // Send new pose
                auto corr = egm_ptr->sendSafePose(target);

                // Notify correction
                if (corr != target)
                    std::cout << colorize("WORKSPACE VIOLATION: correction occured.",
                                          NOTIFY_COLOR)
                              << std::endl;

                std::cout << "============================================" << std::endl;

            } catch (simple_interface::EGMWarnException &warn) {  // catch timeout
                std::cerr << warn.getInfo() << std::endl;
            }
        }
    } catch (simple_interface::EGMErrorException &err) {
        std::cout << err.getInfo() << std::endl;

    } catch (joystick_interface::JoyErrorException &err) {
        std::cout << err.getInfo() << std::endl;
    }


    std::cout << "4: Shutdown..." << std::endl;
    // Clean Shutdown
    io_service.stop();
    thread_group.join_all();
    return 0;
}