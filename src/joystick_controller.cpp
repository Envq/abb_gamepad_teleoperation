#include "joystick_interface.hpp"
#include "simple_interface.hpp"
#include <iostream>



// CONFIGS ===============================================================================
const int EGM_PORT = 6510;
const int JOY_PORT = 6511;
const double EGM_RATE = 250.0;  // [hz]
const int JOY_RANGE = 50;       // [mm]
int WS_RANGE = 50;              // [mm]



// MAIN ==================================================================================
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
        auto initial_pose = egm_ptr->waitConnection();

        std::cout << "3: Start control loop..." << std::endl;
        const int TIMEOUT = 400;  // [ms]
        egm_ptr->setWorkspace(initial_pose, WS_RANGE);

        while (true) {
            try {
                // Get Joystick status
                auto joy_status = joy_ptr->read();
                // std::cout << joy_status.to_string() << std::endl;

                // Check if quit
                if (joy_status.quit)
                    break;

                // Get current pose
                auto pose = egm_ptr->waitForPose(TIMEOUT);
                std::cout << pose << std::endl;

                // Perform and update current target pose
                auto target = initial_pose;
                target.x += (joy_status.axis0 / joy_ptr->getJoystickMaxVal()) * JOY_RANGE;
                target.y += (joy_status.axis1 / joy_ptr->getJoystickMaxVal()) * JOY_RANGE;
                target.z += (joy_status.axis3 / joy_ptr->getJoystickMaxVal()) * JOY_RANGE;

                // Send new pose
                auto corr = egm_ptr->sendSafePose(target);

                // Notify correction
                if (corr != target)
                    std::cout << "WORKSPACE VIOLATION: correction occured. " << std::endl;

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