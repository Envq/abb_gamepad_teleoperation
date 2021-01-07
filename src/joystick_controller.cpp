#include "joystick_interface.hpp"
#include "simple_interface.hpp"
#include <iostream>



// CONFIGS ===============================================================================
const int EGM_PORT = 6510;
const int JOY_PORT = 6511;
const double EGM_RATE = 250.0;



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
        // egm_ptr.reset(new simple_interface::EGMInterface(
        //     io_service, thread_group, EGM_PORT, EGM_RATE, abb_robots::IRB_1100));

        joy_ptr.reset(new joystick_interface::JoystickInterface(io_service, thread_group,
                                                                JOY_PORT));

        // std::cout << "2: Wait for an EGM communication session to start..." <<
        // std::endl; auto initial_pose = egm_ptr->waitConnection();

        // std::cout << "3: Start control loop..." << std::endl;
        // const int TIMEOUT = 400;  // [ms].

        while (true) {
            try {
                // Get current pose
                // auto pose = egm_ptr->waitForPose(TIMEOUT);
                // std::cout << pose << std::endl;

                // Perform and update current target pose
                // auto target = initial_pose;
                auto joy_status = joy_ptr->read();
                std::cout << joy_status.to_string() << std::endl;

                if (joy_status.quit)
                    break;

                // switch (joy_info.axis) {
                // case 0:
                //     target.x += joy_status.value;
                //     break;

                // case 1:
                //     target.y += joy_status.value;
                //     break;

                // case 3:
                //     target.z += joy_status.value;
                //     break;

                // default:
                //     break;
                // }

                // Send new pose
                // egm_ptr->sendPose(target);
                // boost::this_thread::sleep(boost::posix_time::seconds(1));

            } catch (simple_interface::EGMWarnException &warn) {  // catch timeout
                std::cerr << warn.getInfo() << std::endl;
            }
        }
    } catch (simple_interface::EGMErrorException &err) {
        std::cout << err.getInfo() << std::endl;
    }

    // Clean Shutdown
    io_service.stop();
    thread_group.join_all();
    return 0;
}