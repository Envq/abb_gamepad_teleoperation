#include "simple_interface.hpp"
#include <iostream>
#include <stdexcept>



// MAIN
// ==================================================================================
int main(int argc, char **argv) {
    std::cout << "========== Joystick controller ==========" << std::endl;
    // Boost components for managing asynchronous UDP socket(s).
    boost::asio::io_service io_service;
    boost::thread_group thread_group;

    try {
        std::cout << "1: Initialize..." << std::endl;
        simple_interface::EGMInterface egm =
            simple_interface::EGMInterface(io_service, thread_group, 6510);

        std::cout << "2: Wait for an EGM communication session to start..." << std::endl;
        egm.waitConnection();

        std::cout << "3: Start control loop..." << std::endl;
        double offset = 10;  // [mm].
        int timeout = 400;   //[ms].

        // Get initial pose
        auto initial_pose = egm.waitForPose(timeout);
        auto current_pose = initial_pose;

        while (true) {
            // Get current pose
            auto pose = egm.waitForPose(timeout);
            std::cout << pose << std::endl;

            // Update current pose
            current_pose.x += offset;

            // Send new pose
            egm.sendPose(current_pose);
        }

    } catch (std::runtime_error &err) {
        std::cerr << err.what() << std::endl;
    }


    // Clean Shutdown
    io_service.stop();
    thread_group.join_all();
    return 0;
}