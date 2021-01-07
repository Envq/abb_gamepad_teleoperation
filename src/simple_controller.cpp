#include "simple_interface.hpp"
#include <iostream>



// CONFIGS ===============================================================================
const int PORT = 6510;
const double EGM_RATE = 250.0;



// AUXILIARY FUNCTION ===================================================================
namespace interpolation {
const double POSITION_AMPLITUDE = 45.0;      // [mm].
const double ORIENTATION_AMPLITUDE = -10.0;  // [degrees].
const double FREQUENCY = 0.25;               // [Hz].

using namespace simple_interface;
Pose perform_pose(Pose &initial_pose, const int time) {
    return {initial_pose.x +
                POSITION_AMPLITUDE *
                    (1.0 + std::sin(2.0 * M_PI * FREQUENCY * time - 0.5 * M_PI)),
            initial_pose.y,
            initial_pose.z,
            initial_pose.roll,
            initial_pose.pitch +
                ORIENTATION_AMPLITUDE *
                    (1.0 + std::sin(2.0 * M_PI * FREQUENCY * time - 0.5 * M_PI)),
            initial_pose.yaw};
}
}  // namespace interpolation



// MAIN ==================================================================================
int main(int argc, char **argv) {
    std::cout << "========== Simple Controller ==========" << std::endl;

    std::cout << "1: Initialize..." << std::endl;
    // Boost components for managing asynchronous UDP socket(s).
    boost::asio::io_service io_service;
    boost::thread_group thread_group;
    boost::shared_ptr<simple_interface::EGMInterface> egm_ptr;

    try {
        egm_ptr.reset(new simple_interface::EGMInterface(io_service, thread_group, PORT,
                                                         EGM_RATE, abb_robots::IRB_1100));

        std::cout << "2: Wait for an EGM communication session to start..." << std::endl;
        auto initial_pose = egm_ptr->waitConnection();

        std::cout << "3: Start control loop..." << std::endl;
        const int TIMEOUT = 400;  // [ms].
        int counter = 0;

        while (true) {
            try {
                // Get current pose
                auto pose = egm_ptr->waitForPose(TIMEOUT);
                std::cout << pose << std::endl;

                // Perform and update current target pose
                auto target = interpolation::perform_pose(initial_pose,
                                                          counter++ / ((double)EGM_RATE));

                // Send new pose
                egm_ptr->sendPose(target);

            } catch (simple_interface::EGMWarnException &warn) {  // catch timeout
                std::cerr << warn.getInfo() << std::endl;
                // Initialize interpolation
                counter = 0;
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