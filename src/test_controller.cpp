#include "simple_interface.hpp"
#include <abb_libegm/egm_controller_interface.h>
#include <boost/thread.hpp>
#include <iostream>
#include <stdexcept>



// CONFIGS ===============================================================================
const int PORT = 6510;
const double EGM_RATE = 250.0;



// AUXILIARY FUNCTIONS ===================================================================
void logger(const boost::shared_ptr<simple_interface::EGMInterface> &EGM,
            const int TIMEOUT) {
    while (true) {
        try {
            // Get current pose
            auto pose = EGM->waitForPose(500);
            std::cout << pose << std::endl;

        } catch (simple_interface::EGMException &warn) {  // catch timeout
            std::cerr << warn.getInfo() << std::endl;
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(TIMEOUT));
    }
}

void delay(const int ms) {
    boost::this_thread::sleep(boost::posix_time::milliseconds(ms));
}



// MAIN ==================================================================================
int main(int argc, char **argv) {
    std::cout << "========== Test Controller ==========" << std::endl;

    std::cout << "1: Initialize..." << std::endl;
    boost::asio::io_service io_service;
    boost::thread_group thread_group;
    boost::thread *egm_logger;
    boost::shared_ptr<simple_interface::EGMInterface> egm_ptr;


    try {
        egm_ptr.reset(new simple_interface::EGMInterface(io_service, thread_group, PORT,
                                                         EGM_RATE, abb_robots::IRB_1100));

        // std::cout << "X: Launch logger thread..." << std::endl;
        // thread_group.add_thread(egm_logger = new boost::thread(&logger, egm_ptr,
        // 2000));


        std::cout << "2: Wait for an EGM communication session to start..." << std::endl;
        auto initial_pose = egm_ptr->waitConnection();
        std::cout << "Initial Pose:" << std::endl;
        std::cout << initial_pose << std::endl;


        std::cout << "3: Test replanning" << std::endl;
        std::cout << "Perform target1" << std::endl;
        auto target = initial_pose;
        target.x += 100;
        egm_ptr->sendPose(target);
        delay(4000);
        std::cout << target << std::endl;

        std::cout << "\tPerform target2" << std::endl;
        target.x -= 100;
        egm_ptr->sendPose(target);
        delay(4000);
        std::cout << target << std::endl;

        std::cout << "\tPerform target3" << std::endl;
        target.x += 100;
        egm_ptr->sendPose(target);
        delay(500);
        std::cout << target << std::endl;

        std::cout << "\tPerform target4" << std::endl;
        target.x -= 100;
        target.z -= 100;
        egm_ptr->sendPose(target);
        delay(4000);
        std::cout << target << std::endl;


    } catch (simple_interface::EGMErrorException &err) {
        std::cout << err.getInfo() << std::endl;
        thread_group.remove_thread(egm_logger);
    }


    // SHUTDOWN --------------------------------------------------------------------------
    io_service.stop();
    thread_group.join_all();
    return 0;
}