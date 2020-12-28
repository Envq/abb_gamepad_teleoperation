#include "simple_interface.hpp"
#include <abb_libegm/egm_controller_interface.h>
#include <boost/thread.hpp>
#include <iostream>
#include <stdexcept>



// CONFIGS ===============================================================================
const int PORT = 6510;
const double EGM_RATE = 250.0;
const bool TEST_REPLANNING = false;
const bool TEST_FEEDBACK = false;
const bool TEST_CONVERGENCE = false;
const bool TEST_WORKSPACE = true;



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

void delay(const double seconds) {
    boost::this_thread::sleep(boost::posix_time::milliseconds(seconds * 1000));
}



// MAIN
// ==================================================================================
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
        if (TEST_REPLANNING) {
            const int TIMEOUT = 400;
            auto target = initial_pose;
            std::cout << "Perform target1" << std::endl;
            target.x += 100;
            egm_ptr->sendPose(target);
            delay(4.0);
            std::cout << target << std::endl;
            std::cout << egm_ptr->waitForPose(400) << std::endl;


            std::cout << "Perform target2" << std::endl;
            target.x -= 100;
            egm_ptr->sendPose(target);
            delay(4.0);
            std::cout << target << std::endl;


            std::cout << "Perform target3" << std::endl;
            target.x += 100;
            egm_ptr->sendPose(target);
            delay(0.5);
            std::cout << target << std::endl;

            std::cout << "Perform target4" << std::endl;
            target.x -= 100;
            target.z -= 100;
            egm_ptr->sendPose(target);
            delay(4.0);
            std::cout << target << std::endl;
        }


        std::cout << "4: Test feedback" << std::endl;
        if (TEST_FEEDBACK) {
            const int TIMEOUT = 400;
            auto target = initial_pose;
            target.x += 100;
            egm_ptr->sendPose(target);
            while (true) {
                try {
                    std::cout << egm_ptr->waitForPose(TIMEOUT) << std::endl;
                } catch (simple_interface::EGMWarnException &warn) {  // catch timeout
                    std::cerr << warn.getInfo() << std::endl;
                }
            }
        }


        std::cout << "5: Test Convergence Criteria" << std::endl;
        if (TEST_CONVERGENCE) {
            const int TIMEOUT = 400;
            auto target = initial_pose;
            target.x += 100;
            egm_ptr->sendPose(target);
            while (true) {
                try {
                    egm_ptr->waitForPose(TIMEOUT);
                } catch (simple_interface::EGMWarnException &warn) {  // catch timeout
                    std::cerr << warn.getInfo() << std::endl;
                    break;
                }
            }
        }


        std::cout << "6: Test workspace" << std::endl;
        if (TEST_WORKSPACE) {
            const int TIMEOUT = 400;
            double offset = 0.0;
            double violation = 0.0;
            auto target = initial_pose;
            std::cout << "Set workspace" << std::endl;
            egm_ptr->setCubeWorkspace(initial_pose, 50);

            // std::cout << "Go to point0" << std::endl;
            // while (true) {
            //     target.y -= 1;
            //     target.x -= 1;
            //     target.z -= 1;
            //     if (egm_ptr->workspaceViolation(target)) {
            //         target.y += 1;  // rollback
            //         target.x += 1;
            //         target.z += 1;
            //         break;
            //     }
            //     egm_ptr->sendPose(target);
            //     delay(0.004);  // 250 hz
            // }
            // std::cout << egm_ptr->waitForPose(400) << std::endl;
            // std::cout << target << std::endl;

            std::cout << "Go to point0" << std::endl;
            offset = 10.0;
            violation = 0.0;
            while (true) {
                target = egm_ptr->waitForPose(TIMEOUT);
                target.y -= offset;
                target.x -= offset;
                target.z -= offset;
                if (egm_ptr->workspaceViolation(target, violation)) {
                    if (offset == 1.0)
                        break;
                    target.y += offset;  // rollback
                    target.x += offset;
                    target.z += offset;
                    offset = 1.0;
                    violation = 5.0;
                }
                egm_ptr->sendPose(target);
            }
            std::cout << egm_ptr->waitForPose(400) << std::endl;


            std::cout << "Go to point1" << std::endl;
            offset = 10.0;
            violation = 0.0;
            std::cout << offset << std::endl;
            while (true) {
                target = egm_ptr->waitForPose(TIMEOUT);
                target.x += offset;
                if (egm_ptr->workspaceViolation(target, violation)) {
                    if (offset == 1.0)
                        break;
                    target.x -= offset;  // rollback
                    offset = 1.0;
                    violation = 5.0;
                }
                egm_ptr->sendPose(target);
            }
            std::cout << egm_ptr->waitForPose(400) << std::endl;


            std::cout << "Go to point2" << std::endl;
            offset = 10.0;
            violation = 0.0;
            std::cout << offset << std::endl;
            while (true) {
                target = egm_ptr->waitForPose(TIMEOUT);
                target.y += offset;
                if (egm_ptr->workspaceViolation(target, violation)) {
                    if (offset == 1.0)
                        break;
                    target.y -= offset;  // rollback
                    offset = 1.0;
                    violation = 5.0;
                }
                egm_ptr->sendPose(target);
            }
            std::cout << egm_ptr->waitForPose(400) << std::endl;


            std::cout << "Go to point3" << std::endl;
            offset = 10.0;
            violation = 0.0;
            while (true) {
                target = egm_ptr->waitForPose(TIMEOUT);
                target.x -= offset;
                if (egm_ptr->workspaceViolation(target, violation)) {
                    if (offset == 1.0)
                        break;
                    target.x += offset;  // rollback
                    offset = 1.0;
                    violation = 5.0;
                }
                egm_ptr->sendPose(target);
            }
            std::cout << egm_ptr->waitForPose(400) << std::endl;


            std::cout << "Go to point4" << std::endl;
            offset = 10.0;
            violation = 0.0;
            while (true) {
                target = egm_ptr->waitForPose(TIMEOUT);
                target.y -= offset;
                if (egm_ptr->workspaceViolation(target, violation)) {
                    if (offset == 1.0)
                        break;
                    target.y += offset;  // rollback
                    offset = 1.0;
                    violation = 5.0;
                }
                egm_ptr->sendPose(target);
            }
            std::cout << egm_ptr->waitForPose(400) << std::endl;
        }
    } catch (simple_interface::EGMErrorException &err) {
        std::cout << err.getInfo() << std::endl;
        thread_group.remove_thread(egm_logger);
    }


    // SHUTDOWN
    // --------------------------------------------------------------------------
    io_service.stop();
    thread_group.join_all();
    return 0;
}