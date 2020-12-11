/***********************************************************************************************************************
 *
 * Copyright (c) 2018, ABB Schweiz AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with
 * or without modification, are permitted provided that
 * the following conditions are met:
 *
 *    * Redistributions of source code must retain the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer.
 *    * Redistributions in binary form must reproduce the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer in the documentation
 *      and/or other materials provided with the
 *      distribution.
 *    * Neither the name of ABB nor the names of its
 *      contributors may be used to endorse or promote
 *      products derived from this software without
 *      specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***********************************************************************************************************************
 */

#include <abb_libegm/egm_trajectory_interface.h>
#include <iostream>

// Function to check if two joint messages are close to each other.
bool jointsCloseby(abb::egm::wrapper::Joints j1, abb::egm::wrapper::Joints j2) {
    bool result = j1.values_size() > 0 && j2.values_size() > 0;
    double threshold = 0.01;  // [degrees]

    for (int i = 0; i < j1.values_size() && i < j2.values_size() && result; ++i) {
        result = std::abs(j1.values(i) - j2.values(i)) < threshold;
    }

    return result;
}

int main(int argc, char **argv) {
    //----------------------------------------------------------
    // Preparations
    //----------------------------------------------------------
    // Initialize the node.
    // Boost components for managing asynchronous UDP socket(s).
    boost::asio::io_service io_service;
    boost::thread_group thread_group;

    // Create an EGM interface:
    // * Sets up an EGM server (that the robot controller's EGM client can connect to).
    // * Provides APIs to the user (for setting motion references, that are sent in reply
    // to the EGM client's request).
    //
    // Note: It is important to set the correct port number here,
    //       as well as configuring the settings for the EGM client in thre robot
    //       controller. If using the included RobotStudio Pack&Go file, then port 6511 =
    //       then port 6510 = ROB_1
    abb::egm::EGMTrajectoryInterface egm_interface(io_service, 6510);

    if (!egm_interface.isInitialized()) {
        std::cerr << "EGM interface failed to initialize (e.g. due to port already bound)"
                  << std::endl;
        return 0;
    }

    // Spin up a thread to run the io_service.
    thread_group.create_thread(boost::bind(&boost::asio::io_service::run, &io_service));

    //----------------------------------------------------------
    // Execute joint static goal.
    //
    // Note: The EGM communication session is started by the
    //       EGMRunJoint RAPID instruction.
    //----------------------------------------------------------
    std::cout << "========== Joint static goal sample ==========" << std::endl;
    bool wait = true;
    abb::egm::wrapper::trajectory::ExecutionProgress execution_progress;
    abb::egm::wrapper::trajectory::StaticPositionGoal static_goal_1;
    abb::egm::wrapper::trajectory::StaticPositionGoal static_goal_2;
    abb::egm::wrapper::trajectory::StaticPositionGoal static_goal_3;
    abb::egm::wrapper::trajectory::TrajectoryGoal trajectory;
    abb::egm::wrapper::trajectory::PointGoal *p_point;

    // Create static goals.
    static_goal_1.mutable_robot()->mutable_joints()->add_values(0.0);
    static_goal_1.mutable_robot()->mutable_joints()->add_values(0.0);
    static_goal_1.mutable_robot()->mutable_joints()->add_values(-50.0);
    static_goal_1.mutable_robot()->mutable_joints()->add_values(0.0);
    static_goal_1.mutable_robot()->mutable_joints()->add_values(30.0);
    static_goal_1.mutable_robot()->mutable_joints()->add_values(0.0);

    static_goal_2.mutable_robot()->mutable_joints()->add_values(50.0);
    static_goal_2.mutable_robot()->mutable_joints()->add_values(0.0);
    static_goal_2.mutable_robot()->mutable_joints()->add_values(-50.0);
    static_goal_2.mutable_robot()->mutable_joints()->add_values(0.0);
    static_goal_2.mutable_robot()->mutable_joints()->add_values(30.0);
    static_goal_2.mutable_robot()->mutable_joints()->add_values(0.0);

    static_goal_3.mutable_robot()->mutable_joints()->add_values(0.0);
    static_goal_3.mutable_robot()->mutable_joints()->add_values(0.0);
    static_goal_3.mutable_robot()->mutable_joints()->add_values(0.0);
    static_goal_3.mutable_robot()->mutable_joints()->add_values(0.0);
    static_goal_3.mutable_robot()->mutable_joints()->add_values(30.0);
    static_goal_3.mutable_robot()->mutable_joints()->add_values(0.0);

    // Create a trajectory.
    p_point = trajectory.add_points();
    p_point->set_reach(true);
    p_point->set_duration(5.0);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(100.0);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(30.0);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);

    p_point = trajectory.add_points();
    p_point->set_duration(5.0);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(30.0);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);

    //-----------------------------
    // Execute static goals.
    //-----------------------------
    std::cout << "Example 1: Execute static goals" << std::endl;
    std::cout << "1.1: Wait for an EGM communication session to start..." << std::endl;
    while (wait) {
        if (egm_interface.isConnected()) {
            if (egm_interface.getStatus().rapid_execution_state() ==
                abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_UNDEFINED) {
                std::cout
                    << "RAPID execution state is UNDEFINED (might happen first time "
                       "after "
                       "controller start/restart). Try to restart the RAPID program."
                    << std::endl;
            } else {
                wait = egm_interface.getStatus().rapid_execution_state() !=
                       abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_RUNNING;
            }
        }

        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
    }

    std::cout << "1.2: Wait for the interface to enter normal state..." << std::endl;
    while (wait) {
        if (egm_interface.retrieveExecutionProgress(&execution_progress)) {
            wait = !(execution_progress.state() ==
                     abb::egm::wrapper::trajectory::ExecutionProgress_State_NORMAL);
        }

        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
    }

    if (true) {
        std::cout << "1.3: Start static goal execution" << std::endl;
        egm_interface.startStaticGoal();

        std::cout << "1.4: Set static goal" << std::endl;
        egm_interface.setStaticGoal(static_goal_1);

        std::cout << "1.5: Press Enter to change static goal (with fast transition)"
                  << std::endl;
        std::cin.get();
        egm_interface.setStaticGoal(static_goal_2, true);

        std::cout << "1.6: Press Enter to change static goal (with fast transition)"
                  << std::endl;
        std::cin.get();
        egm_interface.setStaticGoal(static_goal_3, true);
    }

    std::cout << "1.7: Wait for the robot to reach the static goal..." << std::endl;
    wait = true;
    while (wait) {
        if (egm_interface.retrieveExecutionProgress(&execution_progress)) {
            wait = !jointsCloseby(
                static_goal_3.robot().joints(),
                execution_progress.inputs().feedback().robot().joints().position());
        }

        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
    }

    if (true) {
        std::cout << "1.8: Finish static goal execution" << std::endl;
        egm_interface.finishStaticGoal(true);
    }

    std::cout << "1.9: Wait for the interface to enter normal state... " << std::endl;
    wait = true;
    while (wait) {
        if (egm_interface.retrieveExecutionProgress(&execution_progress)) {
            wait = !(execution_progress.state() ==
                     abb::egm::wrapper::trajectory::ExecutionProgress_State_NORMAL);
        }

        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
    }

    //-----------------------------
    // Execute a static goal,
    // during trajectory execution.
    //-----------------------------
    std::cout << "========== " << std::endl;
    std::cout << "Example 2: Execute a static goal, during trajectory execution "
              << std::endl;

    if (true) {
        std::cout << "2.1: Add a joint trajectory to the execution queue " << std::endl;
        egm_interface.addTrajectory(trajectory);
        boost::this_thread::sleep(boost::posix_time::milliseconds(3500));

        std::cout << "2.2: Press Enter to start static goal execution " << std::endl;
        std::cin.get();
        egm_interface.startStaticGoal();

        std::cout << "2.3: Set static goal " << std::endl;
        egm_interface.setStaticGoal(static_goal_2);
    }

    std::cout << "2.4: Wait for the robot to reach the static goal... " << std::endl;
    wait = true;
    while (wait) {
        if (egm_interface.retrieveExecutionProgress(&execution_progress)) {
            wait = !jointsCloseby(
                static_goal_2.robot().joints(),
                execution_progress.inputs().feedback().robot().joints().position());
        }

        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
    }

    if (true) {
        std::cout
            << "2.5: Finish static goal execution (and resume the trajectory execution) "
            << std::endl;
        egm_interface.finishStaticGoal(true);
    }

    std::cout << "2.6: Wait for the trajectory execution to finish... " << std::endl;
    wait = true;
    while (wait) {
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));

        if (egm_interface.retrieveExecutionProgress(&execution_progress)) {
            wait = execution_progress.goal_active();
        }
    }

    // Perform a clean shutdown.
    io_service.stop();
    thread_group.join_all();

    return 0;
}
