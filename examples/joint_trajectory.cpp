/***********************************************************************************************************************
 *
 * Copyright (c) 2015, ABB Schweiz AG
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

int main(int argc, char **argv) {
    //----------------------------------------------------------
    // Preparations
    //----------------------------------------------------------
    // Boost components for managing asynchronous UDP socket(s).
    boost::asio::io_service io_service;
    boost::thread_group thread_group;

    // Create an EGM interface:
    // * Sets up an EGM server (that the robot controller's EGM client can
    // connect to).
    // * Provides APIs to the user (for setting motion references, that are sent
    // in reply to the EGM client's request).
    //
    // Note: It is important to set the correct port number here,
    //       as well as configuring the settings for the EGM client in the robot
    //       controller. If using the included RobotStudio Pack&Go file,
    //       then port 6510 = ROB_1
    abb::egm::EGMTrajectoryInterface egm_interface(io_service, 6510);

    if (!egm_interface.isInitialized()) {
        std::cerr << "EGM interface failed to initialize (e.g. due to port "
                     "already bound)"
                  << std::endl;
        return 0;
    }

    // Spin up a thread to run the io_service.
    thread_group.create_thread(boost::bind(&boost::asio::io_service::run, &io_service));

    //----------------------------------------------------------
    // Execute joint trajectories.
    //
    // Note: The EGM communication session is started by the
    //       EGMRunJoint RAPID instruction.
    //----------------------------------------------------------
    std::cout << "========== Joint trajectory sample ==========" << std::endl;
    bool wait = true;
    abb::egm::wrapper::trajectory::TrajectoryGoal trajectory_1;
    abb::egm::wrapper::trajectory::TrajectoryGoal trajectory_2;
    abb::egm::wrapper::trajectory::PointGoal *p_point;

    // Add target points to the trajectories.
    //
    // Note: Reach [-] (specifies if it is important to reach the point or not).
    //       Desired duration [seconds] (the actual time depends on e.g. the EGM
    //       speed limits set in the RAPID code). Joint values [degrees].


    const double time = 2.5;
    // Trajectory 1.
    p_point = trajectory_1.add_points();
    p_point->set_reach(true);
    p_point->set_duration(time);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(-50.0);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(30.0);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);

    p_point = trajectory_1.add_points();
    p_point->set_duration(time);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(30.0);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(30.0);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);

    // Trajectory 2.
    p_point = trajectory_2.add_points();
    p_point->set_duration(time);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(-50.0);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(30.0);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);

    p_point = trajectory_2.add_points();
    p_point->set_duration(time);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(50.0);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(30.0);
    p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);

    std::cout << "1: Wait for an EGM communication session to start..." << std::endl;
    while (wait) {
        if (egm_interface.isConnected()) {
            if (egm_interface.getStatus().rapid_execution_state() ==
                abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_UNDEFINED) {
                std::cout << "RAPID execution state is UNDEFINED (might happen "
                             "first time after controller start/restart). Try "
                             "to restart the RAPID program."
                          << std::endl;
            } else {
                wait = egm_interface.getStatus().rapid_execution_state() !=
                       abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_RUNNING;
            }
        }


        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
    }

    std::cout << "2: Add joint trajectories to the execution queue" << std::endl;
    egm_interface.addTrajectory(trajectory_1);
    egm_interface.addTrajectory(trajectory_2);

    std::cout << "3: Wait for the trajectory execution to finish..." << std::endl;
    abb::egm::wrapper::trajectory::ExecutionProgress execution_progress;
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
