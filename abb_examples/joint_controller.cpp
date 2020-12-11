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

#include <abb_libegm/egm_controller_interface.h>
#include <iostream>

int main(int argc, char **argv) {
    //----------------------------------------------------------
    // Preparations
    //----------------------------------------------------------
    // Boost components for managing asynchronous UDP socket(s).
    boost::asio::io_service io_service;
    boost::thread_group thread_group;

    // Create an EGM interface:
    // * Sets up an EGM server (that the robot controller's EGM client can connect to).
    // * Provides APIs to the user (for setting motion references, that are sent in reply
    // to the EGM client's request).
    //
    // Note: It is important to set the correct port number here,
    //       as well as configuring the settings for the EGM client in the robot
    //       controller. If using the included RobotStudio Pack&Go file,
    //       then port 6510 = ROB_1
    abb::egm::EGMControllerInterface egm_interface(io_service, 6510);

    if (!egm_interface.isInitialized()) {
        std::cerr << "EGM interface failed to initialize (e.g. due to port already bound)"
                  << std::endl;
        return 0;
    }

    // Spin up a thread to run the io_service.
    thread_group.create_thread(boost::bind(&boost::asio::io_service::run, &io_service));

    //----------------------------------------------------------
    // Execute a joint controller loop.
    //
    // Note: The EGM communication session is started by the
    //       EGMRunJoint RAPID instruction.
    //----------------------------------------------------------
    std::cout << "========== Joint controller (open-loop) sample ==========" << std::endl;
    bool wait = true;
    abb::egm::wrapper::Input input;
    abb::egm::wrapper::Joints initial_positions;
    const int egm_rate = 250.0;  // [Hz] (EGM communication rate, specified by the
                                 // EGMActJoint RAPID instruction).
    int sequence_number = 0;     // [-] (sequence number of a received EGM message).
    double time = 0.0;  // [seconds] (elapsed time during an EGM communication session).

    abb::egm::wrapper::Output output;
    double reference = 0.0;   // [degrees].
    double amplitude = 20.0;  // [degrees].
    double frequency = 0.25;  // [Hz].

    std::cout << "1: Wait for an EGM communication session to start..." << std::endl;
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

    while (true) {
        // Wait for a new EGM message from the EGM client (with a timeout of 500 ms).
        if (egm_interface.waitForMessage(500)) {
            // Read the message received from the EGM client.
            egm_interface.read(&input);
            sequence_number = input.header().sequence_number();

            if (sequence_number == 0) {
                // Reset all references, if it is the first message.
                output.Clear();
                initial_positions.CopyFrom(input.feedback().robot().joints().position());
                output.mutable_robot()->mutable_joints()->mutable_position()->CopyFrom(
                    initial_positions);
            } else {
                time = sequence_number / ((double)egm_rate);

                if (initial_positions.values_size() > 1) {
                    // Compute reference for joint 1.
                    reference =
                        initial_positions.values(0) +
                        amplitude *
                            (1.0 + std::sin(2.0 * M_PI * frequency * time - 0.5 * M_PI));
                }

                if (output.mutable_robot()
                        ->mutable_joints()
                        ->mutable_position()
                        ->values_size() > 1) {
                    // Set reference for joint 1.
                    output.mutable_robot()
                        ->mutable_joints()
                        ->mutable_position()
                        ->set_values(0, reference);
                }

                if (sequence_number % egm_rate == 0) {
                    std::cout << "Reference: Joint 1 = " << reference << " [degrees]"
                              << std::endl;
                }
            }

            // Write references back to the EGM client.
            egm_interface.write(output);
        }
    }

    // Perform a clean shutdown.
    io_service.stop();
    thread_group.join_all();

    return 0;
}
