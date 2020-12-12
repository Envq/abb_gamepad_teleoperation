#include <abb_libegm/egm_controller_interface.h>
#include <iostream>

int main(int argc, char **argv) {
    std::cout << "========== Pose controller (open-loop) sample ==========" << std::endl;


    // INITIALIZE ########################################################################
    std::cout << "1: Initialize..." << std::endl;
    // Boost components for managing asynchronous UDP socket(s).
    boost::asio::io_service io_service;
    boost::thread_group thread_group;

    // Create an EGM interface:
    // * Sets up an EGM server (that the robot controller's EGM client can connect to).
    // * Provides APIs to the user (for setting motion references, that are sent in reply
    // to the EGM client's request).
    abb::egm::EGMControllerInterface egm_interface(io_service, 6510);
    if (!egm_interface.isInitialized()) {
        std::cerr << "EGM interface failed to initialize (e.g. due to port already bound)"
                  << std::endl;
        return 0;
    }

    // Spin up a thread to run the io_service.
    thread_group.create_thread(boost::bind(&boost::asio::io_service::run, &io_service));


    // CHECK CONNECTION ##################################################################
    std::cout << "2: Wait for an EGM communication session to start..." << std::endl;
    while (true) {
        using namespace abb::egm::wrapper;
        if (egm_interface.isConnected()) {
            auto status = egm_interface.getStatus().rapid_execution_state();
            if (status == Status_RAPIDExecutionState_RAPID_UNDEFINED)
                std::cout
                    << "RAPID execution state is UNDEFINED (might happen first time "
                       "after controller start/restart). Try to restart the RAPID "
                       "program."
                    << std::endl;
            else if (status == Status_RAPIDExecutionState_RAPID_RUNNING)
                break;
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
    }


    // JOINT CONTROLLER LOOP #############################################################
    // Constants
    const int NUM_JOINT = 6;   // [-] (number of joint of the robot)
    const int EGM_RATE = 250;  // [Hz] (EGM communication rate, specified by EGMActJoint).
    const double AMPLITUDE = 20.0;  // [degrees].
    const double FREQUENCY = 0.25;  // [Hz].

    // Variables
    abb::egm::wrapper::Input input;
    abb::egm::wrapper::Joints initial_joints;
    abb::egm::wrapper::Joints &input_joints =
        *input.mutable_feedback()->mutable_robot()->mutable_joints()->mutable_position();
    abb::egm::wrapper::Output output;
    abb::egm::wrapper::Joints &output_joints =
        *output.mutable_robot()->mutable_joints()->mutable_position();
    int sequence_number = 0;  // [-] (sequence number of a received EGM message).
    double time = 0.0;        // [s] (elapsed time during an EGM communication session).
    double reference = 0.0;   // [degrees].


    // Read the message received from the EGM client for get current sequence number
    egm_interface.read(&input);
    int seq_num_offset = 1 + input.header().sequence_number();

    // Controller loop
    while (true) {
        // Wait for a new EGM message from the EGM client (with a timeout in ms).
        // Note: the message should arrive in about 1/EGM_RATE seconds.
        if (egm_interface.waitForMessage(400)) {
            // Read the message received from the EGM client
            egm_interface.read(&input);
            sequence_number = input.header().sequence_number() - seq_num_offset;

            // Reset all references, if it is the first message.
            if (sequence_number == 0) {
                // Get initial joints values
                initial_joints.CopyFrom(input_joints);
                if (initial_joints.values_size() != NUM_JOINT) {
                    std::cerr << "Client robot used wrong. Use a robot with " << NUM_JOINT
                              << " joints." << std::endl;
                    break;
                }
                // Prepare outputs message
                output.Clear();
                output_joints.CopyFrom(initial_joints);

            } else {
                // Adjust time for manage the offset
                time = sequence_number / ((double)EGM_RATE);  // t = seq_n * 1/f
                // Compute reference for joint.
                reference =
                    initial_joints.values(0) +
                    AMPLITUDE *
                        (1.0 + std::sin(2.0 * M_PI * FREQUENCY * time - 0.5 * M_PI));

                // Compute new joints reference for output
                // output_joints.set_values(0, reference);
                output.mutable_robot()->mutable_joints()->mutable_position()->set_values(
                    0, reference);
            }

            // Write references back to the EGM client.
            egm_interface.write(output);

            // Print info
            if (sequence_number % (1 * EGM_RATE) == 0)
                std::cout << "[" << sequence_number
                          << "] Reference: Joint 1 = " << reference << " [degrees]"
                          << std::endl;

        } else {
            // Timeout info
            std::cerr << "TIMEOUT" << std::endl;
        }
    }


    // SHUTDOWN ##########################################################################
    io_service.stop();
    thread_group.join_all();
    return 0;
}