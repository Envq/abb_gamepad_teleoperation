#include <abb_libegm/egm_controller_interface.h>
#include <iostream>



// CONFIGS ===============================================================================
const int PORT = 6510;
const double EGM_RATE = 250.0;



// MAIN ==================================================================================
int main(int argc, char **argv) {
    std::cout << "========== Pose controller (open-loop) sample ==========" << std::endl;

    // INITIALIZE -----------------------------------------------------------------------
    std::cout << "1: Initialize..." << std::endl;
    // Boost components for managing asynchronous UDP socket(s).
    boost::asio::io_service io_service;
    boost::thread_group thread_group;

    // Create an EGM interface:
    // * Sets up an EGM server (that the robot controller's EGM client can connect to).
    // * Provides APIs to the user (for setting motion references, that are sent in reply
    // to the EGM client's request).
    abb::egm::EGMControllerInterface egm_interface(io_service, PORT);
    if (!egm_interface.isInitialized()) {
        std::cerr << "EGM interface failed to initialize (e.g. due to port already bound)"
                  << std::endl;
        return 0;
    }

    // Spin up a thread to run the io_service.
    thread_group.create_thread(boost::bind(&boost::asio::io_service::run, &io_service));


    // CHECK CONNECTION  -----------------------------------------------------------------
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


    // JOINT CONTROLLER LOOP  ------------------------------------------------------------
    // Constants
    const double POSITION_AMPLITUDE = 45.0;      // [mm].
    const double ORIENTATION_AMPLITUDE = -10.0;  // [degrees].
    const double FREQUENCY = 0.25;               // [Hz].

    // Variables
    abb::egm::wrapper::Input input;
    abb::egm::wrapper::CartesianPose initial_pose;
    abb::egm::wrapper::CartesianPose &input_pose =
        *input.mutable_feedback()->mutable_robot()->mutable_cartesian()->mutable_pose();
    abb::egm::wrapper::Output output;
    int sequence_number = 0;  // [-] (sequence number of a received EGM message).
    double time = 0.0;        // [s] (elapsed time during an EGM communication session).
    double position_reference = 0.0;     // [mm].
    double orientation_reference = 0.0;  // [degrees].


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
                initial_pose.CopyFrom(input_pose);
                // Prepare outputs message
                output.Clear();
                output.mutable_robot()->mutable_cartesian()->mutable_pose()->CopyFrom(
                    initial_pose);
            } else {
                // Adjust time for manage the offset
                time = sequence_number / ((double)EGM_RATE);  // t = seq_n * 1/f
                // Compute references for position e orientation
                const bool SIMPLE = false;  // true for linear motion up to singularity
                if (SIMPLE) {
                    position_reference =
                        initial_pose.position().x() + 10 * sequence_number;
                    orientation_reference = initial_pose.euler().y();
                } else {
                    position_reference =
                        initial_pose.position().x() +
                        POSITION_AMPLITUDE *
                            (1.0 + std::sin(2.0 * M_PI * FREQUENCY * time - 0.5 * M_PI));
                    orientation_reference =
                        initial_pose.euler().y() +
                        ORIENTATION_AMPLITUDE *
                            (1.0 + std::sin(2.0 * M_PI * FREQUENCY * time - 0.5 * M_PI));
                }

                // Compute new pose reference for output
                // Note: The references are relative to the frames specified by the
                // EGMActPose RAPID instruction.
                output.mutable_robot()
                    ->mutable_cartesian()
                    ->mutable_pose()
                    ->mutable_position()
                    ->set_x(position_reference);
                output.mutable_robot()
                    ->mutable_cartesian()
                    ->mutable_pose()
                    ->mutable_euler()
                    ->set_y(orientation_reference);
            }

            // Write references back to the EGM client.
            egm_interface.write(output);

            // Print info
            if (sequence_number % (1 * (int)EGM_RATE) == 0)
                std::cout << "[" << sequence_number << "] References: \n"
                          << "\tX position = " << input_pose.position().x() << " [mm]\n"
                          << "\tY orientation (Euler) = " << input_pose.euler().y()
                          << " [degrees]" << std::endl;
        } else {
            // Timeout info
            std::cerr << "TIMEOUT" << std::endl;
        }
    }


    // SHUTDOWN --------------------------------------------------------------------------
    io_service.stop();
    thread_group.join_all();
    return 0;
}