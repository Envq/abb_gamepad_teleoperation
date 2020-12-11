#include "simple_interface.hpp"


namespace simple_interface {

// Pose =================================================================================
std::ostream &operator<<(std::ostream &stream, const Pose &pose) {
    stream << "Pose:" << std::endl;
    stream << "- Position:\t";
    stream << "x: " << pose.x;
    stream << "\ty: " << pose.y;
    stream << "\tz: " << pose.z << std::endl;
    stream << "- Orientation:\t";
    stream << "roll: " << pose.roll;
    stream << "\tpitch: " << pose.pitch;
    stream << "\tyaw: " << pose.yaw << std::endl;
    return stream;
}  // namespace simple_interface


// EGMInteface ===========================================================================
EGMInterface::EGMInterface(boost::asio::io_service &io_service,
                           boost::thread_group &thread_group, int port) {
    egm_interface_ptr_.reset(new abb::egm::EGMControllerInterface(io_service, port));

    // Check successful initialization
    if (!egm_interface_ptr_->isInitialized())
        throw std::runtime_error(
            "EGM interface failed to initialize (e.g. due to port already bound)");

    // Spin up a thread to run the io_service.
    thread_group.create_thread(boost::bind(&boost::asio::io_service::run, &io_service));

    // Prepare outputs message
    output_.Clear();
}

void EGMInterface::waitConnection(int timeout) {
    while (true) {
        using namespace abb::egm::wrapper;
        if (egm_interface_ptr_->isConnected()) {
            auto status = egm_interface_ptr_->getStatus().rapid_execution_state();
            if (status == Status_RAPIDExecutionState_RAPID_UNDEFINED)
                throw std::runtime_error(
                    "RAPID execution state is UNDEFINED (might happen first time after "
                    "controller start/restart. Try to restart the RAPID program");
            else if (status == Status_RAPIDExecutionState_RAPID_RUNNING)
                break;
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(timeout));
    }
}

Pose EGMInterface::waitForPose(int timeout) {
    // Wait for a new EGM message from the EGM client (with a timeout in ms).
    while (!egm_interface_ptr_->waitForMessage(timeout))
        ;
    // Read the message received from the EGM client
    egm_interface_ptr_->read(&input_);

    // Get input-pose
    auto &pose = input_.feedback().robot().cartesian().pose();

    // Return a Pose with input-pose values
    return Pose(pose.position().x(), pose.position().y(), pose.position().z(),
                pose.euler().x(), pose.euler().y(), pose.euler().z());
}

void EGMInterface::sendPose(Pose pose) {
    // Prepare output-pose
    auto new_pose = output_.mutable_robot()->mutable_cartesian()->mutable_pose();

    // Initialize output-pose
    new_pose->mutable_position()->set_x(pose.x);
    new_pose->mutable_position()->set_y(pose.y);
    new_pose->mutable_position()->set_z(pose.z);
    new_pose->mutable_euler()->set_x(pose.roll);
    new_pose->mutable_euler()->set_y(pose.pitch);
    new_pose->mutable_euler()->set_z(pose.yaw);

    // Write references back to the EGM client.
    egm_interface_ptr_->write(output_);
}
}  // namespace simple_interface