#include "simple_interface.hpp"


namespace simple_interface {

// WORKSPACE =============================================================================
void Workspace::init(const Pose &origin, const int x, const int y, const int z) {
    origin_ = origin;
    x_ = x;
    y_ = y;
    z_ = z;
}

void Workspace::init(const Pose &origin, const int size) {
    init(origin, size, size, size);
}

bool Workspace::insideX(const Pose &pose, const double delta) {
    return (pose.x >= (origin_.x - x_ - delta) && pose.x <= (origin_.x + x_ + delta));
}

bool Workspace::insideY(const Pose &pose, const double delta) {
    return (pose.y >= (origin_.y - y_ - delta) && pose.y <= (origin_.y + y_ + delta));
}

bool Workspace::insideZ(const Pose &pose, const double delta) {
    return (pose.z >= (origin_.z - z_ - delta) && pose.z <= (origin_.z + z_ + delta));
}

Pose Workspace::adjustPose(const Pose &pose) {
    auto target = pose;
    target.x = (pose.x < origin_.x - x_)   ? origin_.x - x_
               : (pose.x > origin_.x + x_) ? origin_.x + x_
                                           : target.x;
    target.y = (pose.y < origin_.y - y_)   ? origin_.y - y_
               : (pose.y > origin_.y + y_) ? origin_.y + y_
                                           : target.y;
    target.z = (pose.z < origin_.z - z_)   ? origin_.z - z_
               : (pose.z > origin_.z + z_) ? origin_.z + z_
                                           : target.z;
    return target;
}


// EGMInteface
// ===========================================================================
EGMInterface::EGMInterface(boost::asio::io_service &io_service,
                           boost::thread_group &thread_group, const int port,
                           const double egm_rate, const abb_robots::Robot &robot)
    : EGM_RATE_(egm_rate), robot_(robot) {
    egm_interface_ptr_.reset(new abb::egm::EGMControllerInterface(io_service, port));

    // Check successful initialization
    if (!egm_interface_ptr_->isInitialized())
        throw EGMErrorException(
            "EGM interface failed to initialize (e.g. due to port already "
            "bound)");

    // Spin up a thread to run the io_service.
    thread_group.create_thread(boost::bind(&boost::asio::io_service::run, &io_service));
    // egm_thread_ =
    //     new boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));

    // Prepare outputs message
    output_.Clear();
}

EGMInterface::~EGMInterface() {
    // std::cout << "DELETE EGM INTERFACE" << std::endl;
}

Pose EGMInterface::waitConnection(const int ms) {
    while (true) {
        using namespace abb::egm::wrapper;
        if (egm_interface_ptr_->isConnected()) {
            auto status = egm_interface_ptr_->getStatus().rapid_execution_state();
            if (status == Status_RAPIDExecutionState_RAPID_UNDEFINED)
                std::cout << "RAPID execution state is UNDEFINED (might happen first "
                             "time after controller start/restart. Try to restart the "
                             "RAPID program"
                          << std::endl;
            else if (status == Status_RAPIDExecutionState_RAPID_RUNNING)
                break;
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(ms));
    }
    return waitForPose(ms);
}

Pose EGMInterface::waitForPose(const int timeout) {
    // Wait for a new EGM message from the EGM client (with a timeout in ms).
    if (!egm_interface_ptr_->waitForMessage(timeout))
        throw EGMWarnException("WaitForPose: Timeout");

    // Read the message received from the EGM client
    egm_interface_ptr_->read(&input_);

    // Get current pose
    auto &input_pose = input_.feedback().robot().cartesian().pose();

    // Return a Pose with input-pose values
    return {input_pose.position().x(), input_pose.position().y(),
            input_pose.position().z(), input_pose.euler().x(),
            input_pose.euler().y(),    input_pose.euler().z()};
}

void EGMInterface::sendPose(const Pose &pose) {
    // Prepare output-pose
    auto output_pose = output_.mutable_robot()->mutable_cartesian()->mutable_pose();

    // Initialize output-pose
    output_pose->mutable_position()->set_x(pose.x);
    output_pose->mutable_position()->set_y(pose.y);
    output_pose->mutable_position()->set_z(pose.z);
    output_pose->mutable_euler()->set_x(pose.roll);
    output_pose->mutable_euler()->set_y(pose.pitch);
    output_pose->mutable_euler()->set_z(pose.yaw);

    // Write references back to the EGM client.
    egm_interface_ptr_->write(output_);
}

void EGMInterface::setWorkspace(const Pose &origin, const int x, const int y,
                                const int z) {
    workspace_.init(origin, x, y, z);
}

void EGMInterface::setWorkspace(const Pose &origin, const int size) {
    workspace_.init(origin, size);
}

bool EGMInterface::workspaceViolations(const Pose &pose, const double delta) {
    return !workspace_.insideX(pose, delta) || !workspace_.insideY(pose, delta) ||
           !workspace_.insideZ(pose, delta);
}

bool EGMInterface::workspaceViolationX(const Pose &pose, const double delta) {
    return !workspace_.insideX(pose, delta);
}

bool EGMInterface::workspaceViolationY(const Pose &pose, const double delta) {
    return !workspace_.insideY(pose, delta);
}

bool EGMInterface::workspaceViolationZ(const Pose &pose, const double delta) {
    return !workspace_.insideZ(pose, delta);
}

Pose EGMInterface::sendSafePose(const Pose &pose) {
    auto target = workspace_.adjustPose(pose);
    sendPose(target);
    return target;
}



}  // namespace simple_interface