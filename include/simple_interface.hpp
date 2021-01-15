#pragma once
#include "abb_robots.hpp"
#include "custom_exceptions.hpp"
#include "custom_utils.hpp"
#include <abb_libegm/egm_controller_interface.h>


namespace simple_interface {

class Workspace {
  private:
    // the origin is in the center
    int x_;  // [mm]
    int y_;  // [mm]
    int z_;  // [mm]
    Pose origin_;

  public:
    // Initializes the workspace as a parallelepiped
    void init(const Pose &origin, const int x, const int y, const int z);
    void init(const Pose &origin, const int size);

    // Return true if this pose is inside the workspace
    bool insideX(const Pose &pose, const double delta);
    bool insideY(const Pose &pose, const double delta);
    bool insideZ(const Pose &pose, const double delta);

    // If this pose is out of the workspace it is corrected and returned, otherwise it
    // returns it unchanged
    Pose adjustPose(const Pose &pose);
};


class EGMInterface {
  private:
    // Create an EGM interface:
    // * Sets up an EGM server (that the robot controller's EGM client can connect to).
    // * Provides APIs to the user (for setting motion references, that are sent in reply
    // to the EGM client's request).
    boost::shared_ptr<abb::egm::EGMControllerInterface> egm_interface_ptr_;
    boost::thread *egm_thread_;

    // Message for egm communication
    abb::egm::wrapper::Input input_;
    abb::egm::wrapper::Output output_;

    // Sample rate
    const double EGM_RATE_;

    // ABB Robot informations
    const abb_robots::Robot robot_;

    // Workspace
    Workspace workspace_;


  public:
    EGMInterface(boost::asio::io_service &io_service, boost::thread_group &thread_group,
                 const int port, const double egm_rate, const abb_robots::Robot &robot);
    ~EGMInterface();
    // Wait the start of the connection and then return current pose.
    Pose waitConnection(const int ms = 500);
    // Wait the current pose else throw exception.
    Pose waitForPose(const int timeout);
    // Send to EGM client new pose.
    void sendPose(const Pose &pose);
    // Set the workspace (size in mm)
    void setWorkspace(const Pose &origin, const int x, const int y, const int z);
    void setWorkspace(const Pose &origin, const int size);
    // Return true if this pose violates the constraints of the workspace
    bool workspaceViolations(const Pose &pose, const double delta = 0.0);
    bool workspaceViolationX(const Pose &pose, const double delta = 0.0);
    bool workspaceViolationY(const Pose &pose, const double delta = 0.0);
    bool workspaceViolationZ(const Pose &pose, const double delta = 0.0);
    // Send to EGM client a new pose inside the workspace (checkViolation and adjust).
    Pose sendSafePose(const Pose &pose);
};
}  // namespace simple_interface