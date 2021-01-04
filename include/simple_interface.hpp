#pragma once
#include "abb_robots.hpp"
#include "custom_exceptions.hpp"
#include <abb_libegm/egm_controller_interface.h>
#include <iostream>
#include <math.h>



namespace simple_interface {

class Pose {
  public:
    double x;
    double y;
    double z;
    double roll;   // arround x.
    double pitch;  // arround y.
    double yaw;    // arround z.

    // Constructors
    Pose() : Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0){};

    Pose(const double x_, const double y_, const double z_, const double roll_,
         const double pitch_, const double yaw_)
        : x(x_), y(y_), z(z_), roll(roll_), pitch(pitch_), yaw(yaw_){};

    Pose(const Pose &pose)
        : Pose(pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw){};

    // Operators
    friend std::ostream &operator<<(std::ostream &stream, const Pose &pose);
    friend bool operator==(const Pose &pose1, const Pose &pose2);
    friend bool operator!=(const Pose &pose1, const Pose &pose2);
};

class Workspace {
  private:
    // the origin is in the center
    int x_;
    int y_;
    int z_;
    Pose origin_;

  public:
    void init(const Pose &origin, const int x, const int y, const int z);
    void init(const Pose &origin, const int size);

    bool insideX(const Pose &pose, const double delta);
    bool insideY(const Pose &pose, const double delta);
    bool insideZ(const Pose &pose, const double delta);
};


class EGMInterface {
  private:
    // Create an EGM interface:
    // * Sets up an EGM server (that the robot controller's EGM client can connect to).
    // * Provides APIs to the user (for setting motion references, that are sent in reply
    // to the EGM client's request).
    boost::shared_ptr<abb::egm::EGMControllerInterface> egm_interface_ptr_;

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
    // Wait the start of the connection and then return current pose.
    Pose waitConnection(const int ms = 500);
    // Wait the current pose else throw exception.
    Pose waitForPose(const int timeout);
    // Sent to EGM client new pose.
    void sendPose(const Pose &pose);
    // Set the workspace (size in mm)
    void setWorkspace(const Pose &origin, const int x, const int y, const int z);
    void setWorkspace(const Pose &origin, const int size);
    // Check if this pose violates the constraints of the workspace
    bool workspaceViolations(const Pose &pose, const double delta = 0.0);
    bool workspaceViolationX(const Pose &pose, const double delta = 0.0);
    bool workspaceViolationY(const Pose &pose, const double delta = 0.0);
    bool workspaceViolationZ(const Pose &pose, const double delta = 0.0);
};
}  // namespace simple_interface