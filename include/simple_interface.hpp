#include <abb_libegm/egm_controller_interface.h>
#include <iomanip>
#include <iostream>
#include <stdexcept>



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

  public:
    EGMInterface(boost::asio::io_service &io_service, boost::thread_group &thread_group,
                 const int port, const double egm_rate);
    // Wait the start of the connection and then return current pose
    Pose waitConnection(const int ms = 500);
    // Wait the current pose else throw exception
    Pose waitForPose(const int timeout);
    // Sent to EGM client new pose
    void sendPose(const Pose &pose);
};
}  // namespace simple_interface