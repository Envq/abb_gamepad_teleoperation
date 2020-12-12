#include <abb_libegm/egm_controller_interface.h>
#include <iostream>
#include <stdexcept>

namespace simple_interface {

struct Pose {
    double x;
    double y;
    double z;
    double roll;   // arround x
    double pitch;  // arround y
    double yaw;    // arround z
    Pose() : x(0.0), y(0.0), z(0.0), roll(0.0), pitch(0.0), yaw(0.0){};
    Pose(const int x, const int y, const int z, const int roll, const int pitch,
         const int yaw)
        : x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw){};
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

    // backup lastpose for check egm iusses
    Pose last_target_pose_;
    int errors_;
    const int LIMIT_ERRORS_ = 10;  // [s] (Time after throw exception).

    // default wait timeout
    const int DEFAULT_TIMEOUT_ = 400;

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