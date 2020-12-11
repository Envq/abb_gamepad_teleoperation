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
    Pose();
    Pose(const int x, const int y, const int z, const int roll, const int pitch,
         const int yaw)
        : x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw){};
    friend std::ostream &operator<<(std::ostream &stream, const Pose &pose);
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

  public:
    EGMInterface(boost::asio::io_service &io_service, boost::thread_group &thread_group,
                 int port);
    void waitConnection(int timeout = 500);
    Pose waitForPose(int timeout);
    void sendPose(Pose pose);
};
}  // namespace simple_interface