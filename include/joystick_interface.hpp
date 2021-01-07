#pragma once
#include <boost/algorithm/string.hpp>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <iostream>



namespace joystick_interface {
struct JoystickStatus {
    int axis0;  // Left x
    int axis1;  // Left y
    int axis2;  // Right x
    int axis3;  // Right y
    bool quit;

    std::string to_string() {
        std::string ret = std::to_string(axis0);
        ret.append("/");
        ret.append(std::to_string(axis1));
        ret.append("/");
        ret.append(std::to_string(axis2));
        ret.append("/");
        ret.append(std::to_string(axis3));
        ret.append("/");
        ret.append(std::to_string(quit));
        return ret;
    }

    static JoystickStatus parse(std::string &sample) {
        std::vector<std::string> info;
        boost::split(info, sample, boost::is_any_of("/"));
        return JoystickStatus{std::stoi(info[0]), std::stoi(info[1]), std::stoi(info[2]),
                              std::stoi(info[3]), static_cast<bool>(std::stoi(info[4]))};
    }
};

class JoystickInterface {
  private:
    boost::shared_ptr<boost::asio::ip::udp::socket> socket_ptr_;
    boost::array<char, 1024> recv_buffer_;
    boost::asio::ip::udp::endpoint remote_endpoint_;
    int port_;
    JoystickStatus status_{0, 0, false};

  public:
    JoystickInterface(boost::asio::io_service &io_service,
                      boost::thread_group &thread_group, const int port);

    void handle_receive(const boost::system::error_code &error, size_t bytes_transferred);
    void wait();
    JoystickStatus read();
    ~JoystickInterface();
};

}  // namespace joystick_interface
