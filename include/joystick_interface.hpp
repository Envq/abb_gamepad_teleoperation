#pragma once
#include <boost/algorithm/string.hpp>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <iostream>



namespace joystick_interface {
struct JoystickInfo {
    int axis;
    int value;
    bool quit;

    std::string to_string() {
        std::string ret = std::to_string(axis);
        ret.append("/");
        ret.append(std::to_string(value));
        ret.append("/");
        ret.append(std::to_string(quit));
        return ret;
    }

    static JoystickInfo parse(std::string &sample) {
        std::vector<std::string> info;
        boost::split(info, sample, boost::is_any_of("/"));
        return JoystickInfo{std::stoi(info[0]), std::stoi(info[1]),
                            static_cast<bool>(std::stoi(info[2]))};
    }
};

class JoystickInterface {
  private:
    boost::shared_ptr<boost::asio::ip::udp::socket> socket_ptr_;
    boost::array<char, 1024> recv_buffer_;
    boost::asio::ip::udp::endpoint remote_endpoint_;
    int port_;
    JoystickInfo last_info_{0, 0, false};

  public:
    JoystickInterface(boost::asio::io_service &io_service,
                      boost::thread_group &thread_group, const int port);

    void handle_receive(const boost::system::error_code &error, size_t bytes_transferred);
    void wait();
    JoystickInfo read();
    ~JoystickInterface();
};

}  // namespace joystick_interface
