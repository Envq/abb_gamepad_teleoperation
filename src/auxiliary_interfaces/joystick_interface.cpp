#include <joystick_interface.hpp>



namespace joystick_interface {

JoystickInterface::JoystickInterface(boost::asio::io_service &io_service,
                                     boost::thread_group &thread_group, const int port) {
    port_ = port;
    socket_ptr_.reset(new boost::asio::ip::udp::socket(
        io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port_)));

    wait();

    std::cout << "Receiving\n";
    thread_group.create_thread(boost::bind(&boost::asio::io_service::run, &io_service));
    std::cout << "Receiver exit\n";
}

void JoystickInterface::handle_receive(const boost::system::error_code &error,
                                       size_t bytes_transferred) {
    if (error) {
        std::cout << "Receive failed: " << error.message() << "\n";
        return;
    }
    std::string msg =
        std::string(recv_buffer_.begin(), recv_buffer_.begin() + bytes_transferred);
    // std::cout << "Received: '" << msg << "'\n";
    last_info_ = JoystickInfo::parse(msg);

    if (!last_info_.quit)
        wait();
}

void JoystickInterface::wait() {
    socket_ptr_->async_receive_from(
        boost::asio::buffer(recv_buffer_), remote_endpoint_,
        boost::bind(&JoystickInterface::handle_receive, this,
                    boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));
}

JoystickInfo JoystickInterface::read() {
    return last_info_;
}

JoystickInterface::~JoystickInterface() {
    if (socket_ptr_) {
        socket_ptr_->close();
        socket_ptr_.reset();
    }
}

}  // namespace joystick_interface