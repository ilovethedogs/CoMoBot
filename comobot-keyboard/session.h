#ifndef COMOBOT_KEYBOARD_SESSION_H
#define COMOBOT_KEYBOARD_SESSION_H

#include <memory>

#define ASIO_STANDALONE
#include "asio.hpp"

namespace my_session {

class session : public std::enable_shared_from_this<session> {
public:
    session(asio::ip::tcp::socket socket)
        :tcp_socket(std::move(socket))
    {

    }

    void start() {
        read();
    }

private:
    void read();
    void write(std::string_view response);
    std::array<char, 1024> data;
    asio::ip::tcp::socket tcp_socket;
};

} // my_session

#endif //COMOBOT_KEYBOARD_SESSION_H
