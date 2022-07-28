#include "session.h"

namespace my_session {
    void session::read() {
        auto self {shared_from_this()};
        tcp_socket.async_read_some(
                asio::buffer(data, data.size()), [this, self] (std::error_code const ec, std::size_t const length) {
                    if (!ec) {
                        auto number {std::string(data.data(), length)};
                    }
                });
    }

    void session::write(std::string_view response) {
        auto self {shared_from_this()};

        tcp_socket.async_write_some(
                asio::buffer(response.data(), response.length()), [this, self] (std::error_code const ec, std::size_t const) {
                    if (!ec) read();
                });
    }
} // my_session