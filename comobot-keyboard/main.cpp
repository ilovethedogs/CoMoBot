#include <iostream>
#include <string>
#include "session.h"

union Speedtype {
    float f;
    uint32_t i;
};

int main() {
    auto host {std::string_view {"192.168.0.223"}};
    auto const port {short{4001}};
    try {
        asio::io_context context;
        asio::ip::tcp::socket tcp_socket(context);
        asio::ip::tcp::resolver resolver(context);
        asio::connect(tcp_socket, resolver.resolve({host.data(), std::to_string(port)}));

        auto speed0 {Speedtype {}};
        auto speed1 {Speedtype {}};
        while (true) {
            auto reply {std::array<char unsigned, 14> {}};
            auto reply_length = tcp_socket.read_some(asio::buffer(reply, reply.size()));
            speed0.i = reply[6] | (reply[7] << 8) | (reply[8] << 16) | (reply[9] << 24);
            speed1.i = reply[10] | (reply[11] << 8) | (reply[12] << 16) | (reply[13] << 24);
            std::cout << speed0.f << ' ' << speed1.f << std::endl;
            //std::cout.write(reply.data(), reply_length);
        }
    }
    catch (std::exception& e) {
        std::cerr << "exception: " << e.what() << std::endl;
    }
}
