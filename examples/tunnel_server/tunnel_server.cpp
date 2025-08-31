//
// Example to demonstrate how to use the TunnelServer plugin to send
// custom TUNNEL messages for arbitrary data tunneling through MAVLink.
//

#include <chrono>
#include <cstdint>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/tunnel_server/tunnel_server.h>
#include <iostream>
#include <future>
#include <memory>
#include <thread>

using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

void usage(const std::string& bin_name)
{
    std::cerr << "Usage : " << bin_name << " <connection_url>\n"
              << "Connection URL format should be :\n"
              << " For TCP : tcp://[server_host][:server_port]\n"
              << " For UDP : udp://[bind_host][:bind_port]\n"
              << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
              << "For example, to connect to the simulator use URL: udp://:14540\n";
}

int main(int argc, char** argv)
{
    if (argc != 2) {
        usage(argv[0]);
        return 1;
    }

    mavsdk::Mavsdk::Configuration companion_computer_configuration{
        mavsdk::Mavsdk::ComponentType::CompanionComputer};
    companion_computer_configuration.set_component_id(MAV_COMP_ID_ONBOARD_COMPUTER);

    Mavsdk mavsdk{companion_computer_configuration};
    ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    // Instantiate plugin
    auto tunnel_server = TunnelServer{mavsdk.server_component()};

    tunnel_server.subscribe_tunnel_message([&](const TunnelServer::TunnelMessage &message) {
        std::cout << "NEW MESSAGE RECEIVED" << std::endl;
        std::cout << message.payload_length << std::endl;
        std::cout << message.payload_type << std::endl;
    });

    std::cout << "Sending tunnel message...\n";
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Send tunnel message with 128 bytes filled with data
    {
        std::vector<std::byte> payload(128);

        // Fill all 128 bytes with incremental data
        for (int i = 0; i < 128; ++i) {
            payload[i] = std::byte{static_cast<uint8_t>(i)};
        }

        auto result = tunnel_server.send_tunnel_message(
            255, // target_system (ground station)
            190, // target_component
            43,  // payload_type
            128, // payload_length (full 128 bytes)
            payload);

        if (result == TunnelServer::Result::Success) {
            std::cout << "Successfully sent tunnel message with 128 bytes\n";
        } else {
            std::cout << "Failed to send tunnel message: " << result << "\n";
        }
    }

    std::cout << "Tunnel message sent. Waiting 5 seconds before exit...\n";
    sleep_for(seconds(5));

    std::cout << "Example completed.\n";

    return 0;
}
