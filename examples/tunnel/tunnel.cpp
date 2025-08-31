//
// Example to demonstrate how to use the Tunnel plugin to send and receive
// custom TUNNEL messages for arbitrary data tunneling through MAVLink.
//

#include <chrono>
#include <cstdint>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/tunnel/tunnel.h>
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

    auto system = mavsdk.first_autopilot(3.0);
    if (!system) {
        std::cerr << "Timed out waiting for system\n";
        return 1;
    }

    // Instantiate plugin
    auto tunnel = Tunnel{system.value()};

    std::cout << "Setting up tunnel message subscriber...\n";

    // Subscribe to incoming tunnel messages
    auto handle = tunnel.subscribe_tunnel_message([](const Tunnel::TunnelMessage& message) {
        std::cout << "Received TUNNEL message:\n";
        std::cout << "  Target System: " << message.target_system << "\n";
        std::cout << "  Target Component: " << message.target_component << "\n";
        std::cout << "  Payload Type: " << message.payload_type << "\n";
        std::cout << "  Payload Length: " << message.payload_length << "\n";
        std::cout << "  Payload: ";
        for (size_t i = 0; i < message.payload_length && i < message.payload.size(); ++i) {
            std::cout << "0x" << std::hex << static_cast<uint8_t>(message.payload[i]) << " ";
        }
        std::cout << std::dec << "\n\n";
    });

    std::cout << "Sending example tunnel messages...\n";

    // Example 1: Send a simple text message
    {
        std::string text_message = "Hello TUNNEL!";
        std::vector<std::byte> payload(128, std::byte{0});

        // Copy text into payload
        for (size_t i = 0; i < text_message.length() && i < 128; ++i) {
            payload[i] = std::byte{static_cast<uint8_t>(text_message[i])};
        }

        auto result = tunnel.send_tunnel_message(
            255, // target_system
            190, // target_component
            100, // payload_type (custom type)
            static_cast<uint32_t>(text_message.length()), // payload_length
            payload);

        if (result == Tunnel::Result::Success) {
            std::cout << "Successfully sent text message: \"" << text_message << "\"\n";
        } else {
            std::cout << "Failed to send text message: " << result << "\n";
        }
    }

    sleep_for(seconds(1));

    // Example 2: Send binary data
    {
        std::vector<std::byte> payload(128, std::byte{0});

        // Fill with some example binary data
        for (int i = 0; i < 128; ++i) {
            payload[i] = std::byte{static_cast<uint8_t>(i * 2)};
        }

        auto result = tunnel.send_tunnel_message(
            255, // target_system
            190, // target_component
            43, // payload_type (different custom type)
            128, // payload_length
            payload);

        if (result == Tunnel::Result::Success) {
            std::cout << "Successfully sent binary data\n";
        } else {
            std::cout << "Failed to send binary data: " << result << "\n";
        }
    }

    sleep_for(seconds(1));

    // Example 3: Send structured data (simulating a custom protocol)
    {
        std::vector<std::byte> payload(128, std::byte{0});

        // Example: Simple packet with header [MAGIC][SEQ][LEN][DATA]
        payload[0] = std::byte{0xAA}; // Magic byte 1
        payload[1] = std::byte{0x55}; // Magic byte 2
        payload[2] = std::byte{0x01}; // Sequence number
        payload[3] = std::byte{0x08}; // Data length

        // Data payload
        const char* data = "MAVSDK!!";
        for (int i = 0; i < 8; ++i) {
            payload[4 + i] = std::byte{static_cast<uint8_t>(data[i])};
        }

        auto result = tunnel.send_tunnel_message(
            255, // target_system
            190, // target_component
            300, // payload_type (custom protocol type)
            12, // payload_length (header + data)
            payload);

        if (result == Tunnel::Result::Success) {
            std::cout << "Successfully sent structured data packet\n";
        } else {
            std::cout << "Failed to send structured data: " << result << "\n";
        }
    }

    std::cout << "Listening for tunnel messages for 10 seconds...\n";
    sleep_for(seconds(10));

    // Unsubscribe
    tunnel.unsubscribe_tunnel_message(handle);
    std::cout << "Example completed.\n";

    return 0;
}
