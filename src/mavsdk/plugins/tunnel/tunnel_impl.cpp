#include "tunnel_impl.h"
#include "mavlink_address.h"
#include "system.h"
#include "callback_list.tpp"

namespace mavsdk {

template class CallbackList<Tunnel::TunnelMessage>;

TunnelImpl::TunnelImpl(System& system) : PluginImplBase(system)
{
    _system_impl->register_plugin(this);
}

TunnelImpl::TunnelImpl(std::shared_ptr<System> system) : PluginImplBase(std::move(system))
{
    _system_impl->register_plugin(this);
}


TunnelImpl::~TunnelImpl()
{

    _system_impl->unregister_plugin(this);

}

void TunnelImpl::init()
{
    _system_impl->register_mavlink_message_handler(
        MAVLINK_MSG_ID_TUNNEL,
        [this](const mavlink_message_t& message) { process_tunnel_message(message); },
        this);
}

void TunnelImpl::deinit()
{
    _system_impl->unregister_all_mavlink_message_handlers(this);
}


void TunnelImpl::enable() {}

void TunnelImpl::disable() {}






Tunnel::Result TunnelImpl::send_tunnel_message(uint32_t target_system, uint32_t target_component, uint32_t payload_type, uint32_t payload_length, std::vector<std::byte> payload)
{
    if (!_system_impl->is_connected()) {
        return Tunnel::Result::NoSystem;
    }

    if (payload_length > 128) {
        return Tunnel::Result::InvalidArgument;
    }

    // Ensure payload is exactly 128 bytes
    if (payload.size() != 128) {
        payload.resize(128, std::byte{0});
    }

    if (!send_tunnel_mavlink_message(target_system, target_component, payload_type, payload_length, payload)) {
        return Tunnel::Result::ConnectionError;
    }

    return Tunnel::Result::Success;
}



    
Tunnel::TunnelMessageHandle TunnelImpl::subscribe_tunnel_message(const Tunnel::TunnelMessageCallback& callback)
{
    std::lock_guard<std::mutex> lock(_tunnel_message_mutex);
    return _tunnel_message_callbacks.subscribe(callback);
}

void TunnelImpl::unsubscribe_tunnel_message(Tunnel::TunnelMessageHandle handle)
{
    std::lock_guard<std::mutex> lock(_tunnel_message_mutex);
    _tunnel_message_callbacks.unsubscribe(handle);
}
    





void TunnelImpl::process_tunnel_message(const mavlink_message_t& message)
{
    mavlink_tunnel_t tunnel_msg;
    mavlink_msg_tunnel_decode(&message, &tunnel_msg);

    Tunnel::TunnelMessage tunnel_message{};
    tunnel_message.target_system = tunnel_msg.target_system;
    tunnel_message.target_component = tunnel_msg.target_component;
    tunnel_message.payload_type = tunnel_msg.payload_type;
    tunnel_message.payload_length = tunnel_msg.payload_length;
    
    // Convert payload to std::vector<std::byte>
    tunnel_message.payload.resize(128);
    for (int i = 0; i < 128; ++i) {
        tunnel_message.payload[i] = std::byte{tunnel_msg.payload[i]};
    }

    std::lock_guard<std::mutex> lock(_tunnel_message_mutex);
    _tunnel_message_callbacks.queue(tunnel_message, [this](const auto& func) {
        _system_impl->call_user_callback(func);
    });
}

bool TunnelImpl::send_tunnel_mavlink_message(uint32_t target_system, uint32_t target_component, uint32_t payload_type, uint32_t payload_length, const std::vector<std::byte>& payload)
{
    mavlink_message_t message;
    
    // Convert std::vector<std::byte> to uint8_t array for MAVLink
    uint8_t mavlink_payload[128];
    for (size_t i = 0; i < 128 && i < payload.size(); ++i) {
        mavlink_payload[i] = static_cast<uint8_t>(payload[i]);
    }
    
    return _system_impl->queue_message([&](MavlinkAddress mavlink_address, uint8_t channel) {
        mavlink_msg_tunnel_pack_chan(
            mavlink_address.system_id,
            mavlink_address.component_id,
            channel,
            &message,
            static_cast<uint8_t>(target_system),
            static_cast<uint8_t>(target_component),
            static_cast<uint16_t>(payload_type),
            static_cast<uint8_t>(payload_length),
            mavlink_payload);
        return message;
    });
}

} // namespace mavsdk