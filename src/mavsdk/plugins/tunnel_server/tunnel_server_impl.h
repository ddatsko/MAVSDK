#pragma once

#include "plugins/tunnel_server/tunnel_server.h"

#include "server_plugin_impl_base.h"

#include "callback_list.h"

namespace mavsdk {

class TunnelServerImpl : public ServerPluginImplBase {
public:
    explicit TunnelServerImpl(std::shared_ptr<ServerComponent> server_component);

    ~TunnelServerImpl() override;

    void init() override;
    void deinit() override;

    TunnelServer::Result send_tunnel_message(
        uint32_t target_system,
        uint32_t target_component,
        uint32_t payload_type,
        uint32_t payload_length,
        std::vector<std::byte> payload);

    TunnelServer::TunnelMessageHandle
    subscribe_tunnel_message(const TunnelServer::TunnelMessageCallback& callback);

    void unsubscribe_tunnel_message(TunnelServer::TunnelMessageHandle handle);

private:
    void process_tunnel_message(const mavlink_message_t& message);

    bool send_tunnel_mavlink_message(
        uint32_t target_system,
        uint32_t target_component,
        uint32_t payload_type,
        uint32_t payload_length,
        const std::vector<std::byte>& payload);
    std::mutex _tunnel_message_mutex{};
    CallbackList<TunnelServer::TunnelMessage> _tunnel_message_callbacks{};
};

} // namespace mavsdk
