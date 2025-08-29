#pragma once

#include <mutex>

#include "plugins/tunnel/tunnel.h"
#include "mavlink_include.h"
#include "plugin_impl_base.h"
#include "system.h"
#include "callback_list.h"


namespace mavsdk {


class TunnelImpl : public PluginImplBase {
public:
    explicit TunnelImpl(System& system);
    explicit TunnelImpl(std::shared_ptr<System> system);

    ~TunnelImpl() override;

    void init() override;
    void deinit() override;


    void enable() override;
    void disable() override;






    Tunnel::Result send_tunnel_message(uint32_t target_system, uint32_t target_component, uint32_t payload_type, uint32_t payload_length, std::vector<std::byte> payload);



        
    Tunnel::TunnelMessageHandle subscribe_tunnel_message(const Tunnel::TunnelMessageCallback& callback);

    void unsubscribe_tunnel_message(Tunnel::TunnelMessageHandle handle);
        





private:
    void process_tunnel_message(const mavlink_message_t& message);
    bool send_tunnel_mavlink_message(uint32_t target_system, uint32_t target_component, uint32_t payload_type, uint32_t payload_length, const std::vector<std::byte>& payload);

    std::mutex _tunnel_message_mutex{};
    CallbackList<Tunnel::TunnelMessage> _tunnel_message_callbacks{};
};

} // namespace mavsdk