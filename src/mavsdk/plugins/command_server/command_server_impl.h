#pragma once

#include "plugins/command_server/command_server.h"

#include "server_plugin_impl_base.h"
#include <mutex>
#include "callback_list.h"

namespace mavsdk {

class CommandServerImpl : public ServerPluginImplBase {
public:
    explicit CommandServerImpl(std::shared_ptr<ServerComponent> server_component);

    ~CommandServerImpl() override;

    void init() override;
    void deinit() override;

    CommandServer::CommandHandle
    subscribe_command(uint32_t command_id, const CommandServer::CommandCallback& callback);

    void unsubscribe_command(CommandServer::CommandHandle handle);

    CommandServer::Result respond_command(CommandServer::CommandAck command_ack);

    void send_command_async(
        CommandServer::MavlinkCommand command, const CommandServer::ResultCallback callback);

    std::pair<CommandServer::Result, CommandServer::CommandAck>
    send_command_and_wait_for_ack(CommandServer::MavlinkCommand command, float timeout_s);

private:
    std::mutex _command_callbacks_mutex;
    CallbackList<CommandServer::MavlinkCommand> _any_command_callbacks{};
    std::map<uint32_t, CallbackList<CommandServer::MavlinkCommand>> _command_callbacks{};

    std::optional<mavlink_command_ack_t>
    process_any_command(const MavlinkCommandReceiver::CommandLong& command);
    bool is_command_sender_ok(const MavlinkCommandReceiver::CommandLong& command);
};

} // namespace mavsdk