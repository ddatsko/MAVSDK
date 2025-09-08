#include "command_server_impl.h"
#include "callback_list.tpp"

namespace mavsdk {

template class CallbackList<CommandServer::MavlinkCommand>;

CommandServerImpl::CommandServerImpl(std::shared_ptr<ServerComponent> server_component) :
    ServerPluginImplBase(server_component)
{
    _server_component_impl->register_plugin(this);
}

CommandServerImpl::~CommandServerImpl()
{
    _server_component_impl->unregister_plugin(this);
}

void CommandServerImpl::init()
{
    _server_component_impl->register_mavlink_command_handler(
        0, // 0 means all commands
        [this](const MavlinkCommandReceiver::CommandLong& command) {
            return process_any_command(command);
        },
        this);
}

void CommandServerImpl::deinit()
{
    _server_component_impl->unregister_all_mavlink_command_handlers(this);
}

CommandServer::CommandHandle CommandServerImpl::subscribe_command(
    uint32_t command_id, const CommandServer::CommandCallback& callback)
{
    std::lock_guard<std::mutex> lock(_command_callbacks_mutex);

    if (command_id == 0) {
        return _any_command_callbacks.subscribe(callback);
    } else {
        return _command_callbacks[command_id].subscribe(callback);
    }
}

void CommandServerImpl::unsubscribe_command(CommandServer::CommandHandle handle)
{
    std::lock_guard<std::mutex> lock(_command_callbacks_mutex);

    _any_command_callbacks.unsubscribe(handle);

    for (auto& [command_id, callback_list] : _command_callbacks) {
        callback_list.unsubscribe(handle);
    }
}

CommandServer::Result CommandServerImpl::respond_command(CommandServer::CommandAck command_ack)
{
    MAV_RESULT mav_result;
    switch (command_ack.result) {
        case CommandServer::CommandResult::Accepted:
            mav_result = MAV_RESULT_ACCEPTED;
            break;
        case CommandServer::CommandResult::TemporarilyRejected:
            mav_result = MAV_RESULT_TEMPORARILY_REJECTED;
            break;
        case CommandServer::CommandResult::Denied:
            mav_result = MAV_RESULT_DENIED;
            break;
        case CommandServer::CommandResult::Unsupported:
            mav_result = MAV_RESULT_UNSUPPORTED;
            break;
        case CommandServer::CommandResult::Failed:
            mav_result = MAV_RESULT_FAILED;
            break;
        case CommandServer::CommandResult::InProgress:
            mav_result = MAV_RESULT_IN_PROGRESS;
            break;
        case CommandServer::CommandResult::Cancelled:
            mav_result = MAV_RESULT_CANCELLED;
            break;
        case CommandServer::CommandResult::CommandIntOnly:
            mav_result = MAV_RESULT_COMMAND_INT_ONLY;
            break;
        case CommandServer::CommandResult::CommandLongOnly:
            mav_result = MAV_RESULT_COMMAND_LONG_ONLY;
            break;
        default:
            mav_result = MAV_RESULT_FAILED;
            break;
    }

    mavlink_command_ack_t mav_command_ack;
    mav_command_ack.command = command_ack.command;
    mav_command_ack.progress = command_ack.progress;
    mav_command_ack.result = mav_result;
    mav_command_ack.result_param2 = command_ack.result_param2;
    mav_command_ack.target_system = command_ack.target_system;
    mav_command_ack.target_component = command_ack.target_component;
    _server_component_impl->send_command_ack(mav_command_ack);

    return CommandServer::Result::Success;
}

void CommandServerImpl::send_command_async(
    CommandServer::MavlinkCommand command, const CommandServer::ResultCallback callback)
{
    mavlink_message_t message{};
    mavlink_msg_command_long_pack(
        _server_component_impl->get_own_system_id(),
        _server_component_impl->get_own_component_id(),
        &message,
        command.target_system,
        command.target_component,
        command.command,
        0, // confirmation
        command.param1,
        command.param2,
        command.param3,
        command.param4,
        command.param5,
        command.param6,
        command.param7);

    if (_server_component_impl->send_message(message)) {
        if (callback) {
            // TODO: this is not async at all))
            callback(CommandServer::Result::Success);
        }
    } else {
        LogErr() << "Failed to send command: " << command.command;
        if (callback) {
            callback(CommandServer::Result::ConnectionError);
        }
    }
}

std::pair<CommandServer::Result, CommandServer::CommandAck>
CommandServerImpl::send_command_and_wait_for_ack(
    CommandServer::MavlinkCommand command, float timeout_s)
{
    UNUSED(timeout_s); // TODO: Implement timeout handling

    mavlink_message_t message{};
    mavlink_msg_command_long_pack(
        _server_component_impl->get_own_system_id(),
        _server_component_impl->get_own_component_id(),
        &message,
        command.target_system,
        command.target_component,
        command.command,
        0, // confirmation
        command.param1,
        command.param2,
        command.param3,
        command.param4,
        command.param5,
        command.param6,
        command.param7);

    // TODO: Implement proper async command sending with ack waiting
    // For now, just send the command and call back with success
    if (_server_component_impl->send_message(message)) {
        CommandServer::CommandAck ack{};
        ack.command = command.command;
        ack.result = CommandServer::CommandResult::Accepted;
        ack.target_system = command.target_system;
        ack.target_component = command.target_component;
        return {CommandServer::Result::Success, ack};
    } else {
        LogErr() << "Failed to send command: " << command.command;
        CommandServer::CommandAck ack{};
        return {CommandServer::Result::ConnectionError, ack};
    }
}

bool CommandServerImpl::is_command_sender_ok(const MavlinkCommandReceiver::CommandLong& command)
{
    if (command.target_system_id != 0 &&
        command.target_system_id != _server_component_impl->get_own_system_id()) {
        return false;
    }
    return true;
}

std::optional<mavlink_command_ack_t>
CommandServerImpl::process_any_command(const MavlinkCommandReceiver::CommandLong& command)
{
    if (!is_command_sender_ok(command)) {
        LogWarn() << "Incoming command is for target sysid "
                  << static_cast<int>(command.target_system_id) << " instead of "
                  << static_cast<int>(_server_component_impl->get_own_system_id());
        return std::nullopt;
    }

    CommandServer::MavlinkCommand mavlink_cmd{};
    mavlink_cmd.command = command.command;
    mavlink_cmd.target_system = command.target_system_id;
    mavlink_cmd.target_component = command.target_component_id;
    // mavlink_cmd.source_system = command.origin_system_id;
    // mavlink_cmd.source_component = command.origin_component_id;
    mavlink_cmd.param1 = command.params.param1;
    mavlink_cmd.param2 = command.params.param2;
    mavlink_cmd.param3 = command.params.param3;
    mavlink_cmd.param4 = command.params.param4;
    mavlink_cmd.param5 = command.params.param5;
    mavlink_cmd.param6 = command.params.param6;
    mavlink_cmd.param7 = command.params.param7;
    mavlink_cmd.confirmation = command.confirmation;

    {
        std::lock_guard<std::mutex> lock(_command_callbacks_mutex);

        // Call all command callbacks (command_id = 0)
        if (!_any_command_callbacks.empty()) {
            _any_command_callbacks(mavlink_cmd);
        }

        // Call specific command callbacks
        auto it = _command_callbacks.find(command.command);
        if (it != _command_callbacks.end() && !it->second.empty()) {
            it->second(mavlink_cmd);
        }
    }

    // Don't send automatic ack - user should call respond_command
    return std::nullopt;
}

} // namespace mavsdk
