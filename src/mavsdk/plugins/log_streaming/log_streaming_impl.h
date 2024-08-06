#pragma once

#include "plugins/log_streaming/log_streaming.h"

#include "plugin_impl_base.h"
#include "callback_list.h"

#include <mutex>
#include <optional>

namespace mavsdk {

class LogStreamingImpl : public PluginImplBase {
public:
    explicit LogStreamingImpl(System& system);
    explicit LogStreamingImpl(std::shared_ptr<System> system);

    ~LogStreamingImpl() override;

    void init() override;
    void deinit() override;

    void enable() override;
    void disable() override;

    void start_log_streaming_async(const LogStreaming::ResultCallback& callback);

    LogStreaming::Result start_log_streaming();

    void stop_log_streaming_async(const LogStreaming::ResultCallback& callback);

    LogStreaming::Result stop_log_streaming();

    LogStreaming::LogStreamingRawHandle
    subscribe_log_streaming_raw(const LogStreaming::LogStreamingRawCallback& callback);

    void unsubscribe_log_streaming_raw(LogStreaming::LogStreamingRawHandle handle);

private:
    void process_logging_data(const mavlink_message_t& message);
    void process_logging_data_acked(const mavlink_message_t& message);
    void check_sequence(uint16_t sequence);
    void process_message();

    static LogStreaming::Result
    log_streaming_result_from_command_result(MavlinkCommandSender::Result result);

    std::mutex _mutex{};
    CallbackList<LogStreaming::LogStreamingRaw> _subscription_callbacks{};
    std::optional<uint16_t> _maybe_current_sequence{};
    unsigned _drops{0};
    std::vector<uint8_t> _ulog_data{};
    bool _active = false;

    bool _debugging{false};
};

} // namespace mavsdk
