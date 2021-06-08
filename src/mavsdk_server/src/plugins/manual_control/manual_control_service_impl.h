// WARNING: THIS FILE IS AUTOGENERATED! As such, it should not be edited.
// Edits need to be made to the proto files
// (see
// https://github.com/mavlink/MAVSDK-Proto/blob/master/protos/manual_control/manual_control.proto)

#include "manual_control/manual_control.grpc.pb.h"
#include "plugins/manual_control/manual_control.h"

#include "mavsdk.h"
#include "lazy_plugin.h"
#include "log.h"
#include <atomic>
#include <cmath>
#include <future>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

namespace mavsdk {
namespace mavsdk_server {

template<typename ManualControl = ManualControl, typename LazyPlugin = LazyPlugin<ManualControl>>
class ManualControlServiceImpl final : public rpc::manual_control::ManualControlService::Service {
public:
    ManualControlServiceImpl(LazyPlugin& lazy_plugin) : _lazy_plugin(lazy_plugin) {}

    template<typename ResponseType>
    void fillResponseWithResult(ResponseType* response, mavsdk::ManualControl::Result& result) const
    {
        auto rpc_result = translateToRpcResult(result);

        auto* rpc_manual_control_result = new rpc::manual_control::ManualControlResult();
        rpc_manual_control_result->set_result(rpc_result);
        std::stringstream ss;
        ss << result;
        rpc_manual_control_result->set_result_str(ss.str());

        response->set_allocated_manual_control_result(rpc_manual_control_result);
    }

    static rpc::manual_control::ManualControlResult::Result
    translateToRpcResult(const mavsdk::ManualControl::Result& result)
    {
        switch (result) {
            default:
                LogErr() << "Unknown result enum value: " << static_cast<int>(result);
            // FALLTHROUGH
            case mavsdk::ManualControl::Result::Unknown:
                return rpc::manual_control::ManualControlResult_Result_RESULT_UNKNOWN;
            case mavsdk::ManualControl::Result::Success:
                return rpc::manual_control::ManualControlResult_Result_RESULT_SUCCESS;
            case mavsdk::ManualControl::Result::NoSystem:
                return rpc::manual_control::ManualControlResult_Result_RESULT_NO_SYSTEM;
            case mavsdk::ManualControl::Result::ConnectionError:
                return rpc::manual_control::ManualControlResult_Result_RESULT_CONNECTION_ERROR;
            case mavsdk::ManualControl::Result::Busy:
                return rpc::manual_control::ManualControlResult_Result_RESULT_BUSY;
            case mavsdk::ManualControl::Result::CommandDenied:
                return rpc::manual_control::ManualControlResult_Result_RESULT_COMMAND_DENIED;
            case mavsdk::ManualControl::Result::Timeout:
                return rpc::manual_control::ManualControlResult_Result_RESULT_TIMEOUT;
            case mavsdk::ManualControl::Result::InputOutOfRange:
                return rpc::manual_control::ManualControlResult_Result_RESULT_INPUT_OUT_OF_RANGE;
            case mavsdk::ManualControl::Result::InputNotSet:
                return rpc::manual_control::ManualControlResult_Result_RESULT_INPUT_NOT_SET;
        }
    }

    static mavsdk::ManualControl::Result
    translateFromRpcResult(const rpc::manual_control::ManualControlResult::Result result)
    {
        switch (result) {
            default:
                LogErr() << "Unknown result enum value: " << static_cast<int>(result);
            // FALLTHROUGH
            case rpc::manual_control::ManualControlResult_Result_RESULT_UNKNOWN:
                return mavsdk::ManualControl::Result::Unknown;
            case rpc::manual_control::ManualControlResult_Result_RESULT_SUCCESS:
                return mavsdk::ManualControl::Result::Success;
            case rpc::manual_control::ManualControlResult_Result_RESULT_NO_SYSTEM:
                return mavsdk::ManualControl::Result::NoSystem;
            case rpc::manual_control::ManualControlResult_Result_RESULT_CONNECTION_ERROR:
                return mavsdk::ManualControl::Result::ConnectionError;
            case rpc::manual_control::ManualControlResult_Result_RESULT_BUSY:
                return mavsdk::ManualControl::Result::Busy;
            case rpc::manual_control::ManualControlResult_Result_RESULT_COMMAND_DENIED:
                return mavsdk::ManualControl::Result::CommandDenied;
            case rpc::manual_control::ManualControlResult_Result_RESULT_TIMEOUT:
                return mavsdk::ManualControl::Result::Timeout;
            case rpc::manual_control::ManualControlResult_Result_RESULT_INPUT_OUT_OF_RANGE:
                return mavsdk::ManualControl::Result::InputOutOfRange;
            case rpc::manual_control::ManualControlResult_Result_RESULT_INPUT_NOT_SET:
                return mavsdk::ManualControl::Result::InputNotSet;
        }
    }

    grpc::Status StartPositionControl(
        grpc::ServerContext* /* context */,
        const rpc::manual_control::StartPositionControlRequest* /* request */,
        rpc::manual_control::StartPositionControlResponse* response) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            if (response != nullptr) {
                auto result = mavsdk::ManualControl::Result::NoSystem;
                fillResponseWithResult(response, result);
            }

            return grpc::Status::OK;
        }

        auto result = _lazy_plugin.maybe_plugin()->start_position_control();

        if (response != nullptr) {
            fillResponseWithResult(response, result);
        }

        return grpc::Status::OK;
    }

    grpc::Status StartAltitudeControl(
        grpc::ServerContext* /* context */,
        const rpc::manual_control::StartAltitudeControlRequest* /* request */,
        rpc::manual_control::StartAltitudeControlResponse* response) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            if (response != nullptr) {
                auto result = mavsdk::ManualControl::Result::NoSystem;
                fillResponseWithResult(response, result);
            }

            return grpc::Status::OK;
        }

        auto result = _lazy_plugin.maybe_plugin()->start_altitude_control();

        if (response != nullptr) {
            fillResponseWithResult(response, result);
        }

        return grpc::Status::OK;
    }

    grpc::Status SetManualControlInput(
        grpc::ServerContext* /* context */,
        const rpc::manual_control::SetManualControlInputRequest* request,
        rpc::manual_control::SetManualControlInputResponse* response) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            if (response != nullptr) {
                auto result = mavsdk::ManualControl::Result::NoSystem;
                fillResponseWithResult(response, result);
            }

            return grpc::Status::OK;
        }

        if (request == nullptr) {
            LogWarn() << "SetManualControlInput sent with a null request! Ignoring...";
            return grpc::Status::OK;
        }

        auto result = _lazy_plugin.maybe_plugin()->set_manual_control_input(
            request->x(), request->y(), request->z(), request->r());

        if (response != nullptr) {
            fillResponseWithResult(response, result);
        }

        return grpc::Status::OK;
    }

    void stop()
    {
        _stopped.store(true);
        for (auto& prom : _stream_stop_promises) {
            if (auto handle = prom.lock()) {
                handle->set_value();
            }
        }
    }

private:
    void register_stream_stop_promise(std::weak_ptr<std::promise<void>> prom)
    {
        // If we have already stopped, set promise immediately and don't add it to list.
        if (_stopped.load()) {
            if (auto handle = prom.lock()) {
                handle->set_value();
            }
        } else {
            _stream_stop_promises.push_back(prom);
        }
    }

    void unregister_stream_stop_promise(std::shared_ptr<std::promise<void>> prom)
    {
        for (auto it = _stream_stop_promises.begin(); it != _stream_stop_promises.end();
             /* ++it */) {
            if (it->lock() == prom) {
                it = _stream_stop_promises.erase(it);
            } else {
                ++it;
            }
        }
    }

    LazyPlugin& _lazy_plugin;
    std::atomic<bool> _stopped{false};
    std::vector<std::weak_ptr<std::promise<void>>> _stream_stop_promises{};
};

} // namespace mavsdk_server
} // namespace mavsdk