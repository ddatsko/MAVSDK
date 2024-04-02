// WARNING: THIS FILE IS AUTOGENERATED! As such, it should not be edited.
// Edits need to be made to the proto files
// (see
// https://github.com/mavlink/MAVSDK-Proto/blob/master/protos/arm_authorizer_server/arm_authorizer_server.proto)

#include <iomanip>

#include "arm_authorizer_server_impl.h"
#include "plugins/arm_authorizer_server/arm_authorizer_server.h"

namespace mavsdk {

ArmAuthorizerServer::ArmAuthorizerServer(std::shared_ptr<ServerComponent> server_component) :
    ServerPluginBase(),
    _impl{std::make_unique<ArmAuthorizerServerImpl>(server_component)}
{}

ArmAuthorizerServer::~ArmAuthorizerServer() {}

ArmAuthorizerServer::ArmAuthorizationHandle
ArmAuthorizerServer::subscribe_arm_authorization(const ArmAuthorizationCallback& callback)
{
    return _impl->subscribe_arm_authorization(callback);
}

void ArmAuthorizerServer::unsubscribe_arm_authorization(ArmAuthorizationHandle handle)
{
    _impl->unsubscribe_arm_authorization(handle);
}

ArmAuthorizerServer::Result
ArmAuthorizerServer::accept_arm_authorization(int32_t valid_time_s) const
{
    return _impl->accept_arm_authorization(valid_time_s);
}

ArmAuthorizerServer::Result ArmAuthorizerServer::reject_arm_authorization(
    bool temporarily, RejectionReason reason, int32_t extra_info) const
{
    return _impl->reject_arm_authorization(temporarily, reason, extra_info);
}

std::ostream& operator<<(std::ostream& str, ArmAuthorizerServer::Result const& result)
{
    switch (result) {
        case ArmAuthorizerServer::Result::Success:
            return str << "Success";
        case ArmAuthorizerServer::Result::Failed:
            return str << "Failed";
        default:
            return str << "Unknown";
    }
}

std::ostream&
operator<<(std::ostream& str, ArmAuthorizerServer::RejectionReason const& rejection_reason)
{
    switch (rejection_reason) {
        case ArmAuthorizerServer::RejectionReason::ReasonGeneric:
            return str << "Reason Generic";
        case ArmAuthorizerServer::RejectionReason::ReasonNone:
            return str << "Reason None";
        case ArmAuthorizerServer::RejectionReason::ReasonInvalidWaypoint:
            return str << "Reason Invalid Waypoint";
        case ArmAuthorizerServer::RejectionReason::ReasonTimeout:
            return str << "Reason Timeout";
        case ArmAuthorizerServer::RejectionReason::ReasonAirspaceInUse:
            return str << "Reason Airspace In Use";
        case ArmAuthorizerServer::RejectionReason::ReasonBadWeather:
            return str << "Reason Bad Weather";
        default:
            return str << "Unknown";
    }
}

} // namespace mavsdk