// WARNING: THIS FILE IS AUTOGENERATED! As such, it should not be edited.
// Edits need to be made to the proto files
// (see https://github.com/mavlink/MAVSDK-Proto/blob/main/protos/arm_authorizer_server/arm_authorizer_server.proto)

#pragma once

#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>


#include "server_plugin_base.h"

#include "handle.h"

namespace mavsdk {


class ServerComponent;
class ArmAuthorizerServerImpl;

/**
 * @brief 
 */
class ArmAuthorizerServer : public ServerPluginBase {
public:

    /**
     * @brief Constructor. Creates the plugin for a ServerComponent instance.
     *
     * The plugin is typically created as shown below:
     *
     *     ```cpp
     *     auto arm_authorizer_server = ArmAuthorizerServer(server_component);
     *     ```
     *
     * @param server_component The ServerComponent instance associated with this server plugin.
     */
    explicit ArmAuthorizerServer(std::shared_ptr<ServerComponent> server_component);


    /**
     * @brief Destructor (internal use only).
     */
    ~ArmAuthorizerServer() override;


    /**
     * @brief 
     */
    enum class CommandAnswer {
        Accepted, /**< @brief Command accepted. */
        Failed, /**< @brief Command failed. */
    };

    /**
     * @brief Stream operator to print information about a `ArmAuthorizerServer::CommandAnswer`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream& operator<<(std::ostream& str, ArmAuthorizerServer::CommandAnswer const& command_answer);

    /**
     * @brief 
     */
    enum class RejectionReason {
        ReasonGeneric, /**< @brief Not a specific reason. */
        ReasonNone, /**< @brief Authorizer will send the error as string to GCS. */
        ReasonInvalidWaypoint, /**< @brief At least one waypoint have a invalid value. */
        ReasonTimeout, /**< @brief Timeout in the authorizer process(in case it depends on network). */
        ReasonAirspaceInUse, /**< @brief Airspace of the mission in use by another vehicle, second result parameter can have the waypoint id that caused it to be denied.. */
        ReasonBadWeather, /**< @brief Weather is not good to fly. */
    };

    /**
     * @brief Stream operator to print information about a `ArmAuthorizerServer::RejectionReason`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream& operator<<(std::ostream& str, ArmAuthorizerServer::RejectionReason const& rejection_reason);





        

    /**
     * @brief Callback type for subscribe_arm_authorization.
     */
    using ArmAuthorizationCallback = std::function<void(uint32_t)>;

    /**
     * @brief Handle type for subscribe_arm_authorization.
     */
    using ArmAuthorizationHandle = Handle<uint32_t>;

    /**
     * @brief Subscribe to arm authorization request messages. Each request received should respond to using RespondArmAuthorization
     */
    ArmAuthorizationHandle subscribe_arm_authorization(const ArmAuthorizationCallback& callback);

    /**
     * @brief Unsubscribe from subscribe_arm_authorization
     */
    void unsubscribe_arm_authorization(ArmAuthorizationHandle handle);

        








    /**
     * @brief Authorize arm for the specific time
     *
     * This function is blocking.
     *
     * @return Result of request.
     */
    ArmAuthorizerServer::CommandAnswer accept_arm_authorization(int32_t valid_time) const;






    /**
     * @brief Reject arm authorization request
     *
     * This function is blocking.
     *
     * @return Result of request.
     */
    ArmAuthorizerServer::CommandAnswer reject_arm_authorization(bool temporarily, RejectionReason reason, int32_t extra_info) const;




    /**
     * @brief Copy constructor.
     */
    ArmAuthorizerServer(const ArmAuthorizerServer& other);

    /**
     * @brief Equality operator (object is not copyable).
     */
    const ArmAuthorizerServer& operator=(const ArmAuthorizerServer&) = delete;

private:
    /** @private Underlying implementation, set at instantiation */
    std::unique_ptr<ArmAuthorizerServerImpl> _impl;
};

} // namespace mavsdk