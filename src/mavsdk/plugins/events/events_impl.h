#pragma once

#include "plugins/events/events.h"

#include "plugin_impl_base.h"


namespace mavsdk {


class EventsImpl : public PluginImplBase {
public:
    explicit EventsImpl(System& system);
    explicit EventsImpl(std::shared_ptr<System> system);

    ~EventsImpl() override;

    void init() override;
    void deinit() override;


    void enable() override;
    void disable() override;




        
    Events::EventsHandle subscribe_events(const Events::EventsCallback& callback);

    void unsubscribe_events(Events::EventsHandle handle);
        





        
    Events::HealthAndArmingChecksHandle subscribe_health_and_arming_checks(const Events::HealthAndArmingChecksCallback& callback);

    void unsubscribe_health_and_arming_checks(Events::HealthAndArmingChecksHandle handle);
        







    std::pair<Events::Result, Events::HealthAndArmingCheckReport> get_health_and_arming_checks_report();



private:
};

} // namespace mavsdk