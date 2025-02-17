#include "events_impl.h"

namespace mavsdk {


EventsImpl::EventsImpl(System& system) : PluginImplBase(system)
{
    _system_impl->register_plugin(this);
}

EventsImpl::EventsImpl(std::shared_ptr<System> system) : PluginImplBase(std::move(system))
{
    _system_impl->register_plugin(this);
}


EventsImpl::~EventsImpl()
{

    _system_impl->unregister_plugin(this);

}

void EventsImpl::init() {}

void EventsImpl::deinit() {}


void EventsImpl::enable() {}

void EventsImpl::disable() {}




    
Events::EventsHandle EventsImpl::subscribe_events(const Events::EventsCallback& callback)
{
    
    UNUSED(callback);
}

void EventsImpl::unsubscribe_events(Events::EventsHandle handle)
{
    UNUSED(handle);
}
    





    
Events::HealthAndArmingChecksHandle EventsImpl::subscribe_health_and_arming_checks(const Events::HealthAndArmingChecksCallback& callback)
{
    
    UNUSED(callback);
}

void EventsImpl::unsubscribe_health_and_arming_checks(Events::HealthAndArmingChecksHandle handle)
{
    UNUSED(handle);
}
    







std::pair<Events::Result, Events::HealthAndArmingCheckReport> EventsImpl::get_health_and_arming_checks_report()
{
    

    // TODO :)
    return {};
}



} // namespace mavsdk