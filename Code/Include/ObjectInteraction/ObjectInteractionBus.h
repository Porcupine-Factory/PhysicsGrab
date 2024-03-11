
#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace ObjectInteraction
{
    class ObjectInteractionRequests
    {
    public:
        AZ_RTTI(ObjectInteractionRequests, "{0A402AE5-14A5-4C61-9EA8-9F9ABCF36F5E}");
        virtual ~ObjectInteractionRequests() = default;
        // Put your public methods here
    };

    class ObjectInteractionBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using ObjectInteractionRequestBus = AZ::EBus<ObjectInteractionRequests, ObjectInteractionBusTraits>;
    using ObjectInteractionInterface = AZ::Interface<ObjectInteractionRequests>;

} // namespace ObjectInteraction
