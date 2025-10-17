
#pragma once

#include <PhysicsGrab/PhysicsGrabTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace PhysicsGrab
{
    class PhysicsGrabRequests
    {
    public:
        AZ_RTTI(PhysicsGrabRequests, PhysicsGrabRequestsTypeId);
        virtual ~PhysicsGrabRequests() = default;
        // Put your public methods here
    };

    class PhysicsGrabBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using PhysicsGrabRequestBus = AZ::EBus<PhysicsGrabRequests, PhysicsGrabBusTraits>;
    using PhysicsGrabInterface = AZ::Interface<PhysicsGrabRequests>;

} // namespace PhysicsGrab
