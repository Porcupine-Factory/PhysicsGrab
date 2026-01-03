/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace PhysicsGrab
{
    class NetworkPhysicsGrabRequests
    {
    public:
        AZ_RTTI(NetworkPhysicsGrabRequests, "{CBC4EB7E-A0A9-462A-B820-536175CEF6F1}");
        virtual ~NetworkPhysicsGrabRequests() = default;
        // Put your public methods here
    };

    class NetworkPhysicsGrabBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using NetworkPhysicsGrabRequestBus = AZ::EBus<NetworkPhysicsGrabRequests, NetworkPhysicsGrabBusTraits>;
    using NetworkPhysicsGrabInterface = AZ::Interface<NetworkPhysicsGrabRequests>;

} // namespace PhysicsGrab
