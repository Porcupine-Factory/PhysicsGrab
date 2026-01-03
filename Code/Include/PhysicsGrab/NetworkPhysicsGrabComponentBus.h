/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

#pragma once

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/RTTI/BehaviorContext.h>

namespace PhysicsGrab
{
    class NetworkPhysicsGrabComponentRequests : public AZ::ComponentBus
    {
    public:
        ~NetworkPhysicsGrabComponentRequests() override = default;

        virtual bool GetIsNetEntityAutonomous() const = 0;
        virtual bool GetEnabled() const = 0;
        virtual void SetEnabled(const bool&) = 0;
    };

    using NetworkPhysicsGrabComponentRequestBus = AZ::EBus<NetworkPhysicsGrabComponentRequests>;

    class NetworkPhysicsGrabComponentNotifications : public AZ::ComponentBus
    {
    public:
        virtual void OnNetworkTickStart(const float&, const bool&, const AZ::EntityId&) = 0;
        virtual void OnNetworkTickFinish(const float&, const bool&, const AZ::EntityId&) = 0;
    };

    using NetworkPhysicsGrabComponentNotificationBus = AZ::EBus<NetworkPhysicsGrabComponentNotifications>;

    class NetworkPhysicsGrabComponentNotificationHandler
        : public NetworkPhysicsGrabComponentNotificationBus::Handler
        , public AZ::BehaviorEBusHandler
    {
    public:
        AZ_EBUS_BEHAVIOR_BINDER(
            NetworkPhysicsGrabComponentNotificationHandler,
            "{B54776D0-B6F8-4C5D-9090-CF3EB0008BC7}",
            AZ::SystemAllocator,
            OnNetworkTickStart,
            OnNetworkTickFinish);

        void OnNetworkTickStart(
            [[maybe_unused]] const float& deltaTime,
            [[maybe_unused]] const bool& server,
            [[maybe_unused]] const AZ::EntityId& entity) override
        {
            Call(FN_OnNetworkTickStart);
        }
        void OnNetworkTickFinish(
            [[maybe_unused]] const float& deltaTime,
            [[maybe_unused]] const bool& server,
            [[maybe_unused]] const AZ::EntityId& entity) override
        {
            Call(FN_OnNetworkTickFinish);
        }
    };
} // namespace PhysicsGrab
