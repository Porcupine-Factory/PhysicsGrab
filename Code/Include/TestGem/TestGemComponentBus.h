#pragma once

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/RTTI/BehaviorContext.h>

namespace TestGem
{
    class TestGemComponentRequests : public AZ::ComponentBus
    {
    public:
        ~TestGemComponentRequests() override = default;

        virtual float GetGrabObjectDistance() const = 0;
        virtual bool GetisObjectKinematic() const = 0;
    };

    using TestGemComponentRequestBus = AZ::EBus<TestGemComponentRequests>;
    /*
    class GrabNotifications
        : public AZ::ComponentBus
    {
    public:
        virtual void OnGrabChanged() = 0;
    };

    using GrabNotificationBus = AZ::EBus<GrabNotifications>;

    class GrabNotificationHandler
        : public GrabNotificationBus::Handler
        , public AZ::BehaviorEBusHandler
    {
    public:
        AZ_EBUS_BEHAVIOR_BINDER(GrabNotificationHandler,
            "{814559c9-032e-44ff-85fa-fc1a8ef95068}",
            AZ::SystemAllocator, OnGrabChanged);

        void OnGrabChanged() override
        {
            Call(FN_OnGrabChanged);
        }
    };*/
} // namespace TestGem
