#pragma once

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzFramework/Physics/Collision/CollisionLayers.h>

namespace TestGem
{
    class TestGemComponentRequests : public AZ::ComponentBus
    {
    public:
        ~TestGemComponentRequests() override = default;

        virtual AZ::EntityId GetGrabbingEntityId() const = 0;
        virtual AZ::EntityId GetActiveCameraEntityId() const = 0;
        virtual AZ::EntityId GetGrabbedObjectEntityId() const = 0;
        virtual AZ::EntityId GetLastGrabbedObjectEntityId() const = 0;
        virtual AZStd::string GetGrabbedCollisionGroup() const = 0;
        virtual void SetGrabbedCollisionGroup(const AZStd::string&) = 0;
        virtual AZStd::string GetCurrentGrabbedCollisionLayerName() const = 0;
        virtual void SetCurrentGrabbedCollisionLayerByName(const AZStd::string&) = 0;
        virtual AzPhysics::CollisionLayer GetCurrentGrabbedCollisionLayer() const = 0;
        virtual void SetCurrentGrabbedCollisionLayer(const AzPhysics::CollisionLayer&) = 0;
        virtual AZStd::string GetPrevGrabbedCollisionLayerName() const = 0;
        virtual void SetPrevGrabbedCollisionLayerByName(const AZStd::string&) = 0;
        virtual AzPhysics::CollisionLayer GetPrevGrabbedCollisionLayer() const = 0;
        virtual void SetPrevGrabbedCollisionLayer(const AzPhysics::CollisionLayer&) = 0;
		virtual AZStd::string GetTempGrabbedCollisionLayerName() const = 0;
		virtual void SetTempGrabbedCollisionLayerByName(const AZStd::string&) = 0;
        virtual AzPhysics::CollisionLayer GetTempGrabbedCollisionLayer() const = 0;
        virtual void SetTempGrabbedCollisionLayer(const AzPhysics::CollisionLayer&) = 0;
        virtual void SetGrabbingEntity(const AZ::EntityId) = 0;
        virtual float GetGrabObjectDistance() const = 0;
        virtual void SetGrabObjectDistance(const float&) = 0;
        virtual float GetMinGrabObjectDistance() const = 0;
        virtual void SetMinGrabObjectDistance(const float&) = 0;
        virtual float GetMaxGrabObjectDistance() const = 0;
        virtual void SetMaxGrabObjectDistance(const float&) = 0;
        virtual float GetInitialGrabObjectDistance() const = 0;
        virtual void SetInitialGrabObjectDistance(const float&) = 0;
        virtual float GetGrabObjectDistanceSpeed() const = 0;
        virtual void SetGrabObjectDistanceSpeed(const float&) = 0;
        virtual float GetGrabStrength() const = 0;
        virtual void SetGrabStrength(const float&) = 0;
        virtual float GetRotateScale() const = 0;
        virtual void SetRotateScale(const float&) = 0;
        virtual float GetThrowStrength() const = 0;
        virtual void SetThrowStrength(const float&) = 0;
        virtual float GetSphereCastRadius() const = 0;
        virtual void SetSphereCastRadius(const float&) = 0;
        virtual float GetSphereCastDistance() const = 0;
        virtual void SetSphereCastDistance(const float&) = 0;
        virtual bool GetGrabbedObjectIsKinematic() const = 0;
        virtual void SetGrabbedObjectIsKinematic(const AZ::EntityId, const bool&) = 0;
        virtual bool GetisGrabbing() const = 0;
        virtual bool GetisThrowing() const = 0;
        virtual bool GetisRotating() const = 0;
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
