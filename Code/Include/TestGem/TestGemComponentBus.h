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
        virtual AZ::EntityId GetThrownGrabbedObjectEntityId() const = 0;
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
        virtual float GetGrabbedObjectDistance() const = 0;
        virtual void SetGrabbedObjectDistance(const float&) = 0;
        virtual float GetMinGrabbedObjectDistance() const = 0;
        virtual void SetMinGrabbedObjectDistance(const float&) = 0;
        virtual float GetMaxGrabbedObjectDistance() const = 0;
        virtual void SetMaxGrabbedObjectDistance(const float&) = 0;
        virtual float GetInitialGrabbedObjectDistance() const = 0;
        virtual void SetInitialGrabbedObjectDistance(const float&) = 0;
        virtual float GetGrabbedObjectDistanceSpeed() const = 0;
        virtual void SetGrabbedObjectDistanceSpeed(const float&) = 0;
        virtual float GetGrabResponse() const = 0;
        virtual void SetGrabResponse(const float&) = 0;
        virtual float GetDynamicRotateScale() const = 0;
        virtual void SetDynamicRotateScale(const float&) = 0;
        virtual float GetKinematicRotateScale() const = 0;
        virtual void SetKinematicRotateScale(const float&) = 0;
        virtual float GetThrowImpulse() const = 0;
        virtual void SetThrowImpulse(const float&) = 0;
        virtual float GetGrabbedObjectThrowStateCounter() const = 0;
        virtual float GetGrabbedObjectThrowStateTime() const = 0;
        virtual void SetGrabbedObjectThrowStateTime(const float&) = 0;
        virtual float GetSphereCastRadius() const = 0;
        virtual void SetSphereCastRadius(const float&) = 0;
        virtual float GetSphereCastDistance() const = 0;
        virtual void SetSphereCastDistance(const float&) = 0;
        virtual bool GetGrabbedObjectIsKinematic() const = 0;
        virtual void SetGrabbedObjectIsKinematic(const AZ::EntityId, const bool&) = 0;
        virtual bool GetInitialGrabbedObjectIsKinematic() const = 0;
        virtual float GetCurrentGrabbedObjectAngularDamping() const = 0;
        virtual void SetCurrentGrabbedObjectAngularDamping(const float&) = 0;
        virtual float GetPrevGrabbedObjectAngularDamping() const = 0;
        virtual void SetPrevGrabbedObjectAngularDamping(const float&) = 0;
        virtual float GetTempGrabbedObjectAngularDamping() const = 0;
        virtual void SetTempGrabbedObjectAngularDamping(const float&) = 0;
        virtual AZ::Vector3 GetGrabbedObjectAngularVelocity() const = 0;
        virtual void SetGrabbedObjectAngularVelocity(const AZ::Vector3&) = 0;
        virtual bool GetIsInGrabState() const = 0;
        virtual bool GetIsInThrowState() const = 0;
        virtual bool GetIsInRotateState() const = 0;
        virtual bool GetObjectSphereCastHit() const = 0;
    };

    using TestGemComponentRequestBus = AZ::EBus<TestGemComponentRequests>;

    class TestGemNotifications : public AZ::ComponentBus
    {
    public:
        virtual void OnObjectSphereCastHit() = 0;
        virtual void OnGrabObject() = 0;
        virtual void OnEndGrabObject() = 0;
        virtual void OnRotateObject() = 0;
        virtual void OnEndRotateObject() = 0;
        virtual void OnThrowObject() = 0;
        virtual void OnThrowStateCounterZero() = 0;
        virtual void OnMaxThrowDistance() = 0;
    };

    using TestGemNotificationBus = AZ::EBus<TestGemNotifications>;

    class TestGemNotificationHandler
        : public TestGemNotificationBus::Handler
        , public AZ::BehaviorEBusHandler
    {
    public:
        AZ_EBUS_BEHAVIOR_BINDER(TestGemNotificationHandler,
            "{2F5A85D9-94C0-47EA-8CCE-5CFD1FAE8A7E}",
            AZ::SystemAllocator, OnObjectSphereCastHit, OnGrabObject, OnEndGrabObject, OnRotateObject, OnEndRotateObject, OnThrowObject, OnThrowStateCounterZero, OnMaxThrowDistance);

        void OnObjectSphereCastHit() override
        {
            Call(FN_OnObjectSphereCastHit);
        }
        void OnGrabObject() override
        {
            Call(FN_OnGrabObject);
        }
        void OnEndGrabObject() override
        {
            Call(FN_OnEndGrabObject);
        }
        void OnRotateObject() override
        {
            Call(FN_OnRotateObject);
        }
        void OnEndRotateObject() override
        {
            Call(FN_OnEndRotateObject);
        }
        void OnThrowObject() override
        {
            Call(FN_OnThrowObject);
        }
        void OnThrowStateCounterZero() override
        {
            Call(FN_OnThrowStateCounterZero);
        }
        void OnMaxThrowDistance() override
        {
            Call(FN_OnMaxThrowDistance);
        }
    };
} // namespace TestGem