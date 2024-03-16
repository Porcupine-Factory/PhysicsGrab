#pragma once

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzFramework/Physics/Collision/CollisionLayers.h>

namespace ObjectInteraction
{
    class ObjectInteractionComponentRequests : public AZ::ComponentBus
    {
    public:
        ~ObjectInteractionComponentRequests() override = default;

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
        virtual float GetPitchKeyValue() const = 0;
        virtual void SetPitchKeyValue(const float&) = 0;
        virtual float GetYawKeyValue() const = 0;
        virtual void SetYawKeyValue(const float&) = 0;
        virtual float GetRollKeyValue() const = 0;
        virtual void SetRollKeyValue(const float&) = 0;
        virtual float GetGrabbedDistanceKeyValue() const = 0;
        virtual void SetGrabbedDistanceKeyValue(const float&) = 0;
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
        virtual bool GetStayInIdleState() const = 0;
        virtual void SetStayInIdleState(const bool&) = 0;
        virtual bool GetDynamicTidalLock() const = 0;
        virtual void SetDynamicTidalLock(const bool&) = 0;
        virtual bool GetKinematicTidalLock() const = 0;
        virtual void SetKinematicTidalLock(const bool&) = 0;
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
        virtual bool GetGrabbedObjectKinematicElseDynamic() const = 0;
        virtual void SetGrabbedObjectKinematicElseDynamic(const bool&) = 0;
        virtual bool GetInitialGrabbedObjectIsKinematic() const = 0;
        virtual float GetCurrentGrabbedObjectAngularDamping() const = 0;
        virtual void SetCurrentGrabbedObjectAngularDamping(const float&) = 0;
        virtual float GetPrevGrabbedObjectAngularDamping() const = 0;
        virtual void SetPrevGrabbedObjectAngularDamping(const float&) = 0;
        virtual float GetTempGrabbedObjectAngularDamping() const = 0;
        virtual void SetTempGrabbedObjectAngularDamping(const float&) = 0;
        virtual AZ::Vector3 GetGrabbedObjectAngularVelocity() const = 0;
        virtual void SetGrabbedObjectAngularVelocity(const AZ::Vector3&) = 0;
        virtual bool GetInitialAngularVelocityZero() const = 0;
        virtual void SetInitialAngularVelocityZero(const bool&) = 0;
        virtual AZStd::string GetStateString() const = 0;
        virtual bool GetIsInIdleState() const = 0;
        virtual bool GetIsInCheckState() const = 0;
        virtual bool GetIsInHeldState() const = 0;
        virtual bool GetIsInRotateState() const = 0;
        virtual bool GetIsInThrowState() const = 0;
        virtual bool GetObjectSphereCastHit() const = 0;
    };

    using ObjectInteractionComponentRequestBus = AZ::EBus<ObjectInteractionComponentRequests>;

    class ObjectInteractionNotifications : public AZ::ComponentBus
    {
    public:
        virtual void OnObjectSphereCastHit() = 0;
        virtual void OnHoldStart() = 0;
        virtual void OnHoldStop() = 0;
        virtual void OnRotateStart() = 0;
        virtual void OnRotateStop() = 0;
        virtual void OnThrowStart() = 0;
        virtual void OnThrowStop() = 0;
        virtual void OnThrowStateCounterZero() = 0;
        virtual void OnMaxThrowDistance() = 0;
    };

    using ObjectInteractionNotificationBus = AZ::EBus<ObjectInteractionNotifications>;

    class ObjectInteractionNotificationHandler
        : public ObjectInteractionNotificationBus::Handler
        , public AZ::BehaviorEBusHandler
    {
    public:
        AZ_EBUS_BEHAVIOR_BINDER(
            ObjectInteractionNotificationHandler,
            "{4B8267D2-7D33-4AE4-AA30-FD933D6CF0E3}",
            AZ::SystemAllocator,
            OnObjectSphereCastHit,
            OnHoldStart,
            OnHoldStop,
            OnRotateStart,
            OnRotateStop,
            OnThrowStart,
            OnThrowStop,
            OnThrowStateCounterZero,
            OnMaxThrowDistance);

        void OnObjectSphereCastHit() override
        {
            Call(FN_OnObjectSphereCastHit);
        }
        void OnHoldStart() override
        {
            Call(FN_OnHoldStart);
        }
        void OnHoldStop() override
        {
            Call(FN_OnHoldStop);
        }
        void OnRotateStart() override
        {
            Call(FN_OnRotateStart);
        }
        void OnRotateStop() override
        {
            Call(FN_OnRotateStop);
        }
        void OnThrowStart() override
        {
            Call(FN_OnThrowStart);
        }
        void OnThrowStop() override
        {
            Call(FN_OnThrowStop);
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
} // namespace ObjectInteraction
