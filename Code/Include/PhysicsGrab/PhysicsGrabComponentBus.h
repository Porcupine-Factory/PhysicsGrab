#pragma once

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzFramework/Physics/Collision/CollisionLayers.h>
#include <PhysicsGrab/PhysicsGrabTypeIds.h>

namespace PhysicsGrab
{
    // Forward declaration for the enum defined in PhysicsGrabComponent
    enum class PhysicsGrabStates;

    class PhysicsGrabComponentRequests : public AZ::ComponentBus
    {
    public:
        ~PhysicsGrabComponentRequests() override = default;

        virtual AZ::EntityId GetGrabbingEntityId() const = 0;
        virtual AZ::EntityId GetActiveCameraEntityId() const = 0;
        virtual AZ::EntityId GetDetectedObjectEntityId() const = 0;
        virtual void SetDetectedObjectEntityId(const AZ::EntityId&) = 0;
        virtual AZ::EntityId GetGrabbedObjectEntityId() const = 0;
        virtual void SetGrabbedObjectEntityId(const AZ::EntityId&) = 0;
        virtual AZ::EntityId GetThrownGrabbedObjectEntityId() const = 0;
        virtual void SetThrownGrabbedObjectEntityId(const AZ::EntityId&) = 0;
        virtual AZStd::string GetGrabbedCollisionGroupName() const = 0;
        virtual void SetGrabbedCollisionGroupByName(const AZStd::string&) = 0;
        virtual AzPhysics::CollisionGroup GetGrabbedCollisionGroup() const = 0;
        virtual void SetGrabbedCollisionGroup(const AzPhysics::CollisionGroup&) = 0;
        virtual bool GetCollisionLayerIsInGrabbedGroup(const AzPhysics::CollisionLayer&) const = 0;
        virtual bool GetCollisionLayerNameIsInGrabbedGroup(const AZStd::string&) const = 0;
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
        virtual void SetGrabbingEntity(const AZ::EntityId&) = 0;
        virtual bool GetGrabEnableToggle() const = 0;
        virtual void SetGrabEnableToggle(const bool&) = 0;
        virtual bool GetRotateEnableToggle() const = 0;
        virtual void SetRotateEnableToggle(const bool&) = 0;
        virtual bool GetGrabMaintained() const = 0;
        virtual void SetGrabMaintained(const bool&) = 0;
        virtual bool GetKinematicWhileHeld() const = 0;
        virtual void SetKinematicWhileHeld(const bool&) = 0;
        virtual float GetGrabKeyValue() const = 0;
        virtual void SetGrabKeyValue(const float&) = 0;
        virtual float GetThrowKeyValue() const = 0;
        virtual void SetThrowKeyValue(const float&) = 0;
        virtual float GetRotateKeyValue() const = 0;
        virtual void SetRotateKeyValue(const float&) = 0;
        virtual float GetPitchKeyValue() const = 0;
        virtual void SetPitchKeyValue(const float&, const bool&) = 0;
        virtual float GetYawKeyValue() const = 0;
        virtual void SetYawKeyValue(const float&, const bool&) = 0;
        virtual float GetRollKeyValue() const = 0;
        virtual void SetRollKeyValue(const float&, const bool&) = 0;
        virtual AZ::EntityId GetMeshEntityId() const = 0;
        virtual void SetMeshEntityId(const AZ::EntityId&) = 0;
        virtual AZStd::string GetMeshTagName() const = 0;
        virtual void SetMeshTagName(const AZStd::string&) = 0;
        virtual float GetGrabbedDistanceKeyValue() const = 0;
        virtual void SetGrabbedDistanceKeyValue(const float&, const bool&) = 0;
        virtual float GetGrabbedObjectDistance() const = 0;
        virtual void SetGrabbedObjectDistance(const float&) = 0;
        virtual float GetMinGrabbedObjectDistance() const = 0;
        virtual void SetMinGrabbedObjectDistance(const float&) = 0;
        virtual float GetMaxGrabbedObjectDistance() const = 0;
        virtual void SetMaxGrabbedObjectDistance(const float&) = 0;
        virtual float GetGrabbedObjectDistanceSpeed() const = 0;
        virtual void SetGrabbedObjectDistanceSpeed(const float&) = 0;
        virtual float GetMaxDropDistance() const = 0;
        virtual void SetMaxDropDistance(const float&) = 0;
        virtual bool GetEnableMaxDropDistance() const = 0;
        virtual void SetEnableMaxDropDistance(const bool&) = 0;
        virtual float GetGrabResponse() const = 0;
        virtual void SetGrabResponse(const float&) = 0;
        virtual bool GetStayInIdleState() const = 0;
        virtual void SetStayInIdleState(const bool&) = 0;
        virtual bool GetHoldKeyToCheckUntilHit() const = 0;
        virtual void SetHoldKeyToCheckUntilHit(const bool&) = 0;
        virtual bool GetDynamicTidalLock() const = 0;
        virtual void SetDynamicTidalLock(const bool&) = 0;
        virtual bool GetKinematicTidalLock() const = 0;
        virtual void SetKinematicTidalLock(const bool&) = 0;
        virtual bool GetTidalLock() const = 0;
        virtual void SetTidalLock(const bool&) = 0;
        virtual bool GetUseFPControllerForGrab() const = 0;
        virtual void SetUseFPControllerForGrab(const bool&) = 0;
        virtual bool GetFullTidalLockForFPC() const = 0;
        virtual void SetFullTidalLockForFPC(const bool&) = 0;
        virtual bool GetTidalLockAndUseFPController() const = 0;
        virtual bool GetMeshSmoothing() const = 0;
        virtual void SetMeshSmoothing(const bool&) = 0;
        virtual float GetDynamicYawRotateScale() const = 0;
        virtual void SetDynamicYawRotateScale(const float&) = 0;
        virtual float GetDynamicPitchRotateScale() const = 0;
        virtual void SetDynamicPitchRotateScale(const float&) = 0;
        virtual float GetDynamicRollRotateScale() const = 0;
        virtual void SetDynamicRollRotateScale(const float&) = 0;
        virtual float GetKinematicYawRotateScale() const = 0;
        virtual void SetKinematicYawRotateScale(const float&) = 0;
        virtual float GetKinematicPitchRotateScale() const = 0;
        virtual void SetKinematicPitchRotateScale(const float&) = 0;
        virtual float GetKinematicRollRotateScale() const = 0;
        virtual void SetKinematicRollRotateScale(const float&) = 0;
        virtual float GetVelocityCompDampRate() const = 0;
        virtual void SetVelocityCompDampRate(const float&) = 0;
        virtual float GetAngularVelocityDampRate() const = 0;
        virtual void SetAngularVelocityDampRate(const float&) = 0;
        virtual bool GetVelocityCompensation() const = 0;
        virtual void SetVelocityCompensation(const bool&) = 0;
        virtual bool GetSmoothDynamicRotation() const = 0;
        virtual void SetSmoothDynamicRotation(const bool&) = 0;
        virtual float GetThrowImpulse() const = 0;
        virtual void SetThrowImpulse(const float&) = 0;
        virtual float GetGrabbedObjectThrowStateCounter() const = 0;
        virtual void SetGrabbedObjectThrowStateCounter(const float&) = 0;
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
        virtual float GetCurrentGrabbedObjectLinearDamping() const = 0;
        virtual void SetCurrentGrabbedObjectLinearDamping(const float&) = 0;
        virtual float GetPrevGrabbedObjectLinearDamping() const = 0;
        virtual void SetPrevGrabbedObjectLinearDamping(const float&) = 0;
        virtual float GetTempGrabbedObjectLinearDamping() const = 0;
        virtual void SetTempGrabbedObjectLinearDamping(const float&) = 0;
        virtual AZ::Vector3 GetGrabbedObjectAngularVelocity() const = 0;
        virtual void SetGrabbedObjectAngularVelocity(const AZ::Vector3&) = 0;
        virtual bool GetInitialAngularVelocityZero() const = 0;
        virtual void SetInitialAngularVelocityZero(const bool&) = 0;
        virtual AZStd::string GetStateString() const = 0;
        virtual void ForceTransition(const PhysicsGrabStates&) = 0;
        virtual void ForceGrab(const AZ::EntityId&) = 0;
        virtual void SetStateLocked(const bool&) = 0;
        virtual bool GetStateLocked() const = 0;
        virtual bool GetDisableGravityWhileHeld() const = 0;
        virtual void SetDisableGravityWhileHeld(const bool&) = 0;
        virtual bool GetOffsetGrab() const = 0;
        virtual void SetOffsetGrab(const bool&) = 0;
        virtual bool GetGravityAppliesToPointRotation() const = 0;
        virtual void SetGravityAppliesToPointRotation(const bool&) = 0;
        virtual bool GetMassIndependentThrow() const = 0;
        virtual void SetMassIndependentThrow(const bool&) = 0;
        virtual bool GetIsObjectGrabbable() const = 0;
        virtual bool GetDetectInIdle() const = 0;
        virtual void SetDetectInIdle(const bool&) = 0;
        virtual bool GetEnablePIDHeldDynamics() const = 0;
        virtual void SetEnablePIDHeldDynamics(const bool&) = 0;
        virtual bool GetMassIndependentHeldPID() const = 0;
        virtual void SetMassIndependentHeldPID(const bool&) = 0;
        virtual float GetHeldProportionalGain() const = 0;
        virtual void SetHeldProportionalGain(const float&) = 0;
        virtual float GetHeldIntegralGain() const = 0;
        virtual void SetHeldIntegralGain(const float&) = 0;
        virtual float GetHeldDerivativeGain() const = 0;
        virtual void SetHeldDerivativeGain(const float&) = 0;
        virtual float GetHeldIntegralWindupLimit() const = 0;
        virtual void SetHeldIntegralWindupLimit(const float&) = 0;
        virtual float GetHeldDerivativeFilterAlpha() const = 0;
        virtual void SetHeldDerivativeFilterAlpha(const float&) = 0;
        virtual bool GetEnablePIDTidalLockDynamics() const = 0;
        virtual void SetEnablePIDTidalLockDynamics(const bool&) = 0;
        virtual bool GetMassIndependentTidalLock() const = 0;
        virtual void SetMassIndependentTidalLock(const bool&) = 0;
        virtual bool GetScaleIndependentTidalLock() const = 0;
        virtual void SetScaleIndependentTidalLock(const bool&) = 0;
        virtual float GetTidalLockProportionalGain() const = 0;
        virtual void SetTidalLockProportionalGain(const float&) = 0;
        virtual float GetTidalLockIntegralGain() const = 0;
        virtual void SetTidalLockIntegralGain(const float&) = 0;
        virtual float GetTidalLockDerivativeGain() const = 0;
        virtual void SetTidalLockDerivativeGain(const float&) = 0;
        virtual float GetTidalLockIntegralWindupLimit() const = 0;
        virtual void SetTidalLockIntegralWindupLimit(const float&) = 0;
        virtual float GetTidalLockDerivativeFilterAlpha() const = 0;
        virtual void SetTidalLockDerivativeFilterAlpha(const float&) = 0;
        virtual bool GetEnableChargeThrow() const = 0;
        virtual void SetEnableChargeThrow(const bool&) = 0;
        virtual bool GetEnableChargeWhileRotating() const = 0;
        virtual void SetEnableChargeWhileRotating(const bool&) = 0;
        virtual float GetMinThrowImpulse() const = 0;
        virtual void SetMinThrowImpulse(const float&) = 0;
        virtual float GetMaxThrowImpulse() const = 0;
        virtual void SetMaxThrowImpulse(const float&) = 0;
        virtual float GetCurrentThrowImpulse() const = 0;
        virtual float GetChargeTime() const = 0;
        virtual void SetChargeTime(const float&) = 0;
        virtual float GetCurrentChargeTime() const = 0;
        virtual bool GetIsChargingThrow() const = 0;
        virtual PidController<AZ::Vector3>::DerivativeCalculationMode GetHeldDerivativeMode() const = 0;
        virtual void SetHeldDerivativeMode(const PidController<AZ::Vector3>::DerivativeCalculationMode&) = 0;
        virtual PidController<AZ::Vector3>::DerivativeCalculationMode GetTidalLockDerivativeMode() const = 0;
        virtual void SetTidalLockDerivativeMode(const PidController<AZ::Vector3>::DerivativeCalculationMode&) = 0;
        virtual AZStd::string GetGrabInputKey() const = 0;
        virtual void SetGrabInputKey(const AZStd::string&) = 0;
        virtual AZStd::string GetThrowInputKey() const = 0;
        virtual void SetThrowInputKey(const AZStd::string&) = 0;
        virtual AZStd::string GetRotateInputKey() const = 0;
        virtual void SetRotateInputKey(const AZStd::string&) = 0;
        virtual AZStd::string GetRotatePitchInputKey() const = 0;
        virtual void SetRotatePitchInputKey(const AZStd::string&) = 0;
        virtual AZStd::string GetRotateYawInputKey() const = 0;
        virtual void SetRotateYawInputKey(const AZStd::string&) = 0;
        virtual AZStd::string GetRotateRollInputKey() const = 0;
        virtual void SetRotateRollInputKey(const AZStd::string&) = 0;
        virtual AZStd::string GetGrabDistanceInputKey() const = 0;
        virtual void SetGrabDistanceInputKey(const AZStd::string&) = 0;
        virtual AZ::Vector3 GetHeldLastProportional() const = 0;
        virtual AZ::Vector3 GetHeldLastIntegral() const = 0;
        virtual AZ::Vector3 GetHeldLastDerivative() const = 0;
        virtual AZ::Vector3 GetTidalLockLastProportional() const = 0;
        virtual AZ::Vector3 GetTidalLockLastIntegral() const = 0;
        virtual AZ::Vector3 GetTidalLockLastDerivative() const = 0;
        virtual AZ::Vector3 GetTargetTranslation() const = 0;
        virtual AZ::Vector3 GetTargetRotation() const = 0;
        virtual bool GetIsInIdleState() const = 0;
        virtual bool GetIsInCheckState() const = 0;
        virtual bool GetIsInHeldState() const = 0;
        virtual bool GetIsInRotateState() const = 0;
        virtual bool GetIsInThrowState() const = 0;
        virtual bool GetObjectSphereCastHit() const = 0;
    };

    using PhysicsGrabComponentRequestBus = AZ::EBus<PhysicsGrabComponentRequests>;

    class PhysicsGrabNotifications : public AZ::ComponentBus
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
        virtual void OnChargeComplete() = 0;
    };

    using PhysicsGrabNotificationBus = AZ::EBus<PhysicsGrabNotifications>;

    class PhysicsGrabNotificationHandler
        : public PhysicsGrabNotificationBus::Handler
        , public AZ::BehaviorEBusHandler
    {
    public:
        AZ_EBUS_BEHAVIOR_BINDER(
            PhysicsGrabNotificationHandler,
            PhysicsGrabNotificationHandlerTypeId,
            AZ::SystemAllocator,
            OnObjectSphereCastHit,
            OnHoldStart,
            OnHoldStop,
            OnRotateStart,
            OnRotateStop,
            OnThrowStart,
            OnThrowStop,
            OnThrowStateCounterZero,
            OnMaxThrowDistance,
            OnChargeComplete);

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
        void OnChargeComplete() override
        {
            Call(FN_OnChargeComplete);
        }
    };
} // namespace PhysicsGrab
