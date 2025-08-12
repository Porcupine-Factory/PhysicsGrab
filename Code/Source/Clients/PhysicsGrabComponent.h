#pragma once

#include <PhysicsGrab/PhysicsGrabTypeIds.h>
#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityBus.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Math/Quaternion.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/Math/Matrix3x3.h>
#include <AzCore/std/containers/map.h>
#include <AzFramework/Components/CameraBus.h>
#include <AzFramework/Physics/Common/PhysicsSceneQueries.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <StartingPointInput/InputEventNotificationBus.h>
#include "PidController.h"
#include <LmbrCentral/Scripting/TagComponentBus.h>
#include <PhysicsGrab/PhysicsGrabComponentBus.h>

#if __has_include(<FirstPersonController/FirstPersonControllerComponentBus.h>)
#include <FirstPersonController/FirstPersonControllerComponentBus.h>
#endif

namespace PhysicsGrab
{
    enum class PhysicsGrabStates
    {
        idleState,
        checkState,
        holdState,
        rotateState,
        throwState
    };

    class PhysicsGrabComponent
        : public AZ::Component
        , public AZ::EntityBus::Handler
        , public Camera::CameraNotificationBus::Handler
        , public AZ::TickBus::Handler
        , public StartingPointInput::InputEventNotificationBus::MultiHandler
        , public PhysicsGrabComponentRequestBus::Handler
    {
    public:
        AZ_COMPONENT(PhysicsGrabComponent, PhysicsGrabComponentTypeId);

        // Provide runtime reflection
        static void Reflect(AZ::ReflectContext* rc);

        // AZ::Component overrides
        void Activate() override;
        void Deactivate() override;

        static void GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::EntityBus overrides
        void OnEntityActivated(const AZ::EntityId& entityId) override;

        // Assigns active camera to m_grabbingEntityPtr as fallback
        void OnActiveViewChanged(const AZ::EntityId& activeEntityId);

        // AZ::InputEventNotificationBus interface. Overrides OnPressed and OnReleased virtual methods.
        void OnPressed(float value) override;
        void OnReleased(float value) override;
        void OnHeld(float value) override;

        void OnTick(float deltaTime, AZ::ScriptTimePoint) override;

        AZ::Entity* GetEntityPtr(AZ::EntityId pointer) const;
        AZ::Entity* GetActiveCameraEntityPtr() const;

        // PhysicsGrabComponentRequestBus
        AZ::EntityId GetGrabbingEntityId() const override;
        AZ::EntityId GetActiveCameraEntityId() const override;
        AZ::EntityId GetDetectedObjectEntityId() const override;
        void SetDetectedObjectEntityId(const AZ::EntityId new_detectedObjectEntityId) override;
        AZ::EntityId GetGrabbedObjectEntityId() const override;
        void SetGrabbedObjectEntityId(const AZ::EntityId new_grabbedObjectEntityId) override;
        AZ::EntityId GetThrownGrabbedObjectEntityId() const override;
        void SetThrownGrabbedObjectEntityId(const AZ::EntityId new_thrownGrabbedObjectEntityId) override;
        AZStd::string GetGrabbedCollisionGroup() const override;
        void SetGrabbedCollisionGroup(const AZStd::string& new_grabbedCollisionGroupId) override;
        AZStd::string GetCurrentGrabbedCollisionLayerName() const override;
        void SetCurrentGrabbedCollisionLayerByName(const AZStd::string& new_currentGrabbedCollisionLayerName) override;
        AzPhysics::CollisionLayer GetCurrentGrabbedCollisionLayer() const override;
        void SetCurrentGrabbedCollisionLayer(const AzPhysics::CollisionLayer& new_currentGrabbedCollisionLayer) override;
        AZStd::string GetPrevGrabbedCollisionLayerName() const override;
        void SetPrevGrabbedCollisionLayerByName(const AZStd::string& new_prevGrabbedCollisionLayerName) override;
        AzPhysics::CollisionLayer GetPrevGrabbedCollisionLayer() const override;
        void SetPrevGrabbedCollisionLayer(const AzPhysics::CollisionLayer& new_prevGrabbedCollisionLayer) override;
        AZStd::string GetTempGrabbedCollisionLayerName() const override;
        void SetTempGrabbedCollisionLayerByName(const AZStd::string& new_tempGrabbedCollisionLayerName) override;
        AzPhysics::CollisionLayer GetTempGrabbedCollisionLayer() const override;
        void SetTempGrabbedCollisionLayer(const AzPhysics::CollisionLayer& new_tempGrabbedCollisionLayer) override;
        void SetGrabbingEntity(const AZ::EntityId new_grabbingEntityId) override;
        AZ::EntityId GetMeshEntityId() const override;
        void SetMeshEntityId(const AZ::EntityId& new_meshEntityId) override;
        AZStd::string GetMeshTagName() const override;
        void SetMeshTagName(const AZStd::string& new_meshTagName) override;
        AZStd::string GetStateString() const override;
        bool GetIsInIdleState() const override;
        bool GetIsInCheckState() const override;
        bool GetIsInHeldState() const override;
        bool GetIsInRotateState() const override;
        bool GetIsInThrowState() const override;
        bool GetObjectSphereCastHit() const override;
        bool GetStayInIdleState() const override;
        void SetStayInIdleState(const bool& new_stayInIdleState) override;
        bool GetGrabEnableToggle() const override;
        void SetGrabEnableToggle(const bool& new_grabEnableToggle) override;
        bool GetRotateEnableToggle() const override;
        void SetRotateEnableToggle(const bool& new_rotateEnableToggle) override;
        float GetGrabKeyValue() const override;
        void SetGrabKeyValue(const float& new_grabKeyValue) override;
        float GetThrowKeyValue() const override;
        void SetThrowKeyValue(const float& new_throwKeyValue) override;
        float GetRotateKeyValue() const override;
        void SetRotateKeyValue(const float& new_rotateKeyValue) override;
        float GetPitchKeyValue() const override;
        void SetPitchKeyValue(const float& new_pitchKeyValue, const bool& new_ignorePitchKeyInputValue) override;
        float GetYawKeyValue() const override;
        void SetYawKeyValue(const float& new_yawKeyValue, const bool& new_ignoreYawKeyInputValue) override;
        float GetRollKeyValue() const override;
        void SetRollKeyValue(const float& new_rollKeyValue, const bool& new_ignoreRollKeyInputValue) override;
        float GetGrabbedDistanceKeyValue() const override;
        void SetGrabbedDistanceKeyValue(const float& new_grabDistanceKeyValue, const bool& new_ignoreGrabDistanceKeyInputValue) override;
        float GetGrabbedObjectDistance() const override;
        void SetGrabbedObjectDistance(const float& new_grabDistance) override;
        float GetMinGrabbedObjectDistance() const override;
        void SetMinGrabbedObjectDistance(const float& new_minGrabDistance) override;
        float GetMaxGrabbedObjectDistance() const override;
        void SetMaxGrabbedObjectDistance(const float& new_maxGrabDistance) override;
        float GetGrabbedObjectDistanceSpeed() const override;
        void SetGrabbedObjectDistanceSpeed(const float& new_grabDistanceSpeed) override;
        float GetGrabResponse() const override;
        void SetGrabResponse(const float& new_grabStrength) override;
        bool GetDynamicTidalLock() const override;
        void SetDynamicTidalLock(const bool& new_dynamicTidalLock) override;
        bool GetKinematicTidalLock() const override;
        void SetKinematicTidalLock(const bool& new_kinematicTidalLock) override;
        bool GetTidalLock() const override;
        void SetTidalLock(const bool& new_tidalLock) override;
        bool GetUseFPCControllerForGrab() const override;
        void SetUseFPCControllerForGrab(const bool& new_useFPControllerForGrab) override;
        bool GetFullTidalLockForFPC() const override;
        void SetFullTidalLockForFPC(const bool& new_fullTidalLockForFPC) override;
        bool GetTidalLockAndUseFPCController() const override;
        bool GetMeshSmoothing() const override;
        void SetMeshSmoothing(const bool& new_meshSmoothing) override;
        float GetDynamicYawRotateScale() const;
        void SetDynamicYawRotateScale(const float& new_dynamicHorizontalYawcale);
        float GetDynamicPitchRotateScale() const;
        void SetDynamicPitchRotateScale(const float& new_dynamicRollRotateScale);
        float GetDynamicRollRotateScale() const;
        void SetDynamicRollRotateScale(const float& new_dynamicRollRotateScale);
        float GetKinematicYawRotateScale() const;
        void SetKinematicYawRotateScale(const float& new_kinematicYawRotateScale);
        float GetKinematicPitchRotateScale() const;
        void SetKinematicPitchRotateScale(const float& new_kinematicPitchRotateScale);
        float GetKinematicRollRotateScale() const;
        void SetKinematicRollRotateScale(const float& new_kinematicRollRotateScale);
        float GetVelocityCompDampRate() const;
        void SetVelocityCompDampRate(const float& new_velocityCompDampRate);
        float GetAngularVelocityDampRate() const;
        void SetAngularVelocityDampRate(const float& new_angularVelocityDampRate);
        bool GetVelocityCompensation() const;
        void SetVelocityCompensation(const bool& new_velocityCompensation);
        bool GetSmoothDynamicRotation() const;
        void SetSmoothDynamicRotation(const bool& new_smoothDynamicRotation);
        float GetThrowImpulse() const override;
        void SetThrowImpulse(const float& new_throwImpulse) override;
        float GetGrabbedObjectThrowStateCounter() const override;
        void SetGrabbedObjectThrowStateCounter(const float& new_throwStateCounter) override;
        float GetGrabbedObjectThrowStateTime() const override;
        void SetGrabbedObjectThrowStateTime(const float& new_throwStateMaxTime) override;
        bool GetEnableChargeThrow() const override;
        void SetEnableChargeThrow(const bool& new_enableChargeThrow) override;
        float GetMinThrowImpulse() const override;
        void SetMinThrowImpulse(const float& new_minThrowImpulse) override;
        float GetMaxThrowImpulse() const override;
        void SetMaxThrowImpulse(const float& new_maxThrowImpulse) override;
        float GetCurrentThrowImpulse() const override;
        float GetChargeTime() const override;
        void SetChargeTime(const float& new_chargeTime) override;
        float GetCurrentChargeTime() const override;
        bool GetEnableChargeWhileRotating() const override;
        void SetEnableChargeWhileRotating(const bool& new_enableChargeWhileRotating) override;
        bool GetIsChargingThrow() const override;
        float GetSphereCastRadius() const override;
        void SetSphereCastRadius(const float& new_sphereCastRadius) override;
        float GetSphereCastDistance() const override;
        void SetSphereCastDistance(const float& new_sphereCastDistance) override;
        bool GetGrabbedObjectKinematicElseDynamic() const override;
        void SetGrabbedObjectKinematicElseDynamic(const bool& isKinematic) override;
        bool GetInitialGrabbedObjectIsKinematic() const override;
        float GetCurrentGrabbedObjectAngularDamping() const override;
        void SetCurrentGrabbedObjectAngularDamping(const float& new_currentObjectAngularDamping) override;
        float GetPrevGrabbedObjectAngularDamping() const override;
        void SetPrevGrabbedObjectAngularDamping(const float& new_prevObjectAngularDamping) override;
        float GetTempGrabbedObjectAngularDamping() const override;
        void SetTempGrabbedObjectAngularDamping(const float& new_tempObjectAngularDamping) override;
        float GetCurrentGrabbedObjectLinearDamping() const override;
        void SetCurrentGrabbedObjectLinearDamping(const float& new_currentObjectLinearDamping) override;
        float GetPrevGrabbedObjectLinearDamping() const override;
        void SetPrevGrabbedObjectLinearDamping(const float& new_prevObjectLinearDamping) override;
        float GetTempGrabbedObjectLinearDamping() const override;
        void SetTempGrabbedObjectLinearDamping(const float& new_tempObjectLinearDamping) override;
        AZ::Vector3 GetGrabbedObjectAngularVelocity() const override;
        void SetGrabbedObjectAngularVelocity(const AZ::Vector3& new_grabbedObjectAngularVelocity) override;
        bool GetInitialAngularVelocityZero() const override;
        void SetInitialAngularVelocityZero(const bool& new_initialAngularVelocityZero) override;
        void ForceTransition(const PhysicsGrabStates& targetState) override;
        void SetStateLocked(const bool& isLocked) override;
        bool GetStateLocked() const override;
        bool GetDisableGravityWhileHeld() const override;
        void SetDisableGravityWhileHeld(const bool& new_disableGravityWhileHeld) override;
        bool GetOffsetGrab() const override;
        void SetOffsetGrab(const bool& new_offsetGrab) override;
        bool GetGravityAppliesToPointRotation() const override;
        void SetGravityAppliesToPointRotation(const bool& new_gravityAppliesToPointRotation) override;
        bool GetMassIndependentThrow() const override;
        void SetMassIndependentThrow(const bool& new_massIndependentThrow) override;
        bool GetEnablePIDHeldDynamics() const override;
        void SetEnablePIDHeldDynamics(const bool& new_enablePIDHeldDynamics) override;
        bool GetMassIndependentHeldPID() const override;
        void SetMassIndependentHeldPID(const bool& new_massIndependentHeldPID) override;
        float GetHeldProportionalGain() const override;
        void SetHeldProportionalGain(const float& new_heldProportionalGain) override;
        float GetHeldIntegralGain() const override;
        void SetHeldIntegralGain(const float& new_heldIntegralGain) override;
        float GetHeldDerivativeGain() const override;
        void SetHeldDerivativeGain(const float& new_heldDerivativeGain) override;
        float GetHeldIntegralWindupLimit() const override;
        void SetHeldIntegralWindupLimit(const float& new_heldIntegralWindupLimit) override;
        float GetHeldDerivativeFilterAlpha() const override;
        void SetHeldDerivativeFilterAlpha(const float& new_heldDerivativeFilterAlpha) override;
        bool GetEnablePIDTidalLockDynamics() const override;
        void SetEnablePIDTidalLockDynamics(const bool& new_enablePIDTidalLockDynamics) override;
        bool GetMassIndependentTidalLock() const override;
        void SetMassIndependentTidalLock(const bool& new_massIndependentTidalLock) override;
        bool GetScaleIndependentTidalLock() const;
        void SetScaleIndependentTidalLock(const bool& new_scaleIndependentTidalLock);
        float GetTidalLockProportionalGain() const override;
        void SetTidalLockProportionalGain(const float& new_tidalLockProportionalGain) override;
        float GetTidalLockIntegralGain() const override;
        void SetTidalLockIntegralGain(const float& new_tidalLockIntegralGain) override;
        float GetTidalLockDerivativeGain() const override;
        void SetTidalLockDerivativeGain(const float& new_tidalLockDerivativeGain) override;
        float GetTidalLockIntegralWindupLimit() const override;
        void SetTidalLockIntegralWindupLimit(const float& new_tidalLockIntegralWindupLimit) override;
        float GetTidalLockDerivativeFilterAlpha() const override;
        void SetTidalLockDerivativeFilterAlpha(const float& new_tidalLockDerivativeFilterAlpha) override;
        PidController<AZ::Vector3>::DerivativeCalculationMode GetHeldDerivativeMode() const override;
        void SetHeldDerivativeMode(const PidController<AZ::Vector3>::DerivativeCalculationMode& new_heldDerivativeMode) override;
        PidController<AZ::Vector3>::DerivativeCalculationMode GetTidalLockDerivativeMode() const override;
        void SetTidalLockDerivativeMode(const PidController<AZ::Vector3>::DerivativeCalculationMode& new_tidalLockDerivativeMode) override;
     
        // Input binding getters and setters
        AZStd::string GetGrabInputKey() const override;
        void SetGrabInputKey(const AZStd::string& keyName) override;
        AZStd::string GetThrowInputKey() const override;
        void SetThrowInputKey(const AZStd::string& keyName) override;
        AZStd::string GetRotateInputKey() const override;
        void SetRotateInputKey(const AZStd::string& keyName) override;
        AZStd::string GetRotatePitchInputKey() const override;
        void SetRotatePitchInputKey(const AZStd::string& keyName) override;
        AZStd::string GetRotateYawInputKey() const override;
        void SetRotateYawInputKey(const AZStd::string& keyName) override;
        AZStd::string GetRotateRollInputKey() const override;
        void SetRotateRollInputKey(const AZStd::string& keyName) override;
        AZStd::string GetGrabDistanceInputKey() const override;
        void SetGrabDistanceInputKey(const AZStd::string& keyName) override;

    private:
        AZ::Entity* m_grabbingEntityPtr = nullptr;
        AZ::Entity* m_meshEntityPtr = nullptr;
  
        AzPhysics::SceneHandle m_attachedSceneHandle = AzPhysics::InvalidSceneHandle;
        AzPhysics::SceneEvents::OnSceneSimulationStartHandler m_sceneSimulationStartHandler;
        AzPhysics::SceneEvents::OnSceneSimulationFinishHandler m_sceneSimulationFinishHandler;

        StartingPointInput::InputEventNotificationId m_grabEventId;
        AZStd::string m_strGrab = "Grab";

        StartingPointInput::InputEventNotificationId m_throwEventId;
        AZStd::string m_strThrow = "Throw";

        StartingPointInput::InputEventNotificationId m_rotateEventId;
        AZStd::string m_strRotate = "Rotate Enable";

        StartingPointInput::InputEventNotificationId m_rotatePitchEventId;
        AZStd::string m_strRotatePitch = "Rotate Pitch";

        StartingPointInput::InputEventNotificationId m_rotateYawEventId;
        AZStd::string m_strRotateYaw = "Rotate Yaw";

        StartingPointInput::InputEventNotificationId m_rotateRollEventId;
        AZStd::string m_strRotateRoll = "Rotate Roll";

        StartingPointInput::InputEventNotificationId m_grabDistanceEventId;
        AZStd::string m_strGrabDistance = "Grab Distance";

        AZStd::string m_meshTagName = "GrabMesh";

        void CheckForObjects();
        void HoldObject(float deltaTime);
        void RotateObject(float deltaTime);
        void ThrowObject();
        void TidalLock(float deltaTime);
        void UpdateGrabDistance(float deltaTime);
        void ReleaseGrabbedObject(bool notifyHoldStop, bool notifyRotateStop);
        bool HandleThrowInput(float deltaTime, bool allowCharging);
        void TransitionToThrow(bool isChargeEnabled);
        AZ::Quaternion GetEffectiveGrabbingRotation() const;
        void ReleaseMesh();
        void InterpolateMeshTransform(float deltaTime);
        void ComputeGrabbingEntityVelocity(float deltaTime);
        void OnSceneSimulationStart(float physicsTimestep);
        void OnSceneSimulationFinish([[maybe_unused]] AzPhysics::SceneHandle sceneHandle, [[maybe_unused]] float fixedDeltaTime);
        #ifdef FIRST_PERSON_CONTROLLER
        void FreezeCharacterRotation();
        #endif
        // Method to update input bindings
        void UpdateInputBinding(
            StartingPointInput::InputEventNotificationId& eventId, AZStd::string& binding, const AZStd::string& newValue);

        // PhysicsGrabNotificationBus
        void OnObjectSphereCastHit();
        void OnHoldStart();
        void OnHoldStop();
        void OnRotateStart();
        void OnRotateStop();
        void OnThrowStart();
        void OnThrowStop();
        void OnMaxThrowDistance();
        void OnThrowStateCounterZero();
        void OnChargeComplete();
        
        // State machine functions
        void ProcessStates(const float& deltaTime, bool isPhysicsUpdate = false);
        void IdleState();
        void CheckForObjectsState();
        void HoldObjectState(float deltaTime, bool isPhysicsUpdate = false);
        void RotateObjectState(float deltaTime, bool isPhysicsUpdate = false);
        void ThrowObjectState(const float &deltaTime);

        AZ::Transform m_grabbingEntityTransform = AZ::Transform::CreateIdentity();
        AZ::Transform m_grabReference = AZ::Transform::CreateIdentity();
        AZ::Transform m_prevPhysicsTransform = AZ::Transform::CreateIdentity();
        AZ::Transform m_currentPhysicsTransform = AZ::Transform::CreateIdentity();
        AZ::Transform m_originalMeshLocalTM = AZ::Transform::CreateIdentity();

        AZ::TransformInterface* m_cameraRotationTransform = nullptr;

        AZ::Quaternion m_lastEntityRotationQuat = AZ::Quaternion::CreateIdentity();
        AZ::Quaternion m_grabbedObjectRelativeQuat = AZ::Quaternion::CreateIdentity();

        AZ::Vector3 m_forwardVector = AZ::Vector3::CreateZero();
        AZ::Vector3 m_rightVector = AZ::Vector3::CreateZero();
        AZ::Vector3 m_upVector = AZ::Vector3::CreateZero();
        AZ::Vector3 m_grabbedObjectTranslation = AZ::Vector3::CreateZero();
        AZ::Vector3 m_grabbedObjectAngularVelocity = AZ::Vector3::CreateZero();
        AZ::Vector3 m_grabbingEntityVelocity = AZ::Vector3::CreateZero();
        AZ::Vector3 m_currentGrabEntityTranslation = AZ::Vector3::CreateZero();
        AZ::Vector3 m_prevGrabbingEntityTranslation = AZ::Vector3::CreateZero();
        AZ::Vector3 m_currentCompensationVelocity = AZ::Vector3::CreateZero();
        AZ::Vector3 m_currentAngularVelocity = AZ::Vector3::CreateZero();
        AZ::Vector3 m_hitPosition = AZ::Vector3::CreateZero();
        AZ::Vector3 m_localGrabOffset = AZ::Vector3::CreateZero();

        AZ::EntityId m_detectedObjectEntityId;
        AZ::EntityId m_meshEntityId;
        AZ::EntityId m_grabbingEntityId;
        AZ::EntityId m_grabbedObjectEntityId;
        AZ::EntityId m_thrownGrabbedObjectEntityId;

        AzPhysics::CollisionGroups::Id m_grabbedCollisionGroupId = AzPhysics::CollisionGroups::Id();
        AzPhysics::CollisionGroup m_grabbedCollisionGroup = AzPhysics::CollisionGroup::All;

        AzPhysics::CollisionLayer m_prevGrabbedCollisionLayer;
        AzPhysics::CollisionLayer m_currentGrabbedCollisionLayer;
        AzPhysics::CollisionLayer m_tempGrabbedCollisionLayer;
        AZStd::string m_currentGrabbedCollisionLayerName;

        // Event value multipliers
        float m_grabKeyValue = 0.f;
        float m_throwKeyValue = 0.f;
        float m_rotateKeyValue = 0.f;
        float m_prevGrabKeyValue = 0.f;
        float m_prevRotateKeyValue = 0.f;
        float m_grabDistanceKeyValue = 0.f;
        float m_pitchKeyValue = 0.f;
        float m_yawKeyValue = 0.f;
        float m_rollKeyValue = 0.f;

        float m_physicsTimeAccumulator = 0.0f;
        float m_physicsTimestep = 1.0f / 60.0f;
        float m_minGrabDistance = 1.5f;
        float m_maxGrabDistance = 4.5f;
        float m_grabDistance = 0.0f;
        float m_velocityCompDampRate = 9.f;
        float m_angularVelocityDampRate = 25.0f;
        float m_kinematicYawRotateScale = 1.2f;
        float m_kinematicPitchRotateScale = 2.133f;
        float m_kinematicRollRotateScale = 1.2f;
        float m_dynamicYawRotateScale = 0.4f;
        float m_dynamicPitchRotateScale = 0.711f;
        float m_dynamicRollRotateScale = 0.4f;
        float m_prevObjectAngularDamping = 0.f;
        float m_currentObjectAngularDamping = 0.f;
        float m_tempObjectAngularDamping = 0.15f;
        float m_tempObjectLinearDamping = 0.05f;
        float m_prevObjectLinearDamping = 0.f;
        float m_currentObjectLinearDamping = 0.0f;
        float m_grabDistanceSpeed = 0.4f;
        float m_grabResponse = 10.f;
        float m_throwImpulse = 8.0f;
        float m_minThrowImpulse = 3.0f;
        float m_maxThrowImpulse = 20.0f;
        float m_chargeTime = 3.0f;
        float m_currentChargeTime = 0.0f;
        float m_currentThrowImpulse = 0.0f;
        float m_prevThrowKeyValue = 0.0f;
        float m_grabbedObjectMass = 1.0f;
        float m_sphereCastRadius = 0.3f;
        float m_sphereCastDistance = 4.5f;
        float m_throwStateMaxTime = 0.5f;
        float m_throwStateCounter = 0.0f;
        float m_combinedGrabDistance = 0.0f;
        float m_prevGravityEnabled = 0.0f;
        float m_heldProportionalGain = 80.f;
        float m_heldIntegralGain = 0.0f;
        float m_heldDerivativeGain = 11.f;
        float m_heldIntegralWindupLimit = 100.0f;
        float m_heldDerivativeFilterAlpha = 0.8f;
        float m_tidalLockProportionalGain = 100.0f;
        float m_tidalLockIntegralGain = 0.0f;
        float m_tidalLockDerivativeGain = 9.0f;
        float m_tidalLockIntegralWindupLimit = 200.0f;
        float m_tidalLockDerivativeFilterAlpha = 0.8f;
        float m_effectiveInertiaFactor = 0.0f;
        float m_pitch = 0.0f;
        float m_yaw = 0.0f;
        float m_roll = 0.0f;
        float m_accumPitch = 0.0f;
        float m_accumYaw = 0.0f;
        float m_accumRoll = 0.0f;

        bool m_needsCameraFallback = false;
        bool m_enableChargeThrow = false;
        bool m_isChargingThrow = false;
        bool m_enableChargeWhileRotating = false;
        bool m_hasNotifiedChargeComplete = false;
        bool m_meshSmoothing = true;
        bool m_velocityCompensation = true;
        bool m_smoothDynamicRotation = true;
        bool m_disableGravityWhileHeld = false;
        bool m_useFPControllerForGrab = true;
        bool m_grabEnableToggle = false;
        bool m_kinematicWhileHeld = false;
        bool m_freezeCharacterRotation = true;
        bool m_rotateEnableToggle = false;
        bool m_tidalLock = true;
        bool m_dynamicTidalLock = true;
        bool m_kinematicTidalLock = true;
        bool m_fullTidalLockForFPC = true;
        bool m_isInitialObjectKinematic = false;
        bool m_grabMaintained = true;
        bool m_enablePIDTidalLockDynamics = true;
        bool m_isInGrabState = false;
        bool m_isInRotateState = false;
        bool m_isInThrowState = false;
        bool m_isObjectKinematic = false;
        bool m_objectSphereCastHit = false;
        bool m_stayInIdleState = false;
        bool m_initialAngularVelocityZero = true;
        bool m_ignoreGrabDistanceKeyInputValue = true;
        bool m_ignoreYawKeyInputValue = true;
        bool m_ignorePitchKeyInputValue = true;
        bool m_ignoreRollKeyInputValue = true;
        bool m_forceTransition = false;
        bool m_isStateLocked = false;
        bool m_enablePIDHeldDynamics = true;
        bool m_massIndependentThrow = true;
        bool m_massIndependentHeldPID = true;
        bool m_massIndependentTidalLock = true;
        bool m_scaleIndependentTidalLock = true;
        bool m_offsetGrab = false;
        bool m_gravityAppliesToPointRotation = true;

        PhysicsGrabStates m_state = PhysicsGrabStates::idleState;
        PhysicsGrabStates m_targetState = PhysicsGrabStates::idleState;

        PidController<AZ::Vector3> m_pidController;
        PidController<AZ::Vector3> m_tidalLockPidController;

        PidController<AZ::Vector3>::DerivativeCalculationMode m_heldDerivativeMode = PidController<AZ::Vector3>::Velocity;
        PidController<AZ::Vector3>::DerivativeCalculationMode m_tidalLockDerivativeMode = PidController<AZ::Vector3>::ErrorRate;

        AZStd::map<PhysicsGrabStates, AZStd::string> m_statesMap = {
          {PhysicsGrabStates::idleState,   "idleState"},
          {PhysicsGrabStates::checkState,  "checkState"},
          {PhysicsGrabStates::holdState,   "holdState"},
          {PhysicsGrabStates::rotateState, "rotateState"},
          {PhysicsGrabStates::throwState,  "throwState"}
        };
    };
} // namespace PhysicsGrab
