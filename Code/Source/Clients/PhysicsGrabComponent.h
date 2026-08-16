#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityBus.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Math/Matrix3x3.h>
#include <AzCore/Math/Quaternion.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/std/containers/map.h>

#include <AzFramework/Components/CameraBus.h>
#include <AzFramework/Physics/Common/PhysicsSceneQueries.h>
#include <AzFramework/Physics/PhysicsScene.h>

#include <PhysicsGrab/PidController.h>
#include <PhysicsGrab/PhysicsGrabComponentBus.h>
#include <PhysicsGrab/PhysicsGrabTypeIds.h>
#ifdef NETWORKPHYSICSGRAB
#include <PhysicsGrab/NetworkPhysicsGrabComponentBus.h>
#endif

#include <LmbrCentral/Scripting/TagComponentBus.h>

#include <StartingPointInput/InputEventNotificationBus.h>

#if __has_include(<FirstPersonController/FirstPersonControllerComponentBus.h>)
#include <FirstPersonController/FirstPersonControllerComponentBus.h>
#ifndef FIRST_PERSON_CONTROLLER
#define FIRST_PERSON_CONTROLLER
#endif
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

    class NetworkPhysicsGrabComponent;

    class PhysicsGrabComponent
        : public AZ::Component
        , public AZ::EntityBus::Handler
        , public Camera::CameraNotificationBus::Handler
        , public AZ::TickBus::Handler
#ifdef NETWORKPHYSICSGRAB
        , public NetworkPhysicsGrabComponentNotificationBus::Handler
#endif
        , public StartingPointInput::InputEventNotificationBus::MultiHandler
        , public PhysicsGrabComponentRequestBus::Handler
    {
        friend class NetworkPhysicsGrabComponent;
        friend class NetworkPhysicsGrabComponentController;

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

        // NetworkPhysicsGrabComponentNotificationBus
        void OnNetworkTickStart(const float deltaTime, const bool server, const AZ::EntityId& entity);
        void OnNetworkTickFinish(const float deltaTime, const bool server, const AZ::EntityId& entity);

        AZ::Entity* GetEntityPtr(AZ::EntityId pointer) const;
        AZ::Entity* GetActiveCameraEntityPtr() const;

        // PhysicsGrabComponentRequestBus
        AZ::EntityId GetGrabbingEntityId() const override;
        AZ::EntityId GetActiveCameraEntityId() const override;
        AZ::EntityId GetDetectedObjectEntityId() const override;
        void SetDetectedObjectEntityId(const AZ::EntityId& detectedObjectEntityId) override;
        AZ::EntityId GetGrabbedObjectEntityId() const override;
        void SetGrabbedObjectEntityId(const AZ::EntityId& grabbedObjectEntityId) override;
        AZ::EntityId GetThrownGrabbedObjectEntityId() const override;
        void SetThrownGrabbedObjectEntityId(const AZ::EntityId& thrownGrabbedObjectEntityId) override;
        AZStd::string GetGrabbedCollisionGroupName() const override;
        void SetGrabbedCollisionGroupByName(const AZStd::string& grabbedCollisionGroupName) override;
        AzPhysics::CollisionGroup GetGrabbedCollisionGroup() const override;
        void SetGrabbedCollisionGroup(const AzPhysics::CollisionGroup& grabbedCollisionGroup) override;
        bool GetCollisionLayerIsInGrabbedGroup(const AzPhysics::CollisionLayer& collisionLayerToCheck) const override;
        bool GetCollisionLayerNameIsInGrabbedGroup(const AZStd::string& collisionLayerNameToCheck) const override;
        AZStd::string GetCurrentGrabbedCollisionLayerName() const override;
        void SetCurrentGrabbedCollisionLayerByName(const AZStd::string& currentGrabbedCollisionLayerName) override;
        AzPhysics::CollisionLayer GetCurrentGrabbedCollisionLayer() const override;
        void SetCurrentGrabbedCollisionLayer(const AzPhysics::CollisionLayer& currentGrabbedCollisionLayer) override;
        AZStd::string GetPrevGrabbedCollisionLayerName() const override;
        void SetPrevGrabbedCollisionLayerByName(const AZStd::string& prevGrabbedCollisionLayerName) override;
        AzPhysics::CollisionLayer GetPrevGrabbedCollisionLayer() const override;
        void SetPrevGrabbedCollisionLayer(const AzPhysics::CollisionLayer& prevGrabbedCollisionLayer) override;
        AZStd::string GetTempGrabbedCollisionLayerName() const override;
        void SetTempGrabbedCollisionLayerByName(const AZStd::string& tempGrabbedCollisionLayerName) override;
        AzPhysics::CollisionLayer GetTempGrabbedCollisionLayer() const override;
        void SetTempGrabbedCollisionLayer(const AzPhysics::CollisionLayer& tempGrabbedCollisionLayer) override;
        void SetGrabbingEntity(const AZ::EntityId& grabbingEntityId) override;
        AZ::EntityId GetMeshEntityId() const override;
        void SetMeshEntityId(const AZ::EntityId& meshEntityId) override;
        AZStd::string GetMeshTagName() const override;
        void SetMeshTagName(const AZStd::string& meshTagName) override;
        AZStd::string GetStateString() const override;
        bool GetIsInIdleState() const override;
        bool GetIsInCheckState() const override;
        bool GetIsInHeldState() const override;
        bool GetIsInRotateState() const override;
        bool GetIsInThrowState() const override;
        bool GetObjectSphereCastHit() const override;
        bool GetStayInIdleState() const override;
        void SetStayInIdleState(const bool stayInIdleState) override;
        bool GetHoldKeyToCheckUntilHit() const override;
        void SetHoldKeyToCheckUntilHit(const bool holdKeyToCheckUntilHit) override;
        bool GetGrabEnableToggle() const override;
        void SetGrabEnableToggle(const bool grabEnableToggle) override;
        bool GetRotateEnableToggle() const override;
        void SetRotateEnableToggle(const bool rotateEnableToggle) override;
        bool GetGrabMaintained() const override;
        void SetGrabMaintained(const bool grabMaintained) override;
        bool GetKinematicWhileHeld() const override;
        void SetKinematicWhileHeld(const bool kinematicWhileHeld) override;
        float GetGrabKeyValue() const override;
        void SetGrabKeyValue(const float grabKeyValue) override;
        float GetThrowKeyValue() const override;
        void SetThrowKeyValue(const float throwKeyValue) override;
        float GetRotateKeyValue() const override;
        void SetRotateKeyValue(const float rotateKeyValue) override;
        float GetPitchKeyValue() const override;
        void SetPitchKeyValue(const float pitchKeyValue, const bool ignorePitchKeyInputValue) override;
        float GetYawKeyValue() const override;
        void SetYawKeyValue(const float yawKeyValue, const bool ignoreYawKeyInputValue) override;
        float GetRollKeyValue() const override;
        void SetRollKeyValue(const float rollKeyValue, const bool ignoreRollKeyInputValue) override;
        float GetGrabbedDistanceKeyValue() const override;
        void SetGrabbedDistanceKeyValue(const float grabDistanceKeyValue, const bool ignoreGrabDistanceKeyInputValue) override;
        float GetGrabbedObjectDistance() const override;
        void SetGrabbedObjectDistance(const float grabDistance) override;
        float GetMinGrabbedObjectDistance() const override;
        void SetMinGrabbedObjectDistance(const float minGrabDistance) override;
        float GetMaxGrabbedObjectDistance() const override;
        void SetMaxGrabbedObjectDistance(const float maxGrabDistance) override;
        float GetGrabbedObjectDistanceSpeed() const override;
        void SetGrabbedObjectDistanceSpeed(const float grabDistanceSpeed) override;
        float GetMaxDropDistance() const override;
        void SetMaxDropDistance(const float maxDropDistance) override;
        bool GetEnableMaxDropDistance() const override;
        void SetEnableMaxDropDistance(const bool enableMaxDropDistance) override;
        float GetGrabResponse() const override;
        void SetGrabResponse(const float grabStrength) override;
        bool GetDynamicTidalLock() const override;
        void SetDynamicTidalLock(const bool dynamicTidalLock) override;
        bool GetKinematicTidalLock() const override;
        void SetKinematicTidalLock(const bool kinematicTidalLock) override;
        bool GetTidalLock() const override;
        void SetTidalLock(const bool tidalLock) override;
        bool GetUseFPControllerForGrab() const override;
        void SetUseFPControllerForGrab(const bool useFPControllerForGrab) override;
        bool GetFullTidalLockForFPC() const override;
        void SetFullTidalLockForFPC(const bool fullTidalLockForFPC) override;
        bool GetTidalLockAndUseFPController() const override;
        bool GetMeshSmoothing() const override;
        void SetMeshSmoothing(const bool meshSmoothing) override;
        float GetDynamicYawRotateScale() const;
        void SetDynamicYawRotateScale(const float dynamicHorizontalYawcale);
        float GetDynamicPitchRotateScale() const;
        void SetDynamicPitchRotateScale(const float dynamicRollRotateScale);
        float GetDynamicRollRotateScale() const;
        void SetDynamicRollRotateScale(const float dynamicRollRotateScale);
        float GetKinematicYawRotateScale() const;
        void SetKinematicYawRotateScale(const float kinematicYawRotateScale);
        float GetKinematicPitchRotateScale() const;
        void SetKinematicPitchRotateScale(const float kinematicPitchRotateScale);
        float GetKinematicRollRotateScale() const;
        void SetKinematicRollRotateScale(const float kinematicRollRotateScale);
        float GetVelocityCompDampRate() const;
        void SetVelocityCompDampRate(const float velocityCompDampRate);
        float GetAngularVelocityDampRate() const;
        void SetAngularVelocityDampRate(const float angularVelocityDampRate);
        bool GetVelocityCompensation() const;
        void SetVelocityCompensation(const bool velocityCompensation);
        bool GetSmoothDynamicRotation() const;
        void SetSmoothDynamicRotation(const bool smoothDynamicRotation);
        float GetThrowImpulse() const override;
        void SetThrowImpulse(const float throwImpulse) override;
        float GetGrabbedObjectThrowStateCounter() const override;
        void SetGrabbedObjectThrowStateCounter(const float throwStateCounter) override;
        float GetGrabbedObjectThrowStateTime() const override;
        void SetGrabbedObjectThrowStateTime(const float throwStateMaxTime) override;
        bool GetEnableChargeThrow() const override;
        void SetEnableChargeThrow(const bool enableChargeThrow) override;
        float GetMinThrowImpulse() const override;
        void SetMinThrowImpulse(const float minThrowImpulse) override;
        float GetMaxThrowImpulse() const override;
        void SetMaxThrowImpulse(const float maxThrowImpulse) override;
        float GetCurrentThrowImpulse() const override;
        float GetChargeTime() const override;
        void SetChargeTime(const float chargeTime) override;
        float GetCurrentChargeTime() const override;
        bool GetEnableChargeWhileRotating() const override;
        void SetEnableChargeWhileRotating(const bool enableChargeWhileRotating) override;
        bool GetIsChargingThrow() const override;
        float GetSphereCastRadius() const override;
        void SetSphereCastRadius(const float sphereCastRadius) override;
        float GetSphereCastDistance() const override;
        void SetSphereCastDistance(const float sphereCastDistance) override;
        bool GetGrabbedObjectKinematicElseDynamic() const override;
        void SetGrabbedObjectKinematicElseDynamic(const bool isKinematic) override;
        bool GetInitialGrabbedObjectIsKinematic() const override;
        float GetCurrentGrabbedObjectAngularDamping() const override;
        void SetCurrentGrabbedObjectAngularDamping(const float currentObjectAngularDamping) override;
        float GetPrevGrabbedObjectAngularDamping() const override;
        void SetPrevGrabbedObjectAngularDamping(const float prevObjectAngularDamping) override;
        float GetTempGrabbedObjectAngularDamping() const override;
        void SetTempGrabbedObjectAngularDamping(const float tempObjectAngularDamping) override;
        float GetCurrentGrabbedObjectLinearDamping() const override;
        void SetCurrentGrabbedObjectLinearDamping(const float currentObjectLinearDamping) override;
        float GetPrevGrabbedObjectLinearDamping() const override;
        void SetPrevGrabbedObjectLinearDamping(const float prevObjectLinearDamping) override;
        float GetTempGrabbedObjectLinearDamping() const override;
        void SetTempGrabbedObjectLinearDamping(const float tempObjectLinearDamping) override;
        AZ::Vector3 GetGrabbedObjectAngularVelocity() const override;
        void SetGrabbedObjectAngularVelocity(const AZ::Vector3& grabbedObjectAngularVelocity) override;
        bool GetInitialAngularVelocityZero() const override;
        void SetInitialAngularVelocityZero(const bool initialAngularVelocityZero) override;
        void ForceTransition(const PhysicsGrabStates& targetState) override;
        void ForceGrab(const AZ::EntityId& objectId);
        void SetStateLocked(const bool isLocked) override;
        bool GetStateLocked() const override;
        bool GetDisableGravityWhileHeld() const override;
        void SetDisableGravityWhileHeld(const bool disableGravityWhileHeld) override;
        bool GetOffsetGrab() const override;
        void SetOffsetGrab(const bool offsetGrab) override;
        bool GetGravityAppliesToPointRotation() const override;
        void SetGravityAppliesToPointRotation(const bool gravityAppliesToPointRotation) override;
        bool GetMassIndependentThrow() const override;
        void SetMassIndependentThrow(const bool massIndependentThrow) override;
        bool GetIsObjectGrabbable() const override;
        bool GetDetectInIdle() const override;
        void SetDetectInIdle(const bool detectInIdle);
        bool GetEnablePIDHeldDynamics() const override;
        void SetEnablePIDHeldDynamics(const bool enablePIDHeldDynamics) override;
        bool GetMassIndependentHeldPID() const override;
        void SetMassIndependentHeldPID(const bool massIndependentHeldPID) override;
        float GetHeldProportionalGain() const override;
        void SetHeldProportionalGain(const float heldProportionalGain) override;
        float GetHeldIntegralGain() const override;
        void SetHeldIntegralGain(const float heldIntegralGain) override;
        float GetHeldDerivativeGain() const override;
        void SetHeldDerivativeGain(const float heldDerivativeGain) override;
        float GetHeldIntegralWindupLimit() const override;
        void SetHeldIntegralWindupLimit(const float heldIntegralWindupLimit) override;
        float GetHeldDerivativeFilterAlpha() const override;
        void SetHeldDerivativeFilterAlpha(const float heldDerivativeFilterAlpha) override;
        bool GetEnablePIDTidalLockDynamics() const override;
        void SetEnablePIDTidalLockDynamics(const bool enablePIDTidalLockDynamics) override;
        bool GetMassIndependentTidalLock() const override;
        void SetMassIndependentTidalLock(const bool massIndependentTidalLock) override;
        bool GetScaleIndependentTidalLock() const override;
        void SetScaleIndependentTidalLock(const bool scaleIndependentTidalLock);
        float GetTidalLockProportionalGain() const override;
        void SetTidalLockProportionalGain(const float tidalLockProportionalGain) override;
        float GetTidalLockIntegralGain() const override;
        void SetTidalLockIntegralGain(const float tidalLockIntegralGain) override;
        float GetTidalLockDerivativeGain() const override;
        void SetTidalLockDerivativeGain(const float tidalLockDerivativeGain) override;
        float GetTidalLockIntegralWindupLimit() const override;
        void SetTidalLockIntegralWindupLimit(const float tidalLockIntegralWindupLimit) override;
        float GetTidalLockDerivativeFilterAlpha() const override;
        void SetTidalLockDerivativeFilterAlpha(const float tidalLockDerivativeFilterAlpha) override;
        PidController<AZ::Vector3>::DerivativeCalculationMode GetHeldDerivativeMode() const override;
        void SetHeldDerivativeMode(const PidController<AZ::Vector3>::DerivativeCalculationMode& heldDerivativeMode) override;
        PidController<AZ::Vector3>::DerivativeCalculationMode GetTidalLockDerivativeMode() const override;
        void SetTidalLockDerivativeMode(const PidController<AZ::Vector3>::DerivativeCalculationMode& tidalLockDerivativeMode) override;
        AZ::Vector3 GetHeldLastProportional() const override;
        AZ::Vector3 GetHeldLastIntegral() const override;
        AZ::Vector3 GetHeldLastDerivative() const override;
        AZ::Vector3 GetTidalLockLastProportional() const override;
        AZ::Vector3 GetTidalLockLastIntegral() const override;
        AZ::Vector3 GetTidalLockLastDerivative() const override;
        AZ::Vector3 GetTargetTranslation() const override;
        AZ::Vector3 GetTargetRotation() const override;
        bool GetDetectMultipleHits() const override;
        void SetDetectMultipleHits(const bool detectMultipleHits);
        bool GetIsAutonomousClient() const override;
        bool GetIsServer() const override;
        bool GetIsHost() const override;
        bool GetLocallyEnableNetworkPhysicsGrabComponent() const override;
        void SetLocallyEnableNetworkPhysicsGrabComponent(const bool networkPhysicsGrabComponentEnabled) override;
        void NetworkPhysicsGrabComponentEnabledIgnoreInputs() override;
        void IsAutonomousSoConnect() override;
        void NotAutonomousSoDisconnect() override;
#ifdef NETWORKPHYSICSGRAB
        AZStd::string GetNetEntityIdStringByEntityId(const AZ::EntityId& entityId) const override;
        Multiplayer::NetEntityId GetNetEntityIdByEntityId(const AZ::EntityId& entityId) const override;
        AZ::EntityId GetEntityIdByNetEntityId(const Multiplayer::NetEntityId& netEntityId) const override;
        AZ::EntityId GetEntityIdByNetEntityIdString(const AZStd::string& netEntityIdString) const override;
        void ForceGrabByNetEntityIdString(const AZStd::string& netEntityIdString) override;
#endif

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

        AZStd::string m_meshTagName = "GrabMesh";

        void CheckForObjects(bool detectionOnly = false);
        void ValidateClientGrabTarget();
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

        // Input event assignment and notification bus connection
        void AssignConnectInputEvents();

        // State machine functions
        void ProcessStates(const float deltaTime, const AZ::u8& tickTimestepNetwork);
        void IdleState();
        void CheckForObjectsState();
        void HoldObjectState(float deltaTime, const AZ::u8& tickTimestepNetwork = 0);
        void RotateObjectState(float deltaTime, const AZ::u8& tickTimestepNetwork = 0);
        void ThrowObjectState(const float deltaTime);

        // NetworkPhysicsGrab object
#ifdef NETWORKPHYSICSGRAB
        NetworkPhysicsGrabComponent* m_networkPhysicsGrabObject = nullptr;
#else
        bool* m_networkPhysicsGrabObject = nullptr;
#endif

        // Networking related variables
        bool m_networkPhysicsGrabComponentEnabled = false;
        bool m_isServer = false;
        bool m_isHost = false;
        bool m_isAutonomousClient = false;
        float m_radiusToleranceMultiplier = 2.f;
        float m_maxNetworkCameraOffset = 4.f;
        AZ::EntityId m_clientGrabTargetEntityId;
        AZ::Vector3 m_networkCameraTranslation = AZ::Vector3::CreateZero();
        AZ::Quaternion m_networkCameraRotation = AZ::Quaternion::CreateIdentity();

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
        AZ::Vector3 m_linearImpulse = AZ::Vector3::CreateZero();
        AZ::Vector3 m_angularImpulse = AZ::Vector3::CreateZero();

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

        // Stores the physics timestep and accumulated physics timestep for interpolation
        float m_physicsTimeAccumulator = 0.f;
        float m_physicsTimestep = 1.f / 60.f;

        float m_minGrabDistance = 1.5f;
        float m_maxGrabDistance = 4.5f;
        float m_maxDropDistance = 4.5f;
        float m_grabDistance = 0.f;
        float m_grabDistanceWheelSensitivity = 0.01f;
        float m_velocityCompDampRate = 20.f;
        float m_angularVelocityDampRate = 40.f;
        float m_kinematicYawRotateScale = 1.2f;
        float m_kinematicPitchRotateScale = 2.133f;
        float m_kinematicRollRotateScale = 1.2f;
        float m_dynamicYawRotateScale = 0.5f;
        float m_dynamicPitchRotateScale = 0.89f;
        float m_dynamicRollRotateScale = 0.5f;
        float m_prevObjectAngularDamping = 0.f;
        float m_currentObjectAngularDamping = 0.f;
        float m_tempObjectAngularDamping = 0.15f;
        float m_tempObjectLinearDamping = 0.05f;
        float m_prevObjectLinearDamping = 0.f;
        float m_currentObjectLinearDamping = 0.f;
        float m_grabDistanceSpeed = 0.3f;
        float m_grabResponse = 15.f;
        float m_throwImpulse = 10.f;
        float m_minThrowImpulse = 3.f;
        float m_maxThrowImpulse = 20.f;
        float m_chargeTime = 3.f;
        float m_currentChargeTime = 0.f;
        float m_currentThrowImpulse = 0.f;
        float m_grabbedObjectMass = 1.f;
        float m_sphereCastRadius = 0.3f;
        float m_sphereCastDistance = 4.5f;
        float m_throwStateMaxTime = 0.5f;
        float m_throwStateCounter = 0.f;
        float m_combinedGrabDistance = 0.f;
        float m_prevGravityEnabled = 0.f;
        float m_heldProportionalGain = 125.f;
        float m_heldIntegralGain = 0.f;
        float m_heldDerivativeGain = 11.f;
        float m_heldIntegralWindupLimit = 100.f;
        float m_heldDerivativeFilterAlpha = 0.8f;
        float m_tidalLockProportionalGain = 115.f;
        float m_tidalLockIntegralGain = 0.f;
        float m_tidalLockDerivativeGain = 9.f;
        float m_tidalLockIntegralWindupLimit = 100.f;
        float m_tidalLockDerivativeFilterAlpha = 0.8f;
        float m_effectiveInertiaFactor = 0.f;
        float m_pitch = 0.f;
        float m_yaw = 0.f;
        float m_roll = 0.f;
        float m_accumPitch = 0.f;
        float m_accumYaw = 0.f;
        float m_accumRoll = 0.f;
        float m_prevGrabKeyValue = 0.f;
        float m_prevRotateKeyValue = 0.f;
        float m_prevThrowKeyValue = 0.f;
        float m_physicsTimestepScaleFactor = 1.f;

        bool m_needsCameraFallback = false;
        bool m_enableChargeThrow = false;
        bool m_isChargingThrow = false;
        bool m_enableChargeWhileRotating = false;
        bool m_hasNotifiedChargeComplete = false;
        bool m_meshSmoothing = false;
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
        bool m_fullTidalLockForFPC = false;
        bool m_isInitialObjectKinematic = false;
        bool m_grabMaintained = true;
        bool m_enablePIDTidalLockDynamics = true;
        bool m_isInGrabState = false;
        bool m_isInRotateState = false;
        bool m_isInThrowState = false;
        bool m_isObjectKinematic = false;
        bool m_objectSphereCastHit = false;
        bool m_stayInIdleState = false;
        bool m_holdKeyToCheckUntilHit = false;
        bool m_initialAngularVelocityZero = true;
        bool m_ignoreGrabDistanceKeyInputValue = true;
        bool m_ignoreYawKeyInputValue = true;
        bool m_ignorePitchKeyInputValue = true;
        bool m_ignoreRollKeyInputValue = true;
        bool m_forceTransition = false;
        bool m_continueToHoldState = false;
        bool m_isStateLocked = false;
        bool m_enablePIDHeldDynamics = true;
        bool m_massIndependentThrow = true;
        bool m_massIndependentHeldPID = true;
        bool m_massIndependentTidalLock = true;
        bool m_scaleIndependentTidalLock = true;
        bool m_offsetGrab = false;
        bool m_gravityAppliesToPointRotation = false;
        bool m_detectInIdle = false;
        bool m_enableMaxDropDistance = true;
        bool m_detectMultipleHits = true;

        PhysicsGrabStates m_state = PhysicsGrabStates::idleState;
        PhysicsGrabStates m_targetState = PhysicsGrabStates::idleState;

        PidController<AZ::Vector3> m_pidController;
        PidController<AZ::Vector3> m_tidalLockPidController;

        PidController<AZ::Vector3>::DerivativeCalculationMode m_heldDerivativeMode = PidController<AZ::Vector3>::Velocity;
        PidController<AZ::Vector3>::DerivativeCalculationMode m_tidalLockDerivativeMode = PidController<AZ::Vector3>::ErrorRate;

        AZStd::map<PhysicsGrabStates, AZStd::string> m_statesMap = { { PhysicsGrabStates::idleState, "idleState" },
                                                                     { PhysicsGrabStates::checkState, "checkState" },
                                                                     { PhysicsGrabStates::holdState, "holdState" },
                                                                     { PhysicsGrabStates::rotateState, "rotateState" },
                                                                     { PhysicsGrabStates::throwState, "throwState" } };

        // Event value multipliers
        float m_grabKeyValue = 0.f;
        float m_rotateKeyValue = 0.f;
        float m_throwKeyValue = 0.f;
        float m_grabDistanceKeyValue = 0.f;
        float m_pitchKeyValue = 0.f;
        float m_yawKeyValue = 0.f;
        float m_rollKeyValue = 0.f;

        // Event IDs and action names
        StartingPointInput::InputEventNotificationId m_grabEventId;
        AZStd::string m_strGrab = "Grab";
        StartingPointInput::InputEventNotificationId m_rotateEventId;
        AZStd::string m_strRotate = "Rotate Enable";
        StartingPointInput::InputEventNotificationId m_throwEventId;
        AZStd::string m_strThrow = "Throw";
        StartingPointInput::InputEventNotificationId m_rotatePitchEventId;
        AZStd::string m_strRotatePitch = "Rotate Pitch";
        StartingPointInput::InputEventNotificationId m_rotateYawEventId;
        AZStd::string m_strRotateYaw = "Rotate Yaw";
        StartingPointInput::InputEventNotificationId m_rotateRollEventId;
        AZStd::string m_strRotateRoll = "Rotate Roll";
        StartingPointInput::InputEventNotificationId m_grabDistanceEventId;
        AZStd::string m_strGrabDistance = "Grab Distance";

        // Array of action names
        AZStd::string* m_inputNames[7] = { &m_strGrab,      &m_strRotate,     &m_strThrow,       &m_strRotatePitch,
                                           &m_strRotateYaw, &m_strRotateRoll, &m_strGrabDistance };

        // Map of event IDs and event value multipliers
        AZStd::map<StartingPointInput::InputEventNotificationId*, float*> m_controlMap = {
            { &m_grabEventId, &m_grabKeyValue },         { &m_rotateEventId, &m_rotateKeyValue },
            { &m_throwEventId, &m_throwKeyValue },       { &m_grabDistanceEventId, &m_grabDistanceKeyValue },
            { &m_rotatePitchEventId, &m_pitchKeyValue }, { &m_rotateYawEventId, &m_yawKeyValue },
            { &m_rotateRollEventId, &m_rollKeyValue }
        };
    };
} // namespace PhysicsGrab
