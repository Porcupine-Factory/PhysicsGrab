#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityBus.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Math/Quaternion.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/std/containers/map.h>
#include <AzFramework/Components/CameraBus.h>
#include <AzFramework/Physics/Common/PhysicsSceneQueries.h>
#include <StartingPointInput/InputEventNotificationBus.h>
#include <ObjectInteraction/ObjectInteractionComponentBus.h>

#if __has_include(<FirstPersonController/FirstPersonControllerComponentBus.h>)
#include <FirstPersonController/FirstPersonControllerComponentBus.h>
#endif

namespace ObjectInteraction
{
    enum class ObjectInteractionStates
    {
        idleState,
        checkState,
        holdState,
        rotateState,
        throwState
    };

    class ObjectInteractionComponent
        : public AZ::Component
        , public AZ::EntityBus::Handler
        , public AZ::TickBus::Handler
        , public StartingPointInput::InputEventNotificationBus::MultiHandler
        , public ObjectInteractionComponentRequestBus::Handler
        , Camera::CameraNotificationBus::Handler
    {
    public:
        AZ_COMPONENT(ObjectInteractionComponent, "{E4630B86-1755-4F7F-88C6-AE11704D7F00}");

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

        // AZ::InputEventNotificationBus interface. Overrides OnPressed and OnReleased virtual methods.
        void OnPressed(float value) override;
        void OnReleased(float value) override;
        void OnHeld(float value) override;

        void OnTick(float deltaTime, AZ::ScriptTimePoint) override;

        AZ::Entity* GetEntityPtr(AZ::EntityId pointer) const;
        AZ::Entity* GetActiveCameraEntityPtr() const;

        // ObjectInteractionRequestBus
        AZ::EntityId GetGrabbingEntityId() const override;
        AZ::EntityId GetActiveCameraEntityId() const override;
        AZ::EntityId GetGrabbedObjectEntityId() const override;
        AZ::EntityId GetLastGrabbedObjectEntityId() const override;
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
        AZStd::string GetMeshEntityName() const override;
        void SetMeshEntityName(const AZStd::string& new_meshEntityName) override;
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
        float GetInitialGrabbedObjectDistance() const override;
        void SetInitialGrabbedObjectDistance(const float& new_initialGrabDistance) override;
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
        float GetDynamicYawRotateScale() const;
        void SetDynamicYawRotateScale(const float& new_dynamicHorizontalYawcale);
        float GetDynamicPitchRotateScale() const;
        void SetDynamicPitchRotateScale(const float& new_dynamicPitchRotateScale);
        float GetKinematicYawRotateScale() const;
        void SetKinematicYawRotateScale(const float& new_kinematicYawRotateScale);
        float GetKinematicPitchRotateScale() const;
        void SetKinematicPitchRotateScale(const float& new_kinematicPitchRotateScale);
        float GetThrowImpulse() const override;
        void SetThrowImpulse(const float& new_throwImpulse) override;
        float GetGrabbedObjectThrowStateCounter() const override;
        void SetGrabbedObjectThrowStateCounter(const float& new_throwStateCounter) override;
        float GetGrabbedObjectThrowStateTime() const override;
        void SetGrabbedObjectThrowStateTime(const float& new_throwStateMaxTime) override;
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
        AZ::Vector3 GetGrabbedObjectAngularVelocity() const override;
        void SetGrabbedObjectAngularVelocity(const AZ::Vector3& new_grabbedObjectAngularVelocity) override;
        bool GetInitialAngularVelocityZero() const override;
        void SetInitialAngularVelocityZero(const bool& new_initialAngularVelocityZero) override;
        void ForceTransition(const ObjectInteractionStates& targetState) override;
        void SetStateLocked(const bool& isLocked) override;
        bool GetStateLocked() const override;
     
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

        AZStd::string m_meshEntityName = "Grab Mesh";

        void OnCameraAdded(const AZ::EntityId& cameraId);
        void CheckForObjects();
        void HoldObject(float deltaTime);
        void RotateObject(float deltaTime);
        void ThrowObject();
        void TidalLock(float deltaTime);
        void UpdateGrabDistance(float deltaTime);
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

        // ObjectInteractionNotificationBus
        void OnObjectSphereCastHit();
        void OnHoldStart();
        void OnHoldStop();
        void OnRotateStart();
        void OnRotateStop();
        void OnThrowStart();
        void OnThrowStop();
        void OnMaxThrowDistance();
        void OnThrowStateCounterZero();
        
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

        AZ::Vector3 m_forwardVector = AZ::Vector3::CreateZero();
        AZ::Vector3 m_rightVector = AZ::Vector3::CreateZero();
        AZ::Vector3 m_upVector = AZ::Vector3::CreateZero();
        AZ::Vector3 m_grabbedObjectTranslation = AZ::Vector3::CreateZero();
        AZ::Vector3 m_grabbedObjectAngularVelocity = AZ::Vector3::CreateZero();
        AZ::Vector3 m_lastEntityRotation = AZ::Vector3::CreateZero();
        AZ::Vector3 m_grabbingEntityVelocity = AZ::Vector3::CreateZero();
        AZ::Vector3 m_currentGrabEntityTranslation = AZ::Vector3::CreateZero();
        AZ::Vector3 m_prevGrabbingEntityTranslation = AZ::Vector3::CreateZero();
        AZ::Vector3 m_currentCompensationVelocity = AZ::Vector3::CreateZero();
        AZ::Vector3 m_currentAngularVelocity = AZ::Vector3::CreateZero();

        AZ::EntityId m_grabbedObjectEntityId;
        AZ::EntityId m_meshEntityId;
        AZ::EntityId m_grabbingEntityId;
        AZ::EntityId m_lastGrabbedObjectEntityId;
        AZ::EntityId m_thrownGrabbedObjectEntityId;

        AzPhysics::CollisionGroups::Id m_grabbedCollisionGroupId = AzPhysics::CollisionGroups::Id();
        AzPhysics::CollisionGroup m_grabbedCollisionGroup = AzPhysics::CollisionGroup::All;

        AzPhysics::CollisionLayer m_prevGrabbedCollisionLayer;
        AzPhysics::CollisionLayer m_currentGrabbedCollisionLayer;
        AzPhysics::CollisionLayer m_tempGrabbedCollisionLayer;
        AZStd::string m_currentGrabbedCollisionLayerName;
        AZStd::string m_tempGrabbedCollisionLayerName;

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

        float m_physicsTimeAccumulator = 0.f;
        float m_physicsTimestep = 1.f / 60.f;
        float m_minGrabDistance = 1.5f;
        float m_maxGrabDistance = 3.5f;
        float m_initialGrabDistance = 2.f;
        float m_grabDistance = m_initialGrabDistance;
        float m_velocityDampRate = 8.f;
        float m_angularDampRate = 16.f;
        float m_kinematicYawRotateScale = 0.8f;
        float m_kinematicPitchRotateScale = 1.422f;
        float m_kinematicRollRotateScale = 0.8f;
        float m_dynamicYawRotateScale = 0.8f;
        float m_dynamicPitchRotateScale = 1.422f;
        float m_dynamicRollRotateScale = 0.8f;
        float m_prevObjectAngularDamping = 0.f;
        float m_currentObjectAngularDamping = 0.f;
        float m_tempObjectAngularDamping = 20.f;
        float m_grabDistanceSpeed = 0.2f;
        float m_grabResponse = 10.f;
        float m_throwImpulse = 7000.f;
        float m_sphereCastRadius = 0.3f;
        float m_sphereCastDistance = 3.f;
        float m_throwStateMaxTime = 0.5f;
        float m_throwStateCounter = 0.f;
        float m_combinedGrabDistance = 0.f;
        float m_pitch = 0.f;
        float m_yaw = 0.f;
        float m_roll = 0.f;

        bool m_enableMeshSmoothing = true;
        bool m_enableVelocityCompensation = true;
        bool m_useFPControllerForGrab = true;
        bool m_grabEnableToggle = false;
        bool m_kinematicWhileHeld = false;
        bool m_freezeCharacterRotation = true;
        bool m_rotateEnableToggle = true;
        bool m_tidalLock = true;
        bool m_dynamicTidalLock = true;
        bool m_kinematicTidalLock = true;
        bool m_isInitialObjectKinematic = false;
        bool m_grabMaintained = false;
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

        ObjectInteractionStates m_state = ObjectInteractionStates::idleState;
        ObjectInteractionStates m_targetState = ObjectInteractionStates::idleState;

        AZStd::map<ObjectInteractionStates, AZStd::string> m_statesMap = {
          {ObjectInteractionStates::idleState,   "idleState"},
          {ObjectInteractionStates::checkState,  "checkState"},
          {ObjectInteractionStates::holdState,   "holdState"},
          {ObjectInteractionStates::rotateState, "rotateState"},
          {ObjectInteractionStates::throwState,  "throwState"}
        };
    };
} // namespace ObjectInteraction
