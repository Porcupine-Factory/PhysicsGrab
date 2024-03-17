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
        AZStd::string GetStateString() const override;
        bool GetIsInIdleState() const override;
        bool GetIsInCheckState() const override;
        bool GetIsInHeldState() const override;
        bool GetIsInRotateState() const override;
        bool GetIsInThrowState() const override;
        bool GetObjectSphereCastHit() const override;
        bool GetStayInIdleState() const override;
        void SetStayInIdleState(const bool& new_stayInIdleState) override;
        float GetPitchKeyValue() const override;
        void SetPitchKeyValue(const float& new_pitchKeyValue, const bool& new_ignorePitchKeyInputValue) override;
        float GetYawKeyValue() const override;
        void SetYawKeyValue(const float& new_yawKeyValue, const bool& new_ignoreYawKeyInputValue) override;
        float GetRollKeyValue() const override;
        void SetRollKeyValue(const float& new_rollKeyValue, const bool& new_ignoreRollKeyInputValue) override;
        float GetGrabbedDistanceKeyValue() const override;
        void SetGrabbedDistanceKeyValue(const float& new_grabDistanceKeyValue) override;
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
        float GetDynamicRotateScale() const override;
        void SetDynamicRotateScale(const float& new_dynamicRotateScale) override;
        float GetKinematicRotateScale() const override;
        void SetKinematicRotateScale(const float& new_kinematicRotateScale) override;
        float GetThrowImpulse() const override;
        void SetThrowImpulse(const float& new_throwImpulse) override;
        float GetGrabbedObjectThrowStateCounter() const override;
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

    private:
        AZ::Entity* m_grabbingEntityPtr = nullptr;

        StartingPointInput::InputEventNotificationId m_grabEventId;
        AZStd::string m_strGrab = "Grab Key";

        StartingPointInput::InputEventNotificationId m_throwEventId;
        AZStd::string m_strThrow = "Throw Key";

        StartingPointInput::InputEventNotificationId m_rotateEventId;
        AZStd::string m_strRotate = "Rotate Enable Key";

        StartingPointInput::InputEventNotificationId m_rotatePitchEventId;
        AZStd::string m_strRotatePitch = "Rotate Pitch Key";

        StartingPointInput::InputEventNotificationId m_rotateYawEventId;
        AZStd::string m_strRotateYaw = "Rotate Yaw Key";

        StartingPointInput::InputEventNotificationId m_rotateRollEventId;
        AZStd::string m_strRotateRoll = "Rotate Roll Key";

        StartingPointInput::InputEventNotificationId m_grabDistanceEventId;
        AZStd::string m_strGrabDistance = "Grab Distance Key";

        void OnCameraAdded(const AZ::EntityId& cameraId);
        void CheckForObjects();
        void HoldObject();
        void RotateObject();
        void ThrowObject();
        void TidalLock();
        #ifdef FIRST_PERSON_CONTROLLER
        void FreezeCharacterRotation();
        #endif

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
        void ProcessStates(const float& deltaTime);
        void IdleState();
        void CheckForObjectsState();
        void HoldObjectState();
        void RotateObjectState();
        void ThrowObjectState(const float &deltaTime);

        AZ::Transform m_grabbingEntityTransform = AZ::Transform::CreateIdentity();
        AZ::Transform m_grabReference = AZ::Transform::CreateIdentity();

        AZ::Vector3 m_forwardVector = AZ::Vector3::CreateZero();
        AZ::Vector3 m_rightVector = AZ::Vector3::CreateZero();
        AZ::Vector3 m_upVector = AZ::Vector3::CreateZero();
        AZ::Vector3 m_grabbedObjectTranslation = AZ::Vector3::CreateZero();
        AZ::Vector3 m_grabbedObjectAngularVelocity = AZ::Vector3::CreateZero();
        AZ::Vector3 m_lastEntityRotation = AZ::Vector3::CreateZero();

        AZ::EntityId m_grabbedObjectEntityId;

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
        float m_prevRotateKeyValue = 0.f;
        float m_grabDistanceKeyValue = 0.f;
        float m_pitchKeyValue = 0.f;
        float m_yawKeyValue = 0.f;
        float m_rollKeyValue = 0.f;

        float m_prevGrabKeyValue = 0.f;
        float m_minGrabDistance = 1.5f;
        float m_maxGrabDistance = 3.f;
        float m_initialGrabDistance = 1.75f;
        float m_grabDistance = m_initialGrabDistance;
        float m_kinematicRotateScale = 0.5f;
        float m_dynamicRotateScale = 0.3f;
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
        float m_pitch = 0.f;
        float m_yaw = 0.f;
        float m_roll = 0.f;

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
        bool m_initialAngularVelocityZero = false;
        bool m_ignoreYawKeyInputValue = true;
        bool m_ignorePitchKeyInputValue = true;
        bool m_ignoreRollKeyInputValue = true;

        ObjectInteractionStates m_state = ObjectInteractionStates::idleState;

        AZStd::map<ObjectInteractionStates, AZStd::string> m_statesMap = {
          {ObjectInteractionStates::idleState,   "idleState"},
          {ObjectInteractionStates::checkState,  "checkState"},
          {ObjectInteractionStates::holdState,   "holdState"},
          {ObjectInteractionStates::rotateState, "rotateState"},
          {ObjectInteractionStates::throwState,  "throwState"}
        };
    };
} // namespace ObjectInteraction
