#pragma once

#include <TestGem/TestGemComponentBus.h>
#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityBus.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/Math/Quaternion.h>
#include <AzFramework/Physics/Common/PhysicsSceneQueries.h>
#include <AzFramework/Components/CameraBus.h>
#include <StartingPointInput/InputEventNotificationBus.h>

namespace TestGem
{

	class Grab
		: public AZ::Component
		, public AZ::EntityBus::Handler
		, public AZ::TickBus::Handler
		, public StartingPointInput::InputEventNotificationBus::MultiHandler
		, public TestGemComponentRequestBus::Handler
		, Camera::CameraNotificationBus::Handler
	{
	public:
		AZ_COMPONENT(Grab, "{D5628156-EF36-41E9-9C15-4BD66B7B834E}");

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

		// TestGemRequestBus
		AZ::EntityId GetGrabbingEntityId() const override;
		AZ::EntityId GetActiveCameraEntityId() const override;
		AZ::EntityId GetGrabbedObjectEntityId() const override;
		AZ::EntityId GetLastGrabbedObjectEntityId() const override;


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
		bool GetisGrabbing() const override;
		bool GetisThrowing() const override;
		bool GetisRotating() const override;
		float GetGrabObjectDistance() const override;
		void SetGrabObjectDistance(const float& new_grabDistance) override;
		float GetMinGrabObjectDistance() const override;
		void SetMinGrabObjectDistance(const float& new_minGrabDistance) override;
		float GetMaxGrabObjectDistance() const override;
		void SetMaxGrabObjectDistance(const float& new_maxGrabDistance) override;
		float GetInitialGrabObjectDistance() const override;
		void SetInitialGrabObjectDistance(const float& new_initialGrabDistance) override;
		float GetGrabObjectDistanceSpeed() const override;
		void SetGrabObjectDistanceSpeed(const float& new_grabDistanceSpeed) override;
		float GetGrabStrength() const override;
		void SetGrabStrength(const float& new_grabStrength) override;
		float GetRotateScale() const override;
		void SetRotateScale(const float& new_rotateScale) override;
		float GetThrowStrength() const override;
		void SetThrowStrength(const float& new_throwStrength) override;
		float GetSphereCastRadius() const override;
		void SetSphereCastRadius(const float& new_sphereCastRadius) override;
		float GetSphereCastDistance() const override;
		void SetSphereCastDistance(const float& new_sphereCastDistance) override;

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

		StartingPointInput::InputEventNotificationId m_grabDistanceEventId;
		AZStd::string m_strGrabDistance = "Grab Distance Key";

		void OnCameraAdded(const AZ::EntityId& cameraId);
		void CheckForObjects(const float& deltaTime);
		void GrabObject(AZ::EntityId objectId, const float& deltaTime);
		void ThrowObject(AZ::EntityId objectId);
		void RotateObject(AZ::EntityId objectId, const float& deltaTime);

		AZ::Transform m_grabbingEntityTransform = AZ::Transform::CreateIdentity();
		AZ::Transform m_grabReference = AZ::Transform::CreateIdentity();

		AZ::Vector3 m_forwardVector = AZ::Vector3::CreateZero();
		AZ::Vector3 m_grabbedObjectTranslation = AZ::Vector3::CreateZero();
		AZ::Vector3 m_delta_pitch = AZ::Vector3::CreateZero();
		AZ::Vector3 m_delta_yaw = AZ::Vector3::CreateZero();

		AZ::EntityId m_grabbedObjectEntityId;

		AZ::EntityId m_grabbingEntityId;
		AZ::EntityId m_lastGrabbedObjectEntityId;

		AzPhysics::CollisionGroups::Id m_grabbedCollisionGroupId = AzPhysics::CollisionGroups::Id();
		AzPhysics::CollisionGroup m_grabbedCollisionGroup = AzPhysics::CollisionGroup::All;

		AzPhysics::CollisionLayer m_prevGrabbedCollisionLayer;
		AzPhysics::CollisionLayer m_currentGrabbedCollisionLayer;
		AZStd::string m_currentGrabbedCollisionLayerName;
		AzPhysics::CollisionLayer m_tempGrabbedCollisionLayer;
		AZStd::string m_tempGrabbedCollisionLayerName;

		// Event value multipliers
		float m_grabKeyValue = 0.f;
		float m_throwKeyValue = 0.f;
		float m_rotateKeyValue = 0.f;
		float m_rotatePrevValue = 0.f;
		float m_grabDistanceKeyValue = 0.f;
		float m_grabDistance = 0.f;
		float m_pitchKeyValue = 0.f;
		float m_yawKeyValue = 0.f;

		float m_grabPrevValue = 0.f;
		float m_minGrabDistance = 1.5f;
		float m_maxGrabDistance = 3.f;
		float m_initialGrabDistance = 1.75f;
		float m_rotateScale = 0.5f;
		float m_grabDistanceSpeed = 0.2f;
		float m_grabStrength = 10.f;
		float m_throwStrength = 5000.f;
		float m_sphereCastRadius = 0.3f;
		float m_sphereCastDistance = 3.f;

		bool m_grabEnableToggle = false;
		bool m_kinematicDefaultEnable = false;
		bool m_rotateEnableToggle = true;
		bool m_isGrabbing = false;
		bool m_grabMaintained = false;
		bool m_isRotating = false;
		bool m_isThrowing = false;
		bool m_hasRotated = false;
		bool m_isObjectKinematic = false;
	};
}
