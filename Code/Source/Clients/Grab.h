#pragma once
#include <AzCore/Component/Component.h>
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
		, public AZ::TickBus::Handler
		, public StartingPointInput::InputEventNotificationBus::MultiHandler
		, Camera::CameraNotificationBus::Handler
	{
	public:
		AZ_COMPONENT(Grab, "{D5628156-EF36-41E9-9C15-4BD66B7B834E}");

		// Provide runtime reflection, if any
		static void Reflect(AZ::ReflectContext* rc);

		// AZ::Component overrides
		void Activate() override;
		void Deactivate() override;

		// AZ::InputEventNotificationBus interface. Overrides OnPressed and OnReleased virtual methods. 
		void OnPressed(float value) override;
		void OnReleased(float value) override;
		void OnHeld(float value) override;

		void OnTick(float deltaTime, AZ::ScriptTimePoint) override;

		AZ::Entity* GetEntityPtr(AZ::EntityId pointer) const;

	private:
		AZ::Entity* m_cameraEntity = nullptr;

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
		void HoldObject(AZ::EntityId objectId, const float& deltaTime);
		void ThrowObject(AZ::EntityId objectId, const float& deltaTime);
		void RotateObject(AZ::EntityId objectId, const float& deltaTime);

		AZ::Transform m_cameraTransform = AZ::Transform::CreateIdentity();
		AZ::Transform m_grabReference = AZ::Transform::CreateIdentity();

		AZ::Vector3 m_forwardVector = AZ::Vector3::CreateZero();
		AZ::Vector3 m_grabbedObject = AZ::Vector3::CreateZero();
		AZ::Vector3 m_delta_pitch = AZ::Vector3::CreateZero();
		AZ::Vector3 m_delta_yaw = AZ::Vector3::CreateZero();

		AZStd::vector<AZ::EntityId> m_grabEntityIds;

		AZ::EntityId m_lastGrabbedObject;

		AzPhysics::CollisionGroups::Id m_grabCollisionGroupId = AzPhysics::CollisionGroups::Id();
		AzPhysics::CollisionGroup m_grabCollisionGroup = AzPhysics::CollisionGroup::All;

		float m_grabKey = 0.f;
		float m_throwKey = 0.f;
		float m_rotateKey = 0.f;
		float m_grabDistanceKey = 0.f;
		float m_grabDistance = 0.f;
		float m_pitch = 0.f;
		float m_yaw = 0.f;

		bool isThrowing = false;
		bool m_objectReset = false;
		bool isObjectKinematic = false;
		
		const float m_rotateScale = 0.5f;
		const float m_minGrabDistance = 1.5f;
		const float m_maxGrabDistance = 3.f;
		const float m_grabInitialDistance = 1.75f;
		const float m_grabDistanceSpeed = 0.2f;
		const float m_grabStrength = 700.f;
		const float m_throwStrength = 60000.f;
		const float m_sphereCastRadius = 0.3f;
		const float m_sphereCastDistance = 3.f;
	};
}