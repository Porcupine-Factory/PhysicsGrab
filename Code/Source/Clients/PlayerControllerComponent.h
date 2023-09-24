#pragma once
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Math/Vector3.h>
#include <AzFramework/Physics/Common/PhysicsSceneQueries.h>
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzFramework/Physics/CharacterBus.h>
#include <StartingPointInput/InputEventNotificationBus.h>


namespace TestGem
{
	// Define the action we want. In this case, notify when forward key is pressed
	const StartingPointInput::InputEventNotificationId
		MoveFwdEventId("move forward");
	const StartingPointInput::InputEventNotificationId
		MoveBackEventId("move backward");
	const StartingPointInput::InputEventNotificationId
		MoveRightEventId("move right");
	const StartingPointInput::InputEventNotificationId
		MoveLeftEventId("move left");
	const StartingPointInput::InputEventNotificationId
		RotatePitchEventId("rotate pitch");
	const StartingPointInput::InputEventNotificationId
		RotateYawEventId("rotate yaw");
	

	// Inherit from StartingPointInput::InputEventNotificationBus::MultiHandler
	class PlayerControllerComponent
		: public AZ::Component
		, public AZ::TickBus::Handler
		, public StartingPointInput::InputEventNotificationBus::MultiHandler
	{

	public:
		AZ_COMPONENT(PlayerControllerComponent,
			"{3C83168C-1BED-4D48-924D-6848DBA59B18}");

		static void Reflect(AZ::ReflectContext* rc);

		// AZ::Component interface implementation
		void Activate() override;
		void Deactivate() override;

		// AZ::InputEventNotificationBus interface. Overrides OnPressed and OnReleased virtual methods. 
		void OnPressed(float value) override;
		void OnReleased(float value) override;
		void OnHeld(float value) override;

		// TickBus interface
		void OnTick(float deltaTime, AZ::ScriptTimePoint) override;

	private:
		AZ::Entity* m_activeCameraEntity = nullptr;
		AZ::Entity* GetActiveCamera();
		
		void ProcessInput();
		void CheckGrounded();
		void UpdateVelocity();
		void HandleGravity(/*const float& deltaTime*/);
		AZ::Vector3 m_velocity = AZ::Vector3::CreateZero();

		void UpdateRotation();
		//AZ::Vector3 m_yaw_delta = AZ::Vector3::CreateZero();

		float m_pitch_sensitivity = 0.04f;
		float m_yaw_sensitivity = 0.04f;

		float m_speed = 5.f;
		float m_forward = 0.f;
		float m_backward = 0.f;
		float m_right = 0.f;
		float m_left = 0.f;
		float m_pitch = 0.f;
		float m_yaw = 0.f;

		float m_initialDownwardVelocity = 0.f;
		float m_updatedDownwardVelocity = 0.f;
		float m_gravity = -9.8f;
		bool m_grounded = true;

		AZ::Vector3 m_applyGravity = AZ::Vector3::CreateZero();
	};
} // namespace TestGem