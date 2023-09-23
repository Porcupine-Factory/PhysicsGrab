#pragma once
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Math/Vector3.h>
//#include <AzFramework/Physics/Common/PhysicsSceneQueries.h>
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzFramework/Physics/CharacterBus.h>
#include <StartingPointInput/InputEventNotificationBus.h>

//#include <PhysXCharacters/API/CharacterController.h>


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
		//void CheckGrounded(/*const float& deltaTime*/);
		void UpdateVelocity();
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
	};
} // namespace TestGem
	
	
	/*// An example of a simple O3DE component
	class MyComponent : public AZ::Component
	{
	public:
		AZ_COMPONENT(MyComponent, "{6C4B7FC6-F4CD-4DBA-99FD-99F7AA2CE8F0}");

		// AZ::Component overrides
		void Activate() override {}
		void Deactivate() override {}

		// Provide runtime reflection, if any
		static void Reflect(AZ::ReflectContext* reflection);
	};
}*/