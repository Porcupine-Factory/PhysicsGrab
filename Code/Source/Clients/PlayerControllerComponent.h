#pragma once
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/Math/Quaternion.h>
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
	const StartingPointInput::InputEventNotificationId
		SprintEventId("sprint");
	const StartingPointInput::InputEventNotificationId
		JumpEventId("jump");

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
		
		void CheckGrounded();
		void UpdateVelocity();
		void setupJumpVariables();
		void ProcessInput(const float& deltaTime);
		void handleGravity();
		void handleJump();

		AZ::Vector3 m_applyGravity = AZ::Vector3::CreateZero();
		AZ::Vector3 m_applyJump = AZ::Vector3::CreateZero();
		AZ::Vector3 m_velocity = AZ::Vector3::CreateZero();
		AZ::Vector3 move = AZ::Vector3::CreateZero();
		AZ::Vector3 moveNormalized = AZ::Vector3::CreateZero();
		AZ::Vector3 m_delta_yaw = AZ::Vector3::CreateZero();
		AZ::Vector3 m_delta_pitch = AZ::Vector3::CreateZero();

		AzPhysics::CollisionGroups::Id m_groundedCollisionGroupId = AzPhysics::CollisionGroups::Id();
		AzPhysics::CollisionGroup m_groundedCollisionGroup = AzPhysics::CollisionGroup::All;

		void UpdateRotation();

		float m_pitch_sensitivity = 0.04f;
		float m_yaw_sensitivity = 0.04f;

		float m_speed = 5.f;
		float m_sprintSpeed = 1.5f;
		float m_sprintPressed = 0.f;
		float m_forward = 0.f;
		float m_backward = 0.f;
		float m_right = 0.f;
		float m_left = 0.f;
		float m_pitch = 0.f;
		float m_yaw = 0.f;
		float m_jump = 0.f;

		float m_currentHeading = 0.f;
		float m_initialDownwardVelocity = 0.f;
		float m_updatedDownwardVelocity = 0.f;
		float m_initialJumpVelocity = 0.f;
		float m_gravity = 0.f;
		float m_fallMultiplier = 0.85f;
		float m_variableJump = 2.2f;
		float m_maxJumpHeight = 5750.f;
		float m_maxJumpTime = 35.f;
		float m_groundCheckRadius = 0.2f;
		bool m_isSprintPressed = false;
		bool m_isJumpPressed = false;
		bool m_isJumpHeld = false;
		bool m_isJumping = false;
		bool m_grounded = true;

	};
} // namespace TestGem