#include <Clients/PlayerControllerComponent.h>

#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzFramework/Components/CameraBus.h>
#include <AzFramework/Physics/CollisionBus.h>
#include <AzFramework/Physics/SystemBus.h>
#include <System/PhysXSystem.h>

namespace TestGem
{
	using namespace StartingPointInput;

	void PlayerControllerComponent::Reflect(AZ::ReflectContext* rc)
	{
		if (auto sc = azrtti_cast<AZ::SerializeContext*>(rc))
		{
			sc->Class<PlayerControllerComponent, AZ::Component>()
				->Field("Speed", &PlayerControllerComponent::m_speed)
				->Field("Sprint Multiplier", &PlayerControllerComponent::m_sprintSpeed)
				->Field("Pitch Sensitivity", &PlayerControllerComponent::m_pitch_sensitivity)
				->Field("Yaw Sensitivity", &PlayerControllerComponent::m_yaw_sensitivity)
				->Field("Max Jump Height", &PlayerControllerComponent::m_maxJumpHeight)
				->Field("Max Jump Time", &PlayerControllerComponent::m_maxJumpTime)
				->Field("Fall Multiplier", &PlayerControllerComponent::m_fallMultiplier)
				->Field("Short Jump Multiplier", &PlayerControllerComponent::m_variableJump)
				->Field("Ground Check Radius", &PlayerControllerComponent::m_groundCheckRadius)
				->Field("Grounded Collision Group", &PlayerControllerComponent::m_groundedCollisionGroupId)
				->Version(1);

			if (AZ::EditContext* ec = sc->GetEditContext())
			{
				using namespace AZ::Edit;
				ec->Class<PlayerControllerComponent>(
					"Player Controller",
					"[Player controlled character]")
					->ClassElement(ClassElements::EditorData, "")
					->Attribute(
						Attributes::AppearsInAddComponentMenu,
						AZ_CRC("Game"))
					->DataElement(nullptr,
						&PlayerControllerComponent::m_pitch_sensitivity, 
						"Camera Pitch Rotate Input", "Camera pitch rotation control")
					->DataElement(nullptr,
						&PlayerControllerComponent::m_yaw_sensitivity,
						"Camera Yaw Rotate Input", "Camera yaw rotation control")
					->DataElement(nullptr, 
						&PlayerControllerComponent::m_speed, 
						"Speed", "Player's Speed")
					->DataElement(nullptr,
						&PlayerControllerComponent::m_sprintSpeed,
						"Sprint Multiplier", "Sprint speed scale factor")
					->DataElement(nullptr,
						&PlayerControllerComponent::m_maxJumpHeight,
						"Max Jump Height", "Maximum character jump height")
					->DataElement(nullptr,
						&PlayerControllerComponent::m_maxJumpTime,
						"Max Jump Time", "Time to max jump height")
					->DataElement(nullptr,
						&PlayerControllerComponent::m_fallMultiplier,
						"Fall Multiplier", "Scale for how fast the character falls")
					->DataElement(nullptr,
						&PlayerControllerComponent::m_variableJump,
						"Short Jump Multiplier", "Scales gravity to create shorter jump when jump key is released")
					->DataElement(nullptr,
						&PlayerControllerComponent::m_groundCheckRadius,
						"Ground Check Radius", "Sphere Cast radius used for ground check")
					->DataElement(nullptr,
						&PlayerControllerComponent::m_groundedCollisionGroupId,
						"Grounded Collision Group", "The collision group which will be used for the ground detection.");
			}
		}
	}

	void PlayerControllerComponent::Activate()
	{
		InputEventNotificationBus::MultiHandler::BusConnect(MoveFwdEventId);
		AZ::TickBus::Handler::BusConnect();
		InputEventNotificationBus::MultiHandler::BusConnect(MoveBackEventId);
		AZ::TickBus::Handler::BusConnect();
		InputEventNotificationBus::MultiHandler::BusConnect(MoveLeftEventId);
		AZ::TickBus::Handler::BusConnect();
		InputEventNotificationBus::MultiHandler::BusConnect(MoveRightEventId);
		AZ::TickBus::Handler::BusConnect();
		InputEventNotificationBus::MultiHandler::BusConnect(RotatePitchEventId);
		AZ::TickBus::Handler::BusConnect();
		InputEventNotificationBus::MultiHandler::BusConnect(RotateYawEventId);
		AZ::TickBus::Handler::BusConnect();
		InputEventNotificationBus::MultiHandler::BusConnect(SprintEventId);
		AZ::TickBus::Handler::BusConnect();
		InputEventNotificationBus::MultiHandler::BusConnect(JumpEventId);
		AZ::TickBus::Handler::BusConnect();

		Physics::CollisionRequestBus::BroadcastResult(
			m_groundedCollisionGroup, &Physics::CollisionRequests::GetCollisionGroupById, m_groundedCollisionGroupId);

		setupJumpVariables();
	}

	void PlayerControllerComponent::Deactivate()
	{
		AZ::TickBus::Handler::BusDisconnect();
		InputEventNotificationBus::MultiHandler::BusDisconnect();
	}
	// Recieve the input event in OnPressed method
	void PlayerControllerComponent::OnPressed(float value)
	{
		const InputEventNotificationId* inputId = InputEventNotificationBus::GetCurrentBusId();
		if (inputId == nullptr)
		{
			return;
		}

		if (*inputId == MoveFwdEventId)
		{
			m_forward = value;
			//AZ_Printf("Player", "forward pressed value %f", value);
		}

		if (*inputId == MoveBackEventId)
		{
			m_backward = value;
			//AZ_Printf("Player", "backward pressed value %f", value);
		}

		if (*inputId == MoveRightEventId)
		{
			m_right = value;
			//AZ_Printf("Player", "right pressed value %f", value);
		}

		if (*inputId == MoveLeftEventId)
		{
			m_left = value;
			//AZ_Printf("Player", "left pressed value %f", value);
		}

		if (*inputId == RotatePitchEventId)
		{
			m_pitch = value;
			//AZ_Printf("Player", "pitch value %f", value);
		}

		if (*inputId == RotateYawEventId)
		{
			m_yaw = value;
			//AZ_Printf("Player", "yaw value %f", value);
		}

		if (*inputId == SprintEventId)
		{
			m_sprintPressed = value;
			m_isSprintPressed = true;
			//AZ_Printf("Player", "sprint pressed value %f", value);
		}

		if (*inputId == JumpEventId)
		{
			m_jump = value;
			m_isJumpPressed = true;
			//AZ_Printf("Player", "jump pressed value %f", value);
		}
	}
	// Recieve the input event in OnReleased method
	void PlayerControllerComponent::OnReleased(float value)
	{
		const InputEventNotificationId* inputId = InputEventNotificationBus::GetCurrentBusId();
		if (inputId == nullptr)
		{
			return;
		}
		if (*inputId == MoveFwdEventId)
		{
			m_forward = value;
			//AZ_Printf("Player", "forward released value %f", value);
		}
		if (*inputId == MoveBackEventId)
		{
			m_backward = value;
			//AZ_Printf("Player", "back released value %f", value);
		}
		if (*inputId == MoveRightEventId)
		{
			m_right = value;
			//AZ_Printf("Player", "right released value %f", value);
		}
		if (*inputId == MoveLeftEventId)
		{
			m_left = value;
			//AZ_Printf("Player", "left released value %f", value);
		}

		if (*inputId == RotatePitchEventId)
		{
			m_pitch = value;
			//AZ_Printf("Player", "pitch released value %f", value);
		}

		if (*inputId == RotateYawEventId)
		{
			m_yaw = value;
			//AZ_Printf("Player", "yaw released value %f", value);
		}

		if (*inputId == SprintEventId)
		{
			m_sprintPressed = value;
			m_isSprintPressed = false;
			//AZ_Printf("Player", "sprint released value %f", value);
		}

		if (*inputId == JumpEventId)
		{
			m_jump = value;
			m_isJumpPressed = false;
			m_isJumpHeld = false;
			//AZ_Printf("Player", "jump released value %f", value);
		}
	}

	void PlayerControllerComponent::OnHeld(float value)
	{
		const InputEventNotificationId* inputId = InputEventNotificationBus::GetCurrentBusId();
		if (inputId == nullptr)
		{
			return;
		}

		if (*inputId == RotatePitchEventId)
		{
			m_pitch = value;
		}
		
		else if (*inputId == RotateYawEventId)
		{
			m_yaw = value;
		}

		else if (*inputId == JumpEventId)
		{
			m_isJumpPressed = false;
			m_isJumpHeld = true;
			//AZ_Printf("Player", "jump released value %f", value);
		}
	}

	void PlayerControllerComponent::OnTick(float deltaTime, AZ::ScriptTimePoint)
	{
		ProcessInput(deltaTime);
	}

	AZ::Entity* PlayerControllerComponent::GetActiveCamera()
	{
		AZ::EntityId activeCameraId;
		Camera::CameraSystemRequestBus::BroadcastResult(activeCameraId,
			&Camera::CameraSystemRequestBus::Events::GetActiveCamera);

		auto ca = AZ::Interface<AZ::ComponentApplicationRequests>::Get();
		return ca->FindEntity(activeCameraId);
	}

	void PlayerControllerComponent::setupJumpVariables()
	{
		float timeToApex = (m_maxJumpTime / 2);
		m_gravity = (-2 * m_maxJumpHeight) / (timeToApex * timeToApex);
		m_initialJumpVelocity = (2 * m_maxJumpHeight) / timeToApex;
	}

	void PlayerControllerComponent::CheckGrounded()
	{	
		// Get our entity's local transform and offset it along Z axis by m_groundCheckRadius distance
		AZ::Transform currentTransform = AZ::Transform::CreateIdentity();
		currentTransform.SetTranslation(GetEntity()->GetTransform()->GetLocalTM().GetTranslation() + AZ::Vector3::CreateAxisZ(m_groundCheckRadius));

		// Perform a spherecast query to check if entity is grounded
		auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();

        AzPhysics::ShapeCastRequest request = AzPhysics::ShapeCastRequestHelpers::CreateSphereCastRequest(m_groundCheckRadius,
			currentTransform,
            AZ::Vector3(0.0f, 0.0f, -1.0f),
            0.0001f,
            AzPhysics::SceneQuery::QueryType::StaticAndDynamic,
            m_groundedCollisionGroup,
            nullptr);

        AzPhysics::SceneHandle sceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        AzPhysics::SceneQueryHits hits = sceneInterface->QueryScene(sceneHandle, &request);
		
		m_grounded = hits ? true : false;
	}
	
	void PlayerControllerComponent::handleGravity()
	{
		AZ::Vector3 zVelocity = AZ::Vector3::CreateZero();
		Physics::CharacterRequestBus::EventResult(zVelocity, GetEntityId(),
			&Physics::CharacterRequestBus::Events::GetVelocity);
		
		// Prints the Entity's velocity on Z
		//AZ_Printf("", "zVelocity.GetZ() = %.10f", zVelocity.GetZ());
	
		bool isFalling = zVelocity.GetZ() <= 0.f;
		//AZ_Printf("", "%s", isFalling ? "Falling" : "NOT Falling");

		// Applies gravity via AddVelocityForTick in Z direction
		m_initialDownwardVelocity = m_gravity;
		m_applyGravity = AZ::Vector3(0.f, 0.f, m_updatedDownwardVelocity);

		if (m_grounded)
		{
			m_updatedDownwardVelocity = 0.f;
		}
		else if (isFalling && !m_grounded && (m_isJumpPressed || m_isJumpHeld))
		{
			m_updatedDownwardVelocity += (m_initialDownwardVelocity * m_fallMultiplier);
		}
		else if (!m_isJumpHeld && !m_isJumpPressed && !m_grounded)
		{
			m_updatedDownwardVelocity += (m_initialDownwardVelocity * m_fallMultiplier * m_variableJump);
		}
		else
		{
			m_updatedDownwardVelocity += m_initialDownwardVelocity;
		}
		//AZ_Printf("", "m_updatedDownwardVelocity = %.10f", m_updatedDownwardVelocity)
		//AZ_Printf("", "m_applyGravity = %.10f", m_applyGravity.GetZ())
	}

	void PlayerControllerComponent::UpdateRotation()
	{
		AZ::Vector3 current_yaw_Rotation = GetEntity()->GetTransform()->GetLocalRotation();
		AZ::Vector3 current_pitch_Rotation = GetActiveCamera()->GetTransform()->GetLocalRotation();

		m_delta_yaw = AZ::Vector3(0.f, 0.f, m_yaw);
		m_delta_pitch = AZ::Vector3(m_pitch, 0.f, 0.f);

		m_delta_yaw *= m_yaw_sensitivity;
		m_delta_pitch *= m_pitch_sensitivity;

		AZ::Vector3 new_yaw_Rotation = AZ::Vector3::CreateZero();
		new_yaw_Rotation = current_yaw_Rotation + m_delta_yaw;

		AZ::Vector3 new_pitch_Rotation = AZ::Vector3::CreateZero();
		new_pitch_Rotation = current_pitch_Rotation + m_delta_pitch;

		using namespace AZ::Constants;
			if (new_pitch_Rotation.GetX() >= Pi / 2)
				new_pitch_Rotation = AZ::Vector3(Pi / 2, 0.f, 0.f);
			if (new_pitch_Rotation.GetX() <= -Pi / 2)
				new_pitch_Rotation = AZ::Vector3(-Pi / 2, 0.f, 0.f);

		GetEntity()->GetTransform()->SetLocalRotation(new_yaw_Rotation);
		GetActiveCamera()->GetTransform()->SetLocalRotation(new_pitch_Rotation);

		//AZ_Printf("", "Get Pitch X value = %.10f", new_pitch_Rotation.GetX());

	}
	void PlayerControllerComponent::UpdateVelocity()
	{

		// Get our Entity's local rotation (Vector3) and setting just the Z component (float) to currentHeading
		m_currentHeading = GetEntity()->GetTransform()->GetLocalRotation().GetZ();
		
		/*
		const float currentHeadingE = GetEntity()->GetTransform()->
			GetWorldRotationQuaternion().GetEulerRadians().GetZ();

		AZ_Printf("", "Get Local Rotation Heading = %.10f", m_currentHeading);
		*/
		const float rightLeft = m_right + m_left;
		const float forwardBack = m_forward + m_backward;

		move = AZ::Vector3(rightLeft, forwardBack, 0.f);
		moveNormalized = move.GetNormalizedSafe();

		if (m_isSprintPressed)
		{
			//AZ_Printf("", "%s", m_isSprintPressed ? "Sprinting" : "NOT Sprinting");
			m_velocity = AZ::Quaternion::CreateRotationZ(m_currentHeading).TransformVector(moveNormalized * m_speed * m_sprintSpeed);
		}

		else
		{
			m_velocity = AZ::Quaternion::CreateRotationZ(m_currentHeading).TransformVector(moveNormalized * m_speed);
		}

		// Print raw input vector length values  to 10 decimal places
		// AZ_Printf("", "rawMovement = %.10f", move.GetLength());
		// Print normalized input vector length values to 10 decimal places
		// AZ_Printf("", "normalizedMovement = %.10f", moveNormalized.GetLength());

		//Prints X or Y velocity 
		/*
		AZ::Vector3 xyVelocity = AZ::Vector3::CreateZero();
		Physics::CharacterRequestBus::EventResult(currentVelocity, GetEntityId(),
			&Physics::CharacterRequestBus::Events::GetVelocity);

		AZ_Printf("", "xyVelocity.GetX() = %.10f", xyVelocity.GetX());
		AZ_Printf("", "xyVelocity.GetY() = %.10f", xyVelocity.GetY());
		*/
	}
	void PlayerControllerComponent::handleJump()
	{
		

		if (!m_isJumping && m_grounded && m_isJumpPressed)
		{
			m_isJumping = true;
			m_applyJump = AZ::Vector3(0.f, 0.f, m_initialJumpVelocity);
		}
		
		else if (!m_isJumpPressed && m_isJumping && m_grounded)
		{
			m_isJumping = false;
		
		}
		
		else if (!m_isJumping && m_grounded)
		{
			m_applyJump = AZ::Vector3(0.f, 0.f, 0.f);
		}

	}
	void PlayerControllerComponent::ProcessInput(const float& deltaTime)
	{
		//AZ_Printf("", "m_velocity vector Length = %.10f", m_velocity.GetLength())
		//AZ_Printf("Player", "jump pressed value %f", m_jump);
		//AZ_Printf("", "m_applyJump.GetZ() = %.10f", m_applyJump.GetZ());
		//AZ_Printf("", "m_isJumpPressed = %s", m_isJumpPressed ? "true" : "false");
		//AZ_Printf("", "%s", m_grounded ? "Grounded" : "NOT Grounded");
		//AZ_Printf("", "m_isJumping = %s", m_isJumping ? "true" : "false");
		//AZ_Printf("", "m_velocity.GetZ() = %.10f", m_velocity.GetZ());

		// Updating character's velocity to include gravity
		m_velocity += ((m_applyGravity + m_applyJump) * deltaTime);

		Physics::CharacterRequestBus::Event(GetEntityId(),
			&Physics::CharacterRequestBus::Events::AddVelocityForTick, (m_velocity));
		
		UpdateRotation();
		UpdateVelocity();
		CheckGrounded();
		handleGravity();
		handleJump();
	}
}  //namespace Testgem
