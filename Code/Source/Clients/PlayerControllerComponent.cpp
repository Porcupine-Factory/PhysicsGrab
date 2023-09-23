#include <Clients/PlayerControllerComponent.h>

#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Serialization/EditContext.h>

//#include <AzFramework/Physics/CharacterBus.h>
//#include <AzFramework/Physics/RigidBodyBus.h>
//#include <AzFramework/Physics/CollisionBus.h>
//#include <AzFramework/Physics/SystemBus.h>
#include <AzFramework/Components/CameraBus.h>

//#include <PhysX/CharacterControllerBus.h>
//#include <System/PhysXSystem.h>

namespace TestGem
{
	using namespace StartingPointInput;

	void PlayerControllerComponent::Reflect(AZ::ReflectContext* rc)
	{
		if (auto sc = azrtti_cast<AZ::SerializeContext*>(rc))
		{
			sc->Class<PlayerControllerComponent, AZ::Component>()
				->Field("Speed", &PlayerControllerComponent::m_speed)
				->Field("Pitch Sensitivity", &PlayerControllerComponent::m_pitch_sensitivity)
				->Field("Yaw Sensitivity", &PlayerControllerComponent::m_yaw_sensitivity)
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
						"Speed", "Player's Speed");
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
			//AZ_Printf("Player", "forward value %f", value);
		}

		if (*inputId == MoveBackEventId)
		{
			m_backward = value;
			//AZ_Printf("Player", "backward value %f", value);
		}

		if (*inputId == MoveRightEventId)
		{
			m_right = value;
			//AZ_Printf("Player", "right value %f", value);
		}

		if (*inputId == MoveLeftEventId)
		{
			m_left = value;
			//AZ_Printf("Player", "left value %f", value);
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
			//AZ_Printf("Player", "forward value %f", value);
		}
		if (*inputId == MoveBackEventId)
		{
			m_backward = value;
			//AZ_Printf("Player", "back value %f", value);
		}
		if (*inputId == MoveRightEventId)
		{
			m_right = value;
			//AZ_Printf("Player", "right value %f", value);
		}
		if (*inputId == MoveLeftEventId)
		{
			m_left = value;
			//AZ_Printf("Player", "left value %f", value);
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
	}

	void PlayerControllerComponent::OnTick(float, AZ::ScriptTimePoint)
	{
		ProcessInput();
	}

	AZ::Entity* PlayerControllerComponent::GetActiveCamera()
	{
		AZ::EntityId activeCameraId;
		Camera::CameraSystemRequestBus::BroadcastResult(activeCameraId,
			&Camera::CameraSystemRequestBus::Events::GetActiveCamera);

		auto ca = AZ::Interface<AZ::ComponentApplicationRequests>::Get();
		return ca->FindEntity(activeCameraId);
	}
	/*
	void PlayerControllerComponent::CheckGrounded(/)
	{
		AZ::Vector3 current_Translation = GetEntity()->GetTransform()->GetLocalTranslation();
		auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();

		AzPhysics::RayCastRequest request;
		request.m_start = current_Translation;
		request.m_direction = AZ::Vector3(0.0f, 0.0f, -1.0f);
		request.m_distance = 0.2f;

		AzPhysics::SceneQueryHits result = sceneInterface->QueryScene(AzPhysics::DefaultPhysicsSceneName, &request);
		auto numHits = result.size();

		if (numHits > 0)
			AZ_Printf("", "Grounded");
		else
			AZ_Printf("", "NOT Grounded");
	}
	*/
	void PlayerControllerComponent::UpdateRotation()
	{
		AZ::Vector3 current_yaw_Rotation = GetEntity()->GetTransform()->GetLocalRotation();
		AZ::Vector3 current_pitch_Rotation = GetActiveCamera()->GetTransform()->GetLocalRotation();

		//AZ::Vector3 m_delta_yaw = AZ::Vector3::CreateZero();
		AZ::Vector3 m_delta_yaw = AZ::Vector3(0.f, 0.f, m_yaw);
		AZ::Vector3 m_delta_pitch = AZ::Vector3(m_pitch, 0.f, 0.f);

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

		// Getting our Entity's local rotation (Vector3) and setting just the Z component (float) to currentHeading
		const float currentHeading = GetEntity()->GetTransform()->GetLocalRotation().GetZ();
		
		//const float currentHeadingE = GetEntity()->GetTransform()->
		//	GetWorldRotationQuaternion().GetEulerRadians().GetZ();

		//AZ_Printf("", "Get Local Rotation Heading = %.10f", currentHeading);

		const float rightLeft = m_right + m_left;
		const float forwardBack = m_forward + m_backward;

		AZ::Vector3 move = AZ::Vector3::CreateZero();

		move = AZ::Vector3(rightLeft, forwardBack, 0.f);
		AZ::Vector3 moveNormalized = move.GetNormalized();

		// Print raw input vector length values  to 10 decimal places
		// AZ_Printf("", "rawMovement = %.10f", move.GetLength());
		// Print normalized input vector length values to 10 decimal places
		// AZ_Printf("", "normalizedMovement = %.10f", moveNormalized.GetLength());

		m_velocity = AZ::Quaternion::CreateRotationZ(currentHeading).TransformVector(moveNormalized * m_speed);

		Physics::CharacterRequestBus::Event(GetEntityId(),
			&Physics::CharacterRequestBus::Events::AddVelocityForTick, m_velocity);
	}

	void PlayerControllerComponent::ProcessInput(
		/*const PlayerInput& input*/)
	{
		UpdateRotation();
		UpdateVelocity();
		//CheckGrounded();
	}
}  //namespace Testgem
	


/*void MyComponent::Reflect(AZ::ReflectContext* reflection)
{
	//AZ_UNUSED(reflection);
	auto sc = azrtti_cast<AZ::SerializeContext*>(reflection);
	if (!sc) return;

	sc->Class<MyComponent, Component>()
		->Version(1);

	AZ::EditContext* ec = sc->GetEditContext();
	if (!ec) return;

	using namespace AZ::Edit::Attributes;

		// reflection of this component for O3DE Editor

		ec->Class<MyComponent>("My Component", "[Test new component]")
		->ClassElement(AZ::Edit::ClassElements::EditorData, "")
		->Attribute(AppearsInAddComponentMenu, AZ_CRC("Game"))
		->Attribute(Category, "TestGem");
}
*/

