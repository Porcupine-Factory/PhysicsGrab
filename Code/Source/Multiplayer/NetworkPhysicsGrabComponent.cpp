#include <Multiplayer/NetworkPhysicsGrabComponent.h>

#include <AzCore/Serialization/SerializeContext.h>

namespace PhysicsGrab
{
    using namespace StartingPointInput;

    NetworkPhysicsGrabComponentController::NetworkPhysicsGrabComponentController(NetworkPhysicsGrabComponent& parent)
        : NetworkPhysicsGrabComponentControllerBase(parent)
        , m_enableNetworkPhysicsGrabComponentChangedEvent(
              [this](bool enable)
              {
                  OnEnableNetworkPhysicsGrabComponentChanged(enable);
              })
    {
    }

    void NetworkPhysicsGrabComponentController::AssignConnectInputEvents()
    {
        // Disconnect prior to connecting since this may be a reassignment
        InputEventNotificationBus::MultiHandler::BusDisconnect();

        if (m_controlMap.size() != (sizeof(m_inputNames) / sizeof(AZStd::string*)))
        {
            AZ_Error("NetworkPhysicsGrabComponentController", false, "Number of input IDs not equal to number of input names!");
        }
        else
        {
            const AZ::u8 size = sizeof(m_inputNames) / sizeof(AZStd::string*);

            for (AZ::u8 i = 0; i < size; ++i)
                m_inputNames[i] = m_physicsGrabObject->m_inputNames[i];

            for (auto& it_event : m_controlMap)
            {
                *(it_event.first) = StartingPointInput::InputEventNotificationId(
                    (m_inputNames[std::distance(m_controlMap.begin(), m_controlMap.find(it_event.first))])->c_str());
                InputEventNotificationBus::MultiHandler::BusConnect(*(it_event.first));
            }
        }
    }

    void NetworkPhysicsGrabComponentController::OnPressed(float value)
    {
        const InputEventNotificationId* inputId = InputEventNotificationBus::GetCurrentBusId();
        if (inputId == nullptr)
            return;

        if (*inputId == m_rotatePitchEventId)
        {
            m_pitchKeyValue += value;
            return;
        }

        if (*inputId == m_rotateYawEventId)
        {
            m_yawKeyValue += value;
            return;
        }

        if (*inputId == m_rotateRollEventId)
        {
            m_rollKeyValue += value;
            return;
        }

        for (auto& it_event : m_controlMap)
        {
            if (*inputId == *(it_event.first))
            {
                *(it_event.second) = value;
                // print the local user ID and the action name CRC
                // AZ_Printf("Pressed", it_event.first->ToString().c_str());
            }
        }
    }

    void NetworkPhysicsGrabComponentController::OnReleased(float value)
    {
        const InputEventNotificationId* inputId = InputEventNotificationBus::GetCurrentBusId();
        if (inputId == nullptr)
            return;

        if (*inputId == m_rotatePitchEventId)
        {
            return;
        }

        if (*inputId == m_rotateYawEventId)
        {
            return;
        }

        if (*inputId == m_rotateRollEventId)
        {
            return;
        }

        for (auto& it_event : m_controlMap)
        {
            if (*inputId == *(it_event.first))
            {
                *(it_event.second) = value;
                // print the local user ID and the action name CRC
                // AZ_Printf("Released", it_event.first->ToString().c_str());
            }
        }
    }

    void NetworkPhysicsGrabComponentController::OnHeld(float value)
    {
        const InputEventNotificationId* inputId = InputEventNotificationBus::GetCurrentBusId();
        if (inputId == nullptr)
            return;

        if (*inputId == m_rotatePitchEventId)
        {
            m_pitchKeyValue += value;
        }
        
        if (*inputId == m_rotateYawEventId)
        {
            m_yawKeyValue += value;
        }
        
        if (*inputId == m_rotateRollEventId)
        {
            m_rollKeyValue += value;
        }
    }

    void NetworkPhysicsGrabComponentController::OnActivate([[maybe_unused]] Multiplayer::EntityIsMigrating entityIsMigrating)
    {
        NetworkPhysicsGrabComponentRequestBus::Handler::BusConnect(GetEntityId());

        // Subscribe to EnableNetworkPhysicsGrabComponent change events
        EnableNetworkPhysicsGrabComponentAddEvent(m_enableNetworkPhysicsGrabComponentChangedEvent);

        // Get access to the PhysicsGrabComponent object and its members
        const AZ::Entity* entity = GetParent().GetEntity();
        m_physicsGrabObject = entity->FindComponent<PhysicsGrabComponent>();
        m_physicsGrabObject->m_networkPhysicsGrabComponentEnabled = GetEnableNetworkPhysicsGrabComponent();

        m_physicsGrabObject->NetworkPhysicsGrabComponentEnabledIgnoreInputs();

        if (IsNetEntityRoleAutonomous())
        {
            m_autonomousNotDetermined = false;
            m_physicsGrabObject->IsAutonomousSoConnect();
            AssignConnectInputEvents();
            if (IsNetEntityRoleAuthority())
                m_physicsGrabObject->m_isHost = true;
            else
                m_physicsGrabObject->m_isAutonomousClient = true;
        }
    }

    void NetworkPhysicsGrabComponentController::OnDeactivate([[maybe_unused]] Multiplayer::EntityIsMigrating entityIsMigrating)
    {
        InputEventNotificationBus::MultiHandler::BusDisconnect();
        NetworkPhysicsGrabComponentRequestBus::Handler::BusDisconnect();
        m_enableNetworkPhysicsGrabComponentChangedEvent.Disconnect();
    }

    void NetworkPhysicsGrabComponentController::CreateInput(
        [[maybe_unused]] Multiplayer::NetworkInput& input, [[maybe_unused]] float deltaTime)
    {
        auto* playerInput = input.FindComponentInput<NetworkPhysicsGrabComponentNetworkInput>();

        // Assign input values
        playerInput->m_grab = m_grabKeyValue;
        playerInput->m_rotate = m_rotateKeyValue;
        playerInput->m_throw = m_throwKeyValue;
        playerInput->m_grabDistance = m_grabDistanceKeyValue;
        playerInput->m_pitch = m_pitchKeyValue;
        playerInput->m_yaw = m_yawKeyValue;
        playerInput->m_roll = m_rollKeyValue;

        if (m_physicsGrabObject)
        {
            // Get the grabbing entity transform
            AZ::Transform grabbingEntityTransform = AZ::Transform::CreateIdentity();

#ifdef FIRST_PERSON_CONTROLLER
            if (m_physicsGrabObject->m_useFPControllerForGrab)
            {
                AZ::TransformInterface* fpcCameraRotationTransform = nullptr;
                FirstPersonController::FirstPersonControllerComponentRequestBus::EventResult(
                    fpcCameraRotationTransform,
                    GetEntityId(),
                    &FirstPersonController::FirstPersonControllerComponentRequests::GetCameraRotationTransform);
                if (fpcCameraRotationTransform)
                {
                    grabbingEntityTransform = fpcCameraRotationTransform->GetWorldTM();
                }
            }
            else
#endif
            {
                if (m_physicsGrabObject->m_grabbingEntityPtr)
                {
                    grabbingEntityTransform = m_physicsGrabObject->m_grabbingEntityPtr->GetTransform()->GetWorldTM();
                }
            }

            SetGrabbingEntityTranslation(grabbingEntityTransform.GetTranslation());
            SetGrabbingEntityRotation(grabbingEntityTransform.GetRotation());
            SetCurrentGrabDistance(m_physicsGrabObject->m_grabDistance);
        }

        m_pitchKeyValue = 0.0f;
        m_yawKeyValue = 0.0f;
        m_rollKeyValue = 0.0f;
    }

    void NetworkPhysicsGrabComponentController::ProcessInput(Multiplayer::NetworkInput& input, [[maybe_unused]] float deltaTime)
    {
        // Disconnect from various buses when the NetworkPhysicsGrabComponentController is not autonomous, and only do this once
        if (m_autonomousNotDetermined)
        {
            if (IsNetEntityRoleAuthority())
                m_physicsGrabObject->m_isServer = true;
            m_physicsGrabObject->NotAutonomousSoDisconnect();
            m_autonomousNotDetermined = false;
        }

        const auto* playerInput = input.FindComponentInput<NetworkPhysicsGrabComponentNetworkInput>();

        // Assign the Physics Grab's inputs from the input prediction
        m_physicsGrabObject->m_grabKeyValue = playerInput->m_grab;
        m_physicsGrabObject->m_rotateKeyValue = playerInput->m_rotate;
        m_physicsGrabObject->m_throwKeyValue = playerInput->m_throw;
        m_physicsGrabObject->m_grabDistanceKeyValue = playerInput->m_grabDistance;
        m_physicsGrabObject->m_pitchKeyValue = playerInput->m_pitch;
        m_physicsGrabObject->m_yawKeyValue = playerInput->m_yaw;
        m_physicsGrabObject->m_rollKeyValue = playerInput->m_roll;

        // AZ_Printf("NetworkPhysicsGrabComponent", "Grab: %f", playerInput->m_grab);
        // AZ_Printf("NetworkPhysicsGrabComponent", "Rotate: %f", playerInput->m_rotate);
        // AZ_Printf("NetworkPhysicsGrabComponent", "Throw: %f", playerInput->m_throw);
        // AZ_Printf("NetworkPhysicsGrabComponent", "Grab Distance: %f", playerInput->m_grabDistance);
        // AZ_Printf("NetworkPhysicsGrabComponent", "Pitch: %f", playerInput->m_pitch);
        // AZ_Printf("NetworkPhysicsGrabComponent", "Yaw: %f", playerInput->m_yaw);
        // AZ_Printf("NetworkPhysicsGrabComponent", "Roll: %f", playerInput->m_roll);

        // Store grabbing entity transform for server detection
        m_physicsGrabObject->m_networkCameraTranslation = GetGrabbingEntityTranslation();
        m_physicsGrabObject->m_networkCameraRotation = GetGrabbingEntityRotation();
        m_physicsGrabObject->m_grabDistance = GetCurrentGrabDistance();

        // Enable network camera transform usage for server
        if (m_physicsGrabObject->m_isServer)
        {
            m_physicsGrabObject->m_useNetworkCameraTransform = true;
        }

        NetworkPhysicsGrabComponentNotificationBus::Broadcast(
            &NetworkPhysicsGrabComponentNotificationBus::Events::OnNetworkTickStart,
            deltaTime,
            m_physicsGrabObject->m_isServer,
            GetEntityId());

        NetworkPhysicsGrabComponentNotificationBus::Broadcast(
            &NetworkPhysicsGrabComponentNotificationBus::Events::OnNetworkTickFinish,
            deltaTime,
            m_physicsGrabObject->m_isServer,
            GetEntityId());
    }

    // Event Notification methods for use in scripts
    void NetworkPhysicsGrabComponentController::OnNetworkTickStart(
        [[maybe_unused]] const float& deltaTime, [[maybe_unused]] const bool& server, [[maybe_unused]] const AZ::EntityId& entity)
    {
    }
    void NetworkPhysicsGrabComponentController::OnNetworkTickFinish(
        [[maybe_unused]] const float& deltaTime, [[maybe_unused]] const bool& server, [[maybe_unused]] const AZ::EntityId& entity)
    {
    }

    void NetworkPhysicsGrabComponentController::OnEnableNetworkPhysicsGrabComponentChanged(const bool& enable)
    {
        m_disabled = !enable;
        m_physicsGrabObject->m_networkPhysicsGrabComponentEnabled = enable;
        if (!m_disabled)
        {
            m_physicsGrabObject->NetworkPhysicsGrabComponentEnabledIgnoreInputs();
            AssignConnectInputEvents();
        }
        else
        {
            InputEventNotificationBus::MultiHandler::BusDisconnect();
            m_physicsGrabObject->AssignConnectInputEvents();
        }
    }

    // Request Bus getter and setter methods for use in scripts
    bool NetworkPhysicsGrabComponentController::GetIsNetEntityAutonomous() const
    {
        return IsNetEntityRoleAutonomous();
    }

    bool NetworkPhysicsGrabComponentController::GetEnabled() const
    {
        return !m_disabled;
    }

    void NetworkPhysicsGrabComponentController::SetEnabled(const bool& new_enabled)
    {
        m_disabled = !new_enabled;
        m_physicsGrabObject->m_networkPhysicsGrabComponentEnabled = new_enabled;
        if (!m_disabled)
        {
            m_physicsGrabObject->NetworkPhysicsGrabComponentEnabledIgnoreInputs();
            AssignConnectInputEvents();
        }
        else
        {
            InputEventNotificationBus::MultiHandler::BusDisconnect();
            m_physicsGrabObject->AssignConnectInputEvents();
        }
    }
} // namespace PhysicsGrab