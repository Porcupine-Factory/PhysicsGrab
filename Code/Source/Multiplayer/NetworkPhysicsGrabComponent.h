#pragma once

#include <PhysicsGrab/NetworkPhysicsGrabComponentBus.h>

#include <Source/AutoGen/NetworkPhysicsGrabComponent.AutoComponent.h>

#include <Clients/PhysicsGrabComponent.h>

#include <Multiplayer/Components/NetworkRigidBodyComponent.h>

namespace PhysicsGrab
{

    class NetworkPhysicsGrabComponentController
        : public NetworkPhysicsGrabComponentControllerBase
        , public NetworkPhysicsGrabComponentRequestBus::Handler
        , public StartingPointInput::InputEventNotificationBus::MultiHandler
    {
        friend class PhysicsGrabComponent;

    public:
        explicit NetworkPhysicsGrabComponentController(NetworkPhysicsGrabComponent& parent);

        void OnActivate(Multiplayer::EntityIsMigrating entityIsMigrating) override;
        void OnDeactivate(Multiplayer::EntityIsMigrating entityIsMigrating) override;

        //! Common input creation logic for the NetworkInput.
        //! Fill out the input struct and the MultiplayerInputDriver will send the input data over the network
        //!    to ensure it's processed.
        //! @param input  input structure which to store input data for sending to the authority
        //! @param deltaTime amount of time to integrate the provided inputs over
        void CreateInput(Multiplayer::NetworkInput& input, float deltaTime) override;

        //! Common input processing logic for the NetworkInput.
        //! @param input  input structure to process
        //! @param deltaTime amount of time to integrate the provided inputs over
        void ProcessInput(Multiplayer::NetworkInput& input, float deltaTime) override;

        // NetworkPhysicsGrabComponentControllerRequestBus
        bool GetIsNetEntityAutonomous() const override;
        bool GetEnabled() const override;
        void SetEnabled(const bool& new_enabled) override;

        // AZ::InputEventNotificationBus interface
        void OnPressed(float value) override;
        void OnReleased(float value) override;
        void OnHeld(float value) override;

    private:
        // Input event assignment and notification bus connection
        void AssignConnectInputEvents();

        // NetworkPhysicsGrabComponentControllerNotificationBus
        void OnNetworkTickStart(const float& deltaTime, const bool& server, const AZ::EntityId& entity);
        void OnNetworkTickFinish(const float& deltaTime, const bool& server, const AZ::EntityId& entity);

        // Keep track of the previous deltaTime for averaging
        float m_prevDeltaTime = 1.f / 60.f;

        // EnableNetworkPhysicsGrabComponent Changed Event
        AZ::Event<bool>::Handler m_enableNetworkPhysicsGrabComponentChangedEvent;
        void OnEnableNetworkPhysicsGrabComponentChanged(const bool& enable);
        bool m_disabled = false;

        // Signals when the controller is determined to be autonomous or not
        bool m_autonomousNotDetermined = true;

        // PhysicsGrabComponent object
        PhysicsGrabComponent* m_physicsGrabObject = nullptr;

        // Event value multipliers
        float m_grabKeyValue = 0.f;
        float m_rotateKeyValue = 0.f;
        float m_throwKeyValue = 0.f;
        float m_grabDistanceKeyValue = 0.f;
        float m_pitchKeyValue = 0.f;
        float m_yawKeyValue = 0.f;
        float m_rollKeyValue = 0.f;

        // Event IDs and action names
        StartingPointInput::InputEventNotificationId m_grabEventId;
        AZStd::string m_strGrab = "Grab";
        StartingPointInput::InputEventNotificationId m_rotateEventId;
        AZStd::string m_strRotate = "Rotate Enable";
        StartingPointInput::InputEventNotificationId m_throwEventId;
        AZStd::string m_strThrow = "Throw";
        StartingPointInput::InputEventNotificationId m_rotatePitchEventId;
        AZStd::string m_strRotatePitch = "Rotate Pitch";
        StartingPointInput::InputEventNotificationId m_rotateYawEventId;
        AZStd::string m_strRotateYaw = "Rotate Yaw";
        StartingPointInput::InputEventNotificationId m_rotateRollEventId;
        AZStd::string m_strRotateRoll = "Rotate Roll";
        StartingPointInput::InputEventNotificationId m_grabDistanceEventId;
        AZStd::string m_strGrabDistance = "Grab Distance";

        // Array of action names
        AZStd::string* m_inputNames[7] = { &m_strGrab,      &m_strRotate,     &m_strThrow,       &m_strRotatePitch,
                                           &m_strRotateYaw, &m_strRotateRoll, &m_strGrabDistance };

        // Map of event IDs and event value multipliers
        AZStd::map<StartingPointInput::InputEventNotificationId*, float*> m_controlMap = {
            { &m_grabEventId, &m_grabKeyValue },         { &m_rotateEventId, &m_rotateKeyValue },
            { &m_throwEventId, &m_throwKeyValue },       { &m_grabDistanceEventId, &m_grabDistanceKeyValue },
            { &m_rotatePitchEventId, &m_pitchKeyValue }, { &m_rotateYawEventId, &m_yawKeyValue },
            { &m_rotateRollEventId, &m_rollKeyValue }
        };
    };
} // namespace PhysicsGrab