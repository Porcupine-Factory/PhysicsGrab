#pragma once

#include <Source/AutoGen/NetworkPhysicsGrabComponent.AutoComponent.h>

#include <Clients/PhysicsGrabComponent.h>

namespace PhysicsGrab
{

    class NetworkPhysicsGrabComponentController
        : public NetworkPhysicsGrabComponentControllerBase
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

        // AZ::InputEventNotificationBus interface
        void OnPressed(float value) override;
        void OnReleased(float value) override;
        void OnHeld(float value) override;

    private:
        // Input event assignment and notification bus connection
        void AssignConnectInputEvents();

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
        StartingPointInput::InputEventNotificationId m_grabDistanceEventId;
        AZStd::string m_strGrabDistance = "Grab Distance";
        StartingPointInput::InputEventNotificationId m_pitchEventId;
        AZStd::string m_strPitch = "Rotate Pitch";
        StartingPointInput::InputEventNotificationId m_yawEventId;
        AZStd::string m_strYaw = "Rotate Yaw";
        StartingPointInput::InputEventNotificationId m_rollEventId;
        AZStd::string m_strRoll = "Rotate Roll";

        // Array of action names
        AZStd::string* m_inputNames[7] = { &m_strGrab, &m_strRotate, &m_strThrow, &m_strPitch, &m_strYaw, &m_strRoll, &m_strGrabDistance };

        // Map of event IDs and event value multipliers
        AZStd::map<StartingPointInput::InputEventNotificationId*, float*> m_controlMap = {
            { &m_grabEventId, &m_grabKeyValue },   { &m_rotateEventId, &m_rotateKeyValue },
            { &m_throwEventId, &m_throwKeyValue }, { &m_grabDistanceEventId, &m_grabDistanceKeyValue },
            { &m_pitchEventId, &m_pitchKeyValue }, { &m_yawEventId, &m_yawKeyValue },
            { &m_rollEventId, &m_rollKeyValue }
        };
    };
} // namespace PhysicsGrab