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

    private:
    // PhysicsGrabComponent object
    PhysicsGrabComponent* m_physicsGrabObject = nullptr;

    // Event value multipliers
    float m_grabKeyValue = 0.f;
    float m_throwKeyValue = 0.f;
    float m_rotateKeyValue = 0.f;
    float m_prevGrabKeyValue = 0.f;
    float m_prevRotateKeyValue = 0.f;
    float m_prevThrowKeyValue = 0.f;
    float m_grabDistanceKeyValue = 0.f;
    float m_pitchKeyValue = 0.f;
    float m_yawKeyValue = 0.f;
    float m_rollKeyValue = 0.f;

    // Event IDs and action names
    StartingPointInput::InputEventNotificationId m_grabEventId;
    AZStd::string m_strGrab = "Grab";
    StartingPointInput::InputEventNotificationId m_throwEventId;
    AZStd::string m_strThrow = "Throw";
    StartingPointInput::InputEventNotificationId m_rotateEventId;
    AZStd::string m_strRotate = "Rotate Enable";
    StartingPointInput::InputEventNotificationId m_rotatePitchEventId;
    AZStd::string m_strRotatePitch = "Rotate Pitch";
    StartingPointInput::InputEventNotificationId m_rotateYawEventId;
    AZStd::string m_strRotateYaw = "Rotate Yaw";
    StartingPointInput::InputEventNotificationId m_rotateRollEventId;
    AZStd::string m_strRotateRoll = "Rotate Roll";
    StartingPointInput::InputEventNotificationId m_grabDistanceEventId;
    AZStd::string m_strGrabDistance = "Grab Distance";
    };
} // namespace PhysicsGrab