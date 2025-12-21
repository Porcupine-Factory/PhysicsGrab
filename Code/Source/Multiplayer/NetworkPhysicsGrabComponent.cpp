#include <Multiplayer/NetworkPhysicsGrabComponent.h>

#include <AzCore/Serialization/SerializeContext.h>

namespace PhysicsGrab
{
    NetworkPhysicsGrabComponentController::NetworkPhysicsGrabComponentController(NetworkPhysicsGrabComponent& parent)
        : NetworkPhysicsGrabComponentControllerBase(parent)
    {
    }

    void NetworkPhysicsGrabComponentController::OnActivate([[maybe_unused]] Multiplayer::EntityIsMigrating entityIsMigrating)
    {
    }

    void NetworkPhysicsGrabComponentController::OnDeactivate([[maybe_unused]] Multiplayer::EntityIsMigrating entityIsMigrating)
    {
    }

    void NetworkPhysicsGrabComponentController::CreateInput(
        [[maybe_unused]] Multiplayer::NetworkInput& input, [[maybe_unused]] float deltaTime)
    {
    }

    void NetworkPhysicsGrabComponentController::ProcessInput(
        [[maybe_unused]] Multiplayer::NetworkInput& input, [[maybe_unused]] float deltaTime)
    {
    }
} // namespace PhysicsGrab