
#include "PhysicsGrabSystemComponent.h"

#include <PhysicsGrab/PhysicsGrabTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>

#include <Source/AutoGen/AutoComponentTypes.h>

namespace PhysicsGrab
{
    AZ_COMPONENT_IMPL(PhysicsGrabSystemComponent, "PhysicsGrabSystemComponent", PhysicsGrabSystemComponentTypeId);

    void PhysicsGrabSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<PhysicsGrabSystemComponent, AZ::Component>()->Version(0);
        }
    }

    void PhysicsGrabSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("PhysicsGrabService"));
    }

    void PhysicsGrabSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("PhysicsGrabService"));
    }

    void PhysicsGrabSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void PhysicsGrabSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    PhysicsGrabSystemComponent::PhysicsGrabSystemComponent()
    {
        if (PhysicsGrabInterface::Get() == nullptr)
        {
            PhysicsGrabInterface::Register(this);
        }
    }

    PhysicsGrabSystemComponent::~PhysicsGrabSystemComponent()
    {
        if (PhysicsGrabInterface::Get() == this)
        {
            PhysicsGrabInterface::Unregister(this);
        }
    }

    void PhysicsGrabSystemComponent::Init()
    {
    }

    void PhysicsGrabSystemComponent::Activate()
    {
        PhysicsGrabRequestBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();

        // Register multiplayer components
        RegisterMultiplayerComponents();
    }

    void PhysicsGrabSystemComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        PhysicsGrabRequestBus::Handler::BusDisconnect();
    }

    void PhysicsGrabSystemComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
    }

} // namespace PhysicsGrab
