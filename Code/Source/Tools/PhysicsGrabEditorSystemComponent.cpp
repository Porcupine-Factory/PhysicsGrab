
#include "PhysicsGrabEditorSystemComponent.h"
#include <AzCore/Serialization/SerializeContext.h>

#include <PhysicsGrab/PhysicsGrabTypeIds.h>

namespace PhysicsGrab
{
    AZ_COMPONENT_IMPL(
        PhysicsGrabEditorSystemComponent, "PhysicsGrabEditorSystemComponent", PhysicsGrabEditorSystemComponentTypeId, BaseSystemComponent);

    void PhysicsGrabEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<PhysicsGrabEditorSystemComponent, PhysicsGrabSystemComponent>()->Version(0);
        }
    }

    PhysicsGrabEditorSystemComponent::PhysicsGrabEditorSystemComponent() = default;

    PhysicsGrabEditorSystemComponent::~PhysicsGrabEditorSystemComponent() = default;

    void PhysicsGrabEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("PhysicsGrabEditorService"));
    }

    void PhysicsGrabEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("PhysicsGrabEditorService"));
    }

    void PhysicsGrabEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void PhysicsGrabEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void PhysicsGrabEditorSystemComponent::Activate()
    {
        PhysicsGrabSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void PhysicsGrabEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        PhysicsGrabSystemComponent::Deactivate();
    }

} // namespace PhysicsGrab
