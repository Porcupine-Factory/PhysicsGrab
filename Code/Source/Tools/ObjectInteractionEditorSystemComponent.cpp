
#include <AzCore/Serialization/SerializeContext.h>
#include "ObjectInteractionEditorSystemComponent.h"

namespace ObjectInteraction
{
    void ObjectInteractionEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ObjectInteractionEditorSystemComponent, ObjectInteractionSystemComponent>()
                ->Version(0);
        }
    }

    ObjectInteractionEditorSystemComponent::ObjectInteractionEditorSystemComponent() = default;

    ObjectInteractionEditorSystemComponent::~ObjectInteractionEditorSystemComponent() = default;

    void ObjectInteractionEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("ObjectInteractionEditorService"));
    }

    void ObjectInteractionEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("ObjectInteractionEditorService"));
    }

    void ObjectInteractionEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void ObjectInteractionEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void ObjectInteractionEditorSystemComponent::Activate()
    {
        ObjectInteractionSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void ObjectInteractionEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        ObjectInteractionSystemComponent::Deactivate();
    }

} // namespace ObjectInteraction
