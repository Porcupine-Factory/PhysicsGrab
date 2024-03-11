
#include "ObjectInteractionSystemComponent.h"

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>

namespace ObjectInteraction
{
    void ObjectInteractionSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ObjectInteractionSystemComponent, AZ::Component>()->Version(0);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ObjectInteractionSystemComponent>("ObjectInteraction", "[Description of functionality provided by this System Component]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true);
            }
        }
    }

    void ObjectInteractionSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ObjectInteractionService"));
    }

    void ObjectInteractionSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ObjectInteractionService"));
    }

    void ObjectInteractionSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void ObjectInteractionSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    ObjectInteractionSystemComponent::ObjectInteractionSystemComponent()
    {
        if (ObjectInteractionInterface::Get() == nullptr)
        {
            ObjectInteractionInterface::Register(this);
        }
    }

    ObjectInteractionSystemComponent::~ObjectInteractionSystemComponent()
    {
        if (ObjectInteractionInterface::Get() == this)
        {
            ObjectInteractionInterface::Unregister(this);
        }
    }

    void ObjectInteractionSystemComponent::Init()
    {
    }

    void ObjectInteractionSystemComponent::Activate()
    {
        ObjectInteractionRequestBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();
    }

    void ObjectInteractionSystemComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        ObjectInteractionRequestBus::Handler::BusDisconnect();
    }

    void ObjectInteractionSystemComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
    }

} // namespace ObjectInteraction
