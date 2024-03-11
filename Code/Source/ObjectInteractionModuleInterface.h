
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>
#include <Clients/ObjectInteractionSystemComponent.h>
#include <Clients/ObjectInteraction.h>

namespace ObjectInteraction
{
    class ObjectInteractionModuleInterface
        : public AZ::Module
    {
    public:
        AZ_RTTI(ObjectInteractionModuleInterface, "{14CDD3AC-F126-4D63-877D-7D4316EA32C9}", AZ::Module);
        AZ_CLASS_ALLOCATOR(ObjectInteractionModuleInterface, AZ::SystemAllocator, 0);

        ObjectInteractionModuleInterface()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
            // This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(m_descriptors.end(), {
                ObjectInteractionSystemComponent::CreateDescriptor(),
                ObjectInteraction::CreateDescriptor()
                });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<ObjectInteractionSystemComponent>(),
            };
        }
    };
}// namespace ObjectInteraction
