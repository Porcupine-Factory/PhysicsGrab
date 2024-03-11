
#include <ObjectInteractionModuleInterface.h>
#include "ObjectInteractionEditorSystemComponent.h"

namespace ObjectInteraction
{
    class ObjectInteractionEditorModule
        : public ObjectInteractionModuleInterface
    {
    public:
        AZ_RTTI(ObjectInteractionEditorModule, "{852D08C1-2525-4131-A9DF-280B77AD7D8B}", ObjectInteractionModuleInterface);
        AZ_CLASS_ALLOCATOR(ObjectInteractionEditorModule, AZ::SystemAllocator, 0);

        ObjectInteractionEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
            // This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(m_descriptors.end(), {
                ObjectInteractionEditorSystemComponent::CreateDescriptor(),
            });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList {
                azrtti_typeid<ObjectInteractionEditorSystemComponent>(),
            };
        }
    };
}// namespace ObjectInteraction

AZ_DECLARE_MODULE_CLASS(Gem_ObjectInteraction, ObjectInteraction::ObjectInteractionEditorModule)
