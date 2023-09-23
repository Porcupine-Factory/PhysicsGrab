
#include <TestGemModuleInterface.h>
#include "TestGemEditorSystemComponent.h"

namespace TestGem
{
    class TestGemEditorModule
        : public TestGemModuleInterface
    {
    public:
        AZ_RTTI(TestGemEditorModule, "{852D08C1-2525-4131-A9DF-280B77AD7D8B}", TestGemModuleInterface);
        AZ_CLASS_ALLOCATOR(TestGemEditorModule, AZ::SystemAllocator, 0);

        TestGemEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
            // This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(m_descriptors.end(), {
                TestGemEditorSystemComponent::CreateDescriptor(),
            });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList {
                azrtti_typeid<TestGemEditorSystemComponent>(),
            };
        }
    };
}// namespace TestGem

AZ_DECLARE_MODULE_CLASS(Gem_TestGem, TestGem::TestGemEditorModule)
