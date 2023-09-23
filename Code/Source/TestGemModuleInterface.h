
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>
#include <Clients/TestGemSystemComponent.h>
#include <Clients/MyComponent.h>
#include <Clients/PlayerControllerComponent.h>

namespace TestGem
{
    class TestGemModuleInterface
        : public AZ::Module
    {
    public:
        AZ_RTTI(TestGemModuleInterface, "{66E60AA9-AA13-41C7-A73F-5378670BAF94}", AZ::Module);
        AZ_CLASS_ALLOCATOR(TestGemModuleInterface, AZ::SystemAllocator, 0);

        TestGemModuleInterface()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
            // This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(m_descriptors.end(), {
                TestGemSystemComponent::CreateDescriptor(),
                MyComponent::CreateDescriptor(),
                PlayerControllerComponent::CreateDescriptor()
                });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<TestGemSystemComponent>(),
            };
        }
    };
}// namespace TestGem
