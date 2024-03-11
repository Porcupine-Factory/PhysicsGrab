
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>
#include <Clients/ObjectInteractionSystemComponent.h>
#include <Clients/MyComponent.h>
#include <Clients/PlayerControllerComponent.h>
#include <Clients/RaycastTest.h>
#include <Clients/HeadBob.h>
#include <Clients/CameraShake.h>
#include <Clients/Grab.h>

namespace ObjectInteraction
{
    class ObjectInteractionModuleInterface
        : public AZ::Module
    {
    public:
        AZ_RTTI(ObjectInteractionModuleInterface, "{66E60AA9-AA13-41C7-A73F-5378670BAF94}", AZ::Module);
        AZ_CLASS_ALLOCATOR(ObjectInteractionModuleInterface, AZ::SystemAllocator, 0);

        ObjectInteractionModuleInterface()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
            // This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(m_descriptors.end(), {
                ObjectInteractionSystemComponent::CreateDescriptor(),
                MyComponent::CreateDescriptor(),
                PlayerControllerComponent::CreateDescriptor(),
                RaycastTest::CreateDescriptor(),
                HeadBob::CreateDescriptor(),
                CameraShake::CreateDescriptor(),
                Grab::CreateDescriptor()
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
