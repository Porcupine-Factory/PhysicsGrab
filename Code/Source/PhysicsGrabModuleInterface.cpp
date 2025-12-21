
#include "PhysicsGrabModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <PhysicsGrab/PhysicsGrabTypeIds.h>

#include <Clients/PhysicsGrabComponent.h>
#include <Clients/PhysicsGrabSystemComponent.h>

#include <Source/AutoGen/AutoComponentTypes.h>

namespace PhysicsGrab
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(PhysicsGrabModuleInterface, "PhysicsGrabModuleInterface", PhysicsGrabModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(PhysicsGrabModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(PhysicsGrabModuleInterface, AZ::SystemAllocator);

    PhysicsGrabModuleInterface::PhysicsGrabModuleInterface()
    {
        // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
        // Add ALL components descriptors associated with this gem to m_descriptors.
        // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
        // This happens through the [MyComponent]::Reflect() function.
        m_descriptors.insert(
            m_descriptors.end(), { PhysicsGrabSystemComponent::CreateDescriptor(), PhysicsGrabComponent::CreateDescriptor() });

        //< Register multiplayer components
        CreateComponentDescriptors(m_descriptors);
    }

    AZ::ComponentTypeList PhysicsGrabModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<PhysicsGrabSystemComponent>(),
        };
    }
} // namespace PhysicsGrab
