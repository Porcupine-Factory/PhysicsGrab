

#include "ObjectInteractionSystemComponent.h"
#include <ObjectInteractionModuleInterface.h>

namespace ObjectInteraction
{
    class ObjectInteractionModule : public ObjectInteractionModuleInterface
    {
    public:
        AZ_RTTI(ObjectInteractionModule, "{07648EAA-D367-4B4F-A123-15F1055A68F9}", ObjectInteractionModuleInterface);
        AZ_CLASS_ALLOCATOR(ObjectInteractionModule, AZ::SystemAllocator, 0);
    };
} // namespace ObjectInteraction

AZ_DECLARE_MODULE_CLASS(Gem_ObjectInteraction, ObjectInteraction::ObjectInteractionModule)
