

#include "ObjectInteractionSystemComponent.h"
#include <ObjectInteractionModuleInterface.h>

namespace ObjectInteraction
{
    class ObjectInteractionModule : public ObjectInteractionModuleInterface
    {
    public:
        AZ_RTTI(ObjectInteractionModule, "{852D08C1-2525-4131-A9DF-280B77AD7D8B}", ObjectInteractionModuleInterface);
        AZ_CLASS_ALLOCATOR(ObjectInteractionModule, AZ::SystemAllocator, 0);
    };
} // namespace ObjectInteraction

AZ_DECLARE_MODULE_CLASS(Gem_ObjectInteraction, ObjectInteraction::ObjectInteractionModule)
