
#include <PhysicsGrab/PhysicsGrabTypeIds.h>
#include <PhysicsGrabModuleInterface.h>
#include "PhysicsGrabSystemComponent.h"

namespace PhysicsGrab
{
    class PhysicsGrabModule
        : public PhysicsGrabModuleInterface
    {
    public:
        AZ_RTTI(PhysicsGrabModule, PhysicsGrabModuleTypeId, PhysicsGrabModuleInterface);
        AZ_CLASS_ALLOCATOR(PhysicsGrabModule, AZ::SystemAllocator);
    };
}// namespace PhysicsGrab

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME), PhysicsGrab::PhysicsGrabModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_PhysicsGrab, PhysicsGrab::PhysicsGrabModule)
#endif
