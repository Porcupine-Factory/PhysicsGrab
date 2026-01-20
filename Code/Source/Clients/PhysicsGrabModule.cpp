
#include "PhysicsGrabSystemComponent.h"
#include <PhysicsGrab/PhysicsGrabTypeIds.h>
#include <PhysicsGrabModuleInterface.h>

namespace PhysicsGrab
{
    class PhysicsGrabModule : public PhysicsGrabModuleInterface
    {
    public:
        AZ_RTTI(PhysicsGrabModule, PhysicsGrabModuleTypeId, PhysicsGrabModuleInterface);
        AZ_CLASS_ALLOCATOR(PhysicsGrabModule, AZ::SystemAllocator);
    };
} // namespace PhysicsGrab

// For monolithic builds, module classes for Client and Server variants
#if defined(AZ_MONOLITHIC_BUILD)
#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME, _Client), PhysicsGrab::PhysicsGrabModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_PhysicsGrab_Client, PhysicsGrab::PhysicsGrabModule)
#endif

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME, _Server), PhysicsGrab::PhysicsGrabModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_PhysicsGrab_Server, PhysicsGrab::PhysicsGrabModule)
#endif
#endif

// Unified variant declaration
#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME), PhysicsGrab::PhysicsGrabModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_PhysicsGrab, PhysicsGrab::PhysicsGrabModule)
#endif
