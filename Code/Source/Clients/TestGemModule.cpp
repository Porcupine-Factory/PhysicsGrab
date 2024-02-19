

#include "TestGemSystemComponent.h"
#include <TestGemModuleInterface.h>

namespace TestGem
{
    class TestGemModule : public TestGemModuleInterface
    {
    public:
        AZ_RTTI(TestGemModule, "{852D08C1-2525-4131-A9DF-280B77AD7D8B}", TestGemModuleInterface);
        AZ_CLASS_ALLOCATOR(TestGemModule, AZ::SystemAllocator, 0);
    };
} // namespace TestGem

AZ_DECLARE_MODULE_CLASS(Gem_TestGem, TestGem::TestGemModule)
