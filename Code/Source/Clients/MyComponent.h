#pragma once
#include <AzCore/Component/Component.h>
#include <StartingPointInput/InputEventNotificationBus.h>

namespace TestGem
{
    // An example of a simple O3DE component
    class MyComponent : public AZ::Component
    {
    public:
        AZ_COMPONENT(MyComponent, "{6C4B7FC6-F4CD-4DBA-99FD-99F7AA2CE8F0}");

        // AZ::Component overrides
        void Activate() override
        {
        }
        void Deactivate() override
        {
        }

        // Provide runtime reflection, if any
        static void Reflect(AZ::ReflectContext* reflection);
    };
} // namespace TestGem