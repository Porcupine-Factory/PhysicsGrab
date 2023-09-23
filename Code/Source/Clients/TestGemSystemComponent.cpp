
#include "TestGemSystemComponent.h"

#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

namespace TestGem
{
    void TestGemSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<TestGemSystemComponent, AZ::Component>()
                ->Version(0)
                ;

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<TestGemSystemComponent>("TestGem", "[Description of functionality provided by this System Component]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }
        }
    }

    void TestGemSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("TestGemService"));
    }

    void TestGemSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("TestGemService"));
    }

    void TestGemSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void TestGemSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    TestGemSystemComponent::TestGemSystemComponent()
    {
        if (TestGemInterface::Get() == nullptr)
        {
            TestGemInterface::Register(this);
        }
    }

    TestGemSystemComponent::~TestGemSystemComponent()
    {
        if (TestGemInterface::Get() == this)
        {
            TestGemInterface::Unregister(this);
        }
    }

    void TestGemSystemComponent::Init()
    {
    }

    void TestGemSystemComponent::Activate()
    {
        TestGemRequestBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();
    }

    void TestGemSystemComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        TestGemRequestBus::Handler::BusDisconnect();
    }

    void TestGemSystemComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
    }

} // namespace TestGem
