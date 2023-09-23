
#include <AzCore/Serialization/SerializeContext.h>
#include "TestGemEditorSystemComponent.h"

namespace TestGem
{
    void TestGemEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<TestGemEditorSystemComponent, TestGemSystemComponent>()
                ->Version(0);
        }
    }

    TestGemEditorSystemComponent::TestGemEditorSystemComponent() = default;

    TestGemEditorSystemComponent::~TestGemEditorSystemComponent() = default;

    void TestGemEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("TestGemEditorService"));
    }

    void TestGemEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("TestGemEditorService"));
    }

    void TestGemEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void TestGemEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void TestGemEditorSystemComponent::Activate()
    {
        TestGemSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void TestGemEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        TestGemSystemComponent::Deactivate();
    }

} // namespace TestGem
