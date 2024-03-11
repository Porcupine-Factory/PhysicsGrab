
#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Clients/ObjectInteractionSystemComponent.h>

namespace ObjectInteraction
{
    /// System component for ObjectInteraction editor
    class ObjectInteractionEditorSystemComponent
        : public ObjectInteractionSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = ObjectInteractionSystemComponent;
    public:
        AZ_COMPONENT(ObjectInteractionEditorSystemComponent, "{C4A42620-D49B-4853-B06C-275FE75268F5}", BaseSystemComponent);
        static void Reflect(AZ::ReflectContext* context);

        ObjectInteractionEditorSystemComponent();
        ~ObjectInteractionEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace ObjectInteraction
