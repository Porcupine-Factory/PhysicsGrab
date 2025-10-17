
#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Clients/PhysicsGrabSystemComponent.h>

namespace PhysicsGrab
{
    /// System component for PhysicsGrab editor
    class PhysicsGrabEditorSystemComponent
        : public PhysicsGrabSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = PhysicsGrabSystemComponent;

    public:
        AZ_COMPONENT_DECL(PhysicsGrabEditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        PhysicsGrabEditorSystemComponent();
        ~PhysicsGrabEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace PhysicsGrab
