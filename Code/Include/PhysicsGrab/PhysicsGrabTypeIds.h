
#pragma once

namespace PhysicsGrab
{
    // System Component TypeIds
    inline constexpr const char* PhysicsGrabSystemComponentTypeId = "{FF213BF9-0DE3-4F97-8547-13AAEE9510CD}";
    inline constexpr const char* PhysicsGrabEditorSystemComponentTypeId = "{FD4B66BA-E482-4708-A43B-18550DD43A48}";

    // Component TypeId from PhysicsGrabComponent.h
    inline constexpr const char* PhysicsGrabComponentTypeId = "{25D20C51-48A5-430A-AE29-4A175D770A14}";

    // EBus-related IDs from PhysicsGrabComponentBus.h
    inline constexpr const char* PhysicsGrabNotificationHandlerTypeId = "{6A1AF733-D5D8-49C4-A01F-5C83240C0215}";

    // Module derived classes TypeIds
    inline constexpr const char* PhysicsGrabModuleInterfaceTypeId = "{950BD36E-B7C2-4485-9A94-C111666A85D5}";
    inline constexpr const char* PhysicsGrabModuleTypeId = "{8A5B6200-0CA9-4E76-85CC-DE52D16C7515}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* PhysicsGrabEditorModuleTypeId = PhysicsGrabModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* PhysicsGrabRequestsTypeId = "{A3B8A1CC-FEEB-4404-BB6E-87B275CA9CA9}";
} // namespace PhysicsGrab
