
if(TARGET Gem::AMultiplayerPhysX5.Unified.Static)
set(FILES
    Source/PhysicsGrabModuleInterface.cpp
    Source/PhysicsGrabModuleInterface.h
    Source/Clients/PhysicsGrabSystemComponent.cpp
    Source/Clients/PhysicsGrabSystemComponent.h
    Source/Clients/PhysicsGrabComponent.cpp
    Source/Clients/PhysicsGrabComponent.h
    Source/Multiplayer/NetworkPhysicsGrabComponent.cpp
    Source/Multiplayer/NetworkPhysicsGrabComponent.h

    Source/AutoGen/NetworkPhysicsGrabComponent.AutoComponent.xml
)
else()
set(FILES
    Source/PhysicsGrabModuleInterface.cpp
    Source/PhysicsGrabModuleInterface.h
    Source/Clients/PhysicsGrabSystemComponent.cpp
    Source/Clients/PhysicsGrabSystemComponent.h
    Source/Clients/PhysicsGrabComponent.cpp
    Source/Clients/PhysicsGrabComponent.h
)
endif()