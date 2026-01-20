find_path(FPC_INCLUDE_DIR
    NAMES FirstPersonController/FirstPersonControllerComponentBus.h
    PATHS
        "${LY_PROJECTS}/../../Gems/FirstPersonController/Code/Include"
        "${LY_PROJECTS}/../../Gems/FirstPersonController/1.0.0/Code/Include"
)

if(TARGET Gem::AMultiplayerPhysX5.Unified.Static)
    add_compile_definitions(NETWORKPHYSICSGRAB)
endif()

if(TARGET Gem::AMultiplayerPhysX5.Unified.Static)
    set(FILES
        Include/PhysicsGrab/PhysicsGrabComponentBus.h
        Include/PhysicsGrab/PhysicsGrabBus.h
        Include/PhysicsGrab/PhysicsGrabTypeIds.h
        Include/PhysicsGrab/NetworkPhysicsGrabBus.h
        Include/PhysicsGrab/NetworkPhysicsGrabComponentBus.h
    )
else()
    set(FILES
        Include/PhysicsGrab/PhysicsGrabComponentBus.h
        Include/PhysicsGrab/PhysicsGrabBus.h
        Include/PhysicsGrab/PhysicsGrabTypeIds.h
    )
endif()