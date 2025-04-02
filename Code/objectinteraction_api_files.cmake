find_path(FPC_INCLUDE_DIR
    NAMES FirstPersonController/FirstPersonControllerComponentBus.h
    PATHS
        "${LY_PROJECTS}/../../Gems/FirstPersonController/Code/Include"
        "${LY_PROJECTS}/../../Gems/FirstPersonController/1.0.0/Code/Include"
)

set(FILES
    Include/ObjectInteraction/ObjectInteractionBus.h
    Include/ObjectInteraction/ObjectInteractionComponentBus.h
    ${FPC_INCLUDE_DIR}/FirstPersonController/FirstPersonControllerComponentBus.h
)