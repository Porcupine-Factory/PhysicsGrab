#include "ObjectInteractionComponent.h"

#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzFramework/Physics/CollisionBus.h>
#include <AzFramework/Physics/NameConstants.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <AzFramework/Physics/SystemBus.h>
#include <System/PhysXSystem.h>

namespace ObjectInteraction
{
    using namespace StartingPointInput;

    void ObjectInteractionComponent::Reflect(AZ::ReflectContext* rc)
    {
        if (auto sc = azrtti_cast<AZ::SerializeContext*>(rc))
        {
            sc->Class<ObjectInteractionComponent, AZ::Component>()

                // Grab Input Binding Keys
                ->Field("Grab Input Key", &ObjectInteractionComponent::m_strGrab)
                ->Field("Grab Distance Input Key", &ObjectInteractionComponent::m_strGrabDistance)
                ->Field("Throw Input Key", &ObjectInteractionComponent::m_strThrow)
                ->Field("Rotate Enable Input Key", &ObjectInteractionComponent::m_strRotate)
                ->Field("Rotate Pitch Key", &ObjectInteractionComponent::m_strRotatePitch)
                ->Field("Rotate Yaw Key", &ObjectInteractionComponent::m_strRotateYaw)
                ->Field("Rotate Roll Key", &ObjectInteractionComponent::m_strRotateRoll)

                ->Field("GrabbingEntityId", &ObjectInteractionComponent::m_grabbingEntityId)
                ->Field("Enable Mesh Smoothing", &ObjectInteractionComponent::m_enableMeshSmoothing)
                ->Field("Grab Mesh Entity Name", &ObjectInteractionComponent::m_meshEntityName)
                #ifdef FIRST_PERSON_CONTROLLER
                ->Field("Freeze Character Rotation", &ObjectInteractionComponent::m_freezeCharacterRotation)
                #endif
                ->Field("Grab Enable Toggle", &ObjectInteractionComponent::m_grabEnableToggle)
                ->Field("Maintain Grab", &ObjectInteractionComponent::m_grabMaintained)
                ->Field("Kinematic While Grabbing", &ObjectInteractionComponent::m_kinematicWhileHeld)
                ->Field("Rotate Enable Toggle", &ObjectInteractionComponent::m_rotateEnableToggle)
                ->Field("Tidal Lock Grabbed Object", &ObjectInteractionComponent::m_tidalLock)
                ->Field("Sphere Cast Radius", &ObjectInteractionComponent::m_sphereCastRadius)
                ->Attribute(AZ::Edit::Attributes::Suffix, " " + Physics::NameConstants::GetLengthUnit())
                ->Field("Sphere Cast Distance", &ObjectInteractionComponent::m_sphereCastDistance)
                ->Attribute(AZ::Edit::Attributes::Suffix, " " + Physics::NameConstants::GetLengthUnit())
                ->Field("Default Grab Distance", &ObjectInteractionComponent::m_initialGrabDistance)
                ->Attribute(AZ::Edit::Attributes::Suffix, " " + Physics::NameConstants::GetLengthUnit())
                ->Field("Min Grab Distance", &ObjectInteractionComponent::m_minGrabDistance)
                ->Attribute(AZ::Edit::Attributes::Suffix, " " + Physics::NameConstants::GetLengthUnit())
                ->Field("Max Grab Distance", &ObjectInteractionComponent::m_maxGrabDistance)
                ->Attribute(AZ::Edit::Attributes::Suffix, " " + Physics::NameConstants::GetLengthUnit())
                ->Field("Grab Distance Speed", &ObjectInteractionComponent::m_grabDistanceSpeed)
                ->Attribute(AZ::Edit::Attributes::Suffix, " " + Physics::NameConstants::GetSpeedUnit())
                ->Field("Throw Impulse", &ObjectInteractionComponent::m_throwImpulse)
                ->Attribute(AZ::Edit::Attributes::Suffix, AZStd::string::format(" N%ss", Physics::NameConstants::GetInterpunct().c_str()))
                ->Field("Grab Response", &ObjectInteractionComponent::m_grabResponse)
                ->Attribute(
                    AZ::Edit::Attributes::Suffix,
                    AZStd::string::format(
                        " s%s%s",
                        Physics::NameConstants::GetSuperscriptMinus().c_str(),
                        Physics::NameConstants::GetSuperscriptOne().c_str()))
                ->Field("Kinematic Rotate Scale", &ObjectInteractionComponent::m_kinematicRotateScale)
                ->Field("Dynamic Rotate Scale", &ObjectInteractionComponent::m_dynamicRotateScale)
                ->Field("Angular Damping", &ObjectInteractionComponent::m_tempObjectAngularDamping)
                ->Field("Grabbed Object Collision Group", &ObjectInteractionComponent::m_grabbedCollisionGroupId)
                ->Field("Grabbed Object Temporary Collision Layer", &ObjectInteractionComponent::m_tempGrabbedCollisionLayer)
                ->Version(1);

            if (AZ::EditContext* ec = sc->GetEditContext())
            {
                using namespace AZ::Edit;
                ec->Class<ObjectInteractionComponent>("Object Interaction", "[Object Interaction Component]")
                    ->ClassElement(ClassElements::EditorData, "")
                    ->Attribute(Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))

                    ->DataElement(
                        0,
                        &ObjectInteractionComponent::m_grabbingEntityId,
                        "Grab Entity",
                        "Reference entity that interacts with objects. If left blank, Camera entity will be used by default.")
                    ->DataElement(
                        0,
                        &ObjectInteractionComponent::m_enableMeshSmoothing,
                        "Enable Mesh Smoothing",
                        "Enables smooth interpolation of the mesh transform for dynamic objects to reduce stuttering.")
                    ->DataElement(
                        0,
                        &ObjectInteractionComponent::m_meshEntityName,
                        "Mesh Mesh Entity Name",
                        "Name (or partial name) of the child entity to use for mesh interpolation (e.g., 'Grab Mesh').")

                    // Input Binding Keys
                    ->ClassElement(AZ::Edit::ClassElements::Group, "Input Bindings")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(nullptr, &ObjectInteractionComponent::m_strGrab, "Grab Key", "Grab interaction input binding")
                    ->DataElement(nullptr, &ObjectInteractionComponent::m_strGrabDistance, "Grab Distance Key", "Grab distance input binding")
                    ->DataElement(nullptr, &ObjectInteractionComponent::m_strThrow, "Throw Input Key", "Throw object input binding")
                    ->DataElement(nullptr, &ObjectInteractionComponent::m_strRotate, "Rotate Enable Key", "Enable rotate object input binding")
                    ->DataElement(nullptr, &ObjectInteractionComponent::m_strRotatePitch, "Rotate Pitch Key", "Rotate object about X axis input binding")
                    ->DataElement(nullptr, &ObjectInteractionComponent::m_strRotateYaw, "Rotate Yaw Key", "Rotate object about Z axis input binding")
                    ->DataElement(nullptr, &ObjectInteractionComponent::m_strRotateRoll, "Rotate Roll Key", "Rotate object about Y axis input binding")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Toggle Preferences")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    #ifdef FIRST_PERSON_CONTROLLER
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_freezeCharacterRotation,
                        "Freeze Character Rotation",
                        "Enables character controller rotation while in Rotate State.")
                    #endif
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_grabEnableToggle,
                        "Grab Enable Toggle",
                        "Determines whether pressing Grab Key toggles Grab mode. Disabling this requires the Grab key to be held to "
                        "maintain Grab mode.")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_grabMaintained,
                        "Maintain Grab",
                        "Grabbed Object remains held even if sphere cast no longer intersects it. This prevents the Grabbed Object from "
                        "flying off when quickly changing directions.")
                    ->DataElement(
                        nullptr, &ObjectInteractionComponent::m_kinematicWhileHeld, "Kinematic Grabbed Object", "Sets the grabbed object to kinematic.")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_rotateEnableToggle,
                        "Rotate Enable Toggle",
                        "Determines whether pressing Rotate Key toggles Rotate mode. Disabling this requires the Rotate key to be held to "
                        "maintain Rotate mode.")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_tidalLock,
                        "Tidal Lock Kinematic Object",
                        "Determines whether a Kinematic Object is tidal locked when being grabbed. This means that the object will always "
                        "face the Grabbing Entity in it's current relative rotation.")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Scaling Factors")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        nullptr, &ObjectInteractionComponent::m_throwImpulse, "Throw Impulse", "Linear Impulse scale applied when throwing grabbed object")
                    ->DataElement(
                        nullptr, &ObjectInteractionComponent::m_grabResponse, "Grab Response", "Linear velocity scale applied when holding grabbed object")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_kinematicRotateScale,
                        "Kinematic Rotate Scale",
                        "Rotation speed scale applied when rotating kinematic object")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_dynamicRotateScale,
                        "Dynamic Rotate Scale",
                        "Angular Velocity scale applied when rotating dynamic object")
                    ->DataElement(
                        nullptr, &ObjectInteractionComponent::m_tempObjectAngularDamping, "Angular Damping", "Angular Damping of Grabbed Object while Grabbing")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Sphere Cast Parameters")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_grabbedCollisionGroupId,
                        "Sphere Cast Collision Group",
                        "The collision group which will be used for detecting grabbable objects.")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_tempGrabbedCollisionLayer,
                        "Grabbed Object Temporary Collision Layer",
                        "The temporary collision layer assigned to the grabbed object while it is being grabbed/held.")
                    ->DataElement(nullptr, &ObjectInteractionComponent::m_sphereCastRadius, "Sphere Cast Radius", "Sphere Cast radius used for grabbing objects")
                    ->DataElement(
                        nullptr, &ObjectInteractionComponent::m_sphereCastDistance, "Sphere Cast Distance", "Sphere Cast distance along m_sphereCastDirection")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Grab Distance Parameters")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_initialGrabDistance,
                        "Default Grab Distance",
                        "Distance the grabbed object will default to when letting go of the object")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_minGrabDistance,
                        "Min Grab Distance",
                        "Minimum allowable grab distance. Grabbed object cannot get closer than this distance.")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_maxGrabDistance,
                        "Max Grab Distance",
                        "Maximum allowable grab distance. Grabbed object cannot get further than this distance.")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_grabDistanceSpeed,
                        "Grab Distance Speed",
                        "The speed at which you move the grabbed object closer or away.");
            }
        }

        if (auto bc = azrtti_cast<AZ::BehaviorContext*>(rc))
        {
            bc->EBus<ObjectInteractionNotificationBus>("GrabNotificationBus", "GrabComponentNotificationBus", "Notifications for Grab Component")
                ->Handler<ObjectInteractionNotificationHandler>();

            // Reflect the enum
            bc->Enum<static_cast<int>(ObjectInteractionStates::idleState)>("ObjectInteractionStates_IdleState")
                ->Enum<static_cast<int>(ObjectInteractionStates::checkState)>("ObjectInteractionStates_CheckState")
                ->Enum<static_cast<int>(ObjectInteractionStates::holdState)>("ObjectInteractionStates_HoldState")
                ->Enum<static_cast<int>(ObjectInteractionStates::rotateState)>("ObjectInteractionStates_RotateState")
                ->Enum<static_cast<int>(ObjectInteractionStates::throwState)>("ObjectInteractionStates_ThrowState");

            bc->EBus<ObjectInteractionComponentRequestBus>("ObjectInteractionComponentRequestBus")
                ->Attribute(AZ::Script::Attributes::Scope, AZ::Script::Attributes::ScopeFlags::Common)
                ->Attribute(AZ::Script::Attributes::Module, "interaction")
                ->Attribute(AZ::Script::Attributes::Category, "Object Interaction")
                ->Event("Get Grabbing EntityId", &ObjectInteractionComponentRequests::GetGrabbingEntityId)
                ->Event("Get Active Camera EntityId", &ObjectInteractionComponentRequests::GetActiveCameraEntityId)
                ->Event("Get Grabbed Object EntityId", &ObjectInteractionComponentRequests::GetGrabbedObjectEntityId)
                ->Event("Get Last Grabbed Object EntityId", &ObjectInteractionComponentRequests::GetLastGrabbedObjectEntityId)
                ->Event("Get Thrown Grabbed Object EntityId", &ObjectInteractionComponentRequests::GetThrownGrabbedObjectEntityId)
                ->Event("Set Thrown Grabbed Object EntityId", &ObjectInteractionComponentRequests::SetThrownGrabbedObjectEntityId)
                ->Event("Set Grabbing Entity", &ObjectInteractionComponentRequests::SetGrabbingEntity)
                ->Event("Get Grabbed Collision Group", &ObjectInteractionComponentRequests::GetGrabbedCollisionGroup)
                ->Event("Set Grabbed Collision Group", &ObjectInteractionComponentRequests::SetGrabbedCollisionGroup)
                ->Event("Get Current Grabbed Layer Name", &ObjectInteractionComponentRequests::GetCurrentGrabbedCollisionLayerName)
                ->Event("Set Current Grabbed Layer By Name", &ObjectInteractionComponentRequests::SetCurrentGrabbedCollisionLayerByName)
                ->Event("Get Current Grabbed Layer", &ObjectInteractionComponentRequests::GetCurrentGrabbedCollisionLayer)
                ->Event("Set Current Grabbed Layer", &ObjectInteractionComponentRequests::SetCurrentGrabbedCollisionLayer)
                ->Event("Get Previous Grabbed Layer Name", &ObjectInteractionComponentRequests::GetPrevGrabbedCollisionLayerName)
                ->Event("Set Previous Grabbed Layer Name By Name", &ObjectInteractionComponentRequests::SetPrevGrabbedCollisionLayerByName)
                ->Event("Get Previous Grabbed Layer", &ObjectInteractionComponentRequests::GetPrevGrabbedCollisionLayer)
                ->Event("Set Previous Grabbed Layer", &ObjectInteractionComponentRequests::SetPrevGrabbedCollisionLayer)
                ->Event("Get Temporary Grabbed Layer Name", &ObjectInteractionComponentRequests::GetTempGrabbedCollisionLayerName)
                ->Event("Set Temporary Grabbed Layer By Name", &ObjectInteractionComponentRequests::SetTempGrabbedCollisionLayerByName)
                ->Event("Get Temporary Grabbed Layer", &ObjectInteractionComponentRequests::GetTempGrabbedCollisionLayer)
                ->Event("Set Temporary Grabbed Layer", &ObjectInteractionComponentRequests::SetTempGrabbedCollisionLayer)
                ->Event("Get State String", &ObjectInteractionComponentRequests::GetStateString)
                ->Event("Get Is In Idle State", &ObjectInteractionComponentRequests::GetIsInIdleState)
                ->Event("Get Is In Check State", &ObjectInteractionComponentRequests::GetIsInCheckState)
                ->Event("Get Is In Held State", &ObjectInteractionComponentRequests::GetIsInHeldState)
                ->Event("Get Is In Rotate State", &ObjectInteractionComponentRequests::GetIsInRotateState)
                ->Event("Get Is In Throw State", &ObjectInteractionComponentRequests::GetIsInThrowState)
                ->Event("Get Object Sphere Cast Hit", &ObjectInteractionComponentRequests::GetObjectSphereCastHit)
                ->Event("Get Stay In Idle State", &ObjectInteractionComponentRequests::GetStayInIdleState)
                ->Event("Set Stay In Idle State", &ObjectInteractionComponentRequests::SetStayInIdleState)
                ->Event("Get Grab Enable Toggle", &ObjectInteractionComponentRequests::GetGrabEnableToggle)
                ->Event("Set Grab Enable Toggle", &ObjectInteractionComponentRequests::SetGrabEnableToggle)
                ->Event("Get Rotate Enable Toggle", &ObjectInteractionComponentRequests::GetRotateEnableToggle)
                ->Event("Set Rotate Enable Toggle", &ObjectInteractionComponentRequests::SetRotateEnableToggle)
                ->Event("Get Grab Key Value", &ObjectInteractionComponentRequests::GetGrabKeyValue)
                ->Event("Set Grab Key Value", &ObjectInteractionComponentRequests::SetGrabKeyValue)
                ->Event("Get Throw Key Value", &ObjectInteractionComponentRequests::GetThrowKeyValue)
                ->Event("Set Throw Key Value", &ObjectInteractionComponentRequests::SetThrowKeyValue)
                ->Event("Get Rotate Key Value", &ObjectInteractionComponentRequests::GetRotateKeyValue)
                ->Event("Set Rotate Key Value", &ObjectInteractionComponentRequests::SetRotateKeyValue)
                ->Event("Get Pitch Key Value", &ObjectInteractionComponentRequests::GetPitchKeyValue)
                ->Event("Set Pitch Key Value", &ObjectInteractionComponentRequests::SetPitchKeyValue)
                ->Event("Get Yaw Key Value", &ObjectInteractionComponentRequests::GetYawKeyValue)
                ->Event("Set Yaw Key Value", &ObjectInteractionComponentRequests::SetYawKeyValue)
                ->Event("Get Roll Key Value", &ObjectInteractionComponentRequests::GetRollKeyValue)
                ->Event("Set Roll Key Value", &ObjectInteractionComponentRequests::SetRollKeyValue)
                ->Event("Get Grab Distance Key Value", &ObjectInteractionComponentRequests::GetGrabbedDistanceKeyValue)
                ->Event("Set Grab Distance Key Value", &ObjectInteractionComponentRequests::SetGrabbedDistanceKeyValue)         
                ->Event("Get Grabbed Object Distance", &ObjectInteractionComponentRequests::GetGrabbedObjectDistance)
                ->Event("Set Grabbed Object Distance", &ObjectInteractionComponentRequests::SetGrabbedObjectDistance)
                ->Event("Get Minimum Grabbed Object Distance", &ObjectInteractionComponentRequests::GetMinGrabbedObjectDistance)
                ->Event("Set Minimum Grabbed Object Distance", &ObjectInteractionComponentRequests::SetMinGrabbedObjectDistance)
                ->Event("Get Maximum Grabbed Objectt Distance", &ObjectInteractionComponentRequests::GetMaxGrabbedObjectDistance)
                ->Event("Set Maximum Grabbed Object Distance", &ObjectInteractionComponentRequests::SetMaxGrabbedObjectDistance)
                ->Event("Get Initial Grabbed Object Distance", &ObjectInteractionComponentRequests::GetInitialGrabbedObjectDistance)
                ->Event("Set Initial Grabbed Objectt Distance", &ObjectInteractionComponentRequests::SetInitialGrabbedObjectDistance)
                ->Event("Get Grabbed Object Distance Speed", &ObjectInteractionComponentRequests::GetGrabbedObjectDistanceSpeed)
                ->Event("Set Grabbed Object Distance Speed", &ObjectInteractionComponentRequests::SetGrabbedObjectDistanceSpeed)
                ->Event("Get Grab Response", &ObjectInteractionComponentRequests::GetGrabResponse)
                ->Event("Set Grab Response", &ObjectInteractionComponentRequests::SetGrabResponse)
                ->Event("Get Dynamic Object Tidal Lock", &ObjectInteractionComponentRequests::GetDynamicTidalLock)
                ->Event("Set Dynamic Object Tidal Lock", &ObjectInteractionComponentRequests::SetDynamicTidalLock)
                ->Event("Get Kinematic Object Tidal Lock", &ObjectInteractionComponentRequests::GetKinematicTidalLock)
                ->Event("Set Kinematic Object Tidal Lock", &ObjectInteractionComponentRequests::SetKinematicTidalLock)
                ->Event("Get Object Tidal Lock", &ObjectInteractionComponentRequests::GetTidalLock)
                ->Event("Set Object Tidal Lock", &ObjectInteractionComponentRequests::SetTidalLock)
                ->Event("Get Grabbed Dynamic Object Rotation Scale", &ObjectInteractionComponentRequests::GetDynamicRotateScale)
                ->Event("Set Grabbed Dynamic Object Rotation Scale", &ObjectInteractionComponentRequests::SetDynamicRotateScale)
                ->Event("Get Grabbed Kinematic Object Rotation Scale", &ObjectInteractionComponentRequests::GetKinematicRotateScale)
                ->Event("Set Grabbed Kinematic Object Rotation Scale", &ObjectInteractionComponentRequests::SetKinematicRotateScale)
                ->Event("Get Grab Throw Impulse", &ObjectInteractionComponentRequests::GetThrowImpulse)
                ->Event("Set Grab Throw Impulse", &ObjectInteractionComponentRequests::SetThrowImpulse)
                ->Event("Get Grabbed Object Throw State Counter", &ObjectInteractionComponentRequests::GetGrabbedObjectThrowStateCounter)
                ->Event("Set Grabbed Object Throw State Counter", &ObjectInteractionComponentRequests::SetGrabbedObjectThrowStateCounter)
                ->Event("Get Grabbed Object Throw State Max Time", &ObjectInteractionComponentRequests::GetGrabbedObjectThrowStateTime)
                ->Event("Set Grabbed Object Throw State Max Time", &ObjectInteractionComponentRequests::SetGrabbedObjectThrowStateTime)
                ->Event("Get Grab Sphere Cast Radius", &ObjectInteractionComponentRequests::GetSphereCastRadius)
                ->Event("Set Grab Sphere Cast Radius", &ObjectInteractionComponentRequests::SetSphereCastRadius)
                ->Event("Get Grab Sphere Cast Distance", &ObjectInteractionComponentRequests::GetSphereCastDistance)
                ->Event("Set Grab Sphere Cast Distance", &ObjectInteractionComponentRequests::SetSphereCastDistance)
                ->Event("Get Grabbed Object Is Kinematic", &ObjectInteractionComponentRequests::GetGrabbedObjectKinematicElseDynamic)
                ->Event("Set Grabbed Object Is Kinematic", &ObjectInteractionComponentRequests::SetGrabbedObjectKinematicElseDynamic)
                ->Event("Get Current Grabbed Object Angular Damping", &ObjectInteractionComponentRequests::GetCurrentGrabbedObjectAngularDamping)
                ->Event("Set Current Grabbed Object Angular Damping", &ObjectInteractionComponentRequests::SetCurrentGrabbedObjectAngularDamping)
                ->Event("Get Previous Grabbed Object Angular Damping", &ObjectInteractionComponentRequests::GetPrevGrabbedObjectAngularDamping)
                ->Event("Set Previous Grabbed Object Angular Damping", &ObjectInteractionComponentRequests::SetPrevGrabbedObjectAngularDamping)
                ->Event("Get Temporary Grabbed Object Angular Damping", &ObjectInteractionComponentRequests::GetTempGrabbedObjectAngularDamping)
                ->Event("Set Temporary Grabbed Object Angular Damping", &ObjectInteractionComponentRequests::SetTempGrabbedObjectAngularDamping)
                ->Event("Get Initial Angular Velocity Zero", &ObjectInteractionComponentRequests::GetInitialAngularVelocityZero)
                ->Event("Set Initial Angular Velocity Zero", &ObjectInteractionComponentRequests::SetInitialAngularVelocityZero)
                ->Event("Force State Transition", &ObjectInteractionComponentRequests::ForceTransition)
                ->Event("Set Locked State Transition", &ObjectInteractionComponentRequests::SetStateLocked)
                ->Event("Get Locked State Transition", &ObjectInteractionComponentRequests::GetStateLocked)
                ->Event("GetGrabInputKey", &ObjectInteractionComponentRequests::GetGrabInputKey)
                ->Event("SetGrabInputKey", &ObjectInteractionComponentRequests::SetGrabInputKey)
                ->Event("GetThrowInputKey", &ObjectInteractionComponentRequests::GetThrowInputKey)
                ->Event("SetThrowInputKey", &ObjectInteractionComponentRequests::SetThrowInputKey)
                ->Event("GetRotateInputKey", &ObjectInteractionComponentRequests::GetRotateInputKey)
                ->Event("SetRotateInputKey", &ObjectInteractionComponentRequests::SetRotateInputKey)
                ->Event("GetRotatePitchInputKey", &ObjectInteractionComponentRequests::GetRotatePitchInputKey)
                ->Event("SetRotatePitchInputKey", &ObjectInteractionComponentRequests::SetRotatePitchInputKey)
                ->Event("GetRotateYawInputKey", &ObjectInteractionComponentRequests::GetRotateYawInputKey)
                ->Event("SetRotateYawInputKey", &ObjectInteractionComponentRequests::SetRotateYawInputKey)
                ->Event("GetRotateRollInputKey", &ObjectInteractionComponentRequests::GetRotateRollInputKey)
                ->Event("SetRotateRollInputKey", &ObjectInteractionComponentRequests::SetRotateRollInputKey)
                ->Event("GetGrabDistanceInputKey", &ObjectInteractionComponentRequests::GetGrabDistanceInputKey)
                ->Event("SetGrabDistanceInputKey", &ObjectInteractionComponentRequests::SetGrabDistanceInputKey)
                //->Event("GetMeshEntityId", &ObjectInteractionComponentRequests::GetMeshEntityId)
                //->Event("SetMeshEntityId", &ObjectInteractionComponentRequests::SetMeshEntityId)
                ->Event("GetMeshEntityName", &ObjectInteractionComponentRequests::GetMeshEntityName)
                ->Event("SetMeshEntityName", &ObjectInteractionComponentRequests::SetMeshEntityName);

            bc->Class<ObjectInteractionComponent>()->RequestBus("ObjectInteractionComponentRequestBus");
        }
    }
    void ObjectInteractionComponent::Activate()
    {
        m_grabEventId = StartingPointInput::InputEventNotificationId(m_strGrab.c_str());
        InputEventNotificationBus::MultiHandler::BusConnect(m_grabEventId);

        m_grabDistanceEventId = StartingPointInput::InputEventNotificationId(m_strGrabDistance.c_str());
        InputEventNotificationBus::MultiHandler::BusConnect(m_grabDistanceEventId);

        m_throwEventId = StartingPointInput::InputEventNotificationId(m_strThrow.c_str());
        InputEventNotificationBus::MultiHandler::BusConnect(m_throwEventId);

        m_rotateEventId = StartingPointInput::InputEventNotificationId(m_strRotate.c_str());
        InputEventNotificationBus::MultiHandler::BusConnect(m_rotateEventId);

        m_rotatePitchEventId = StartingPointInput::InputEventNotificationId(m_strRotatePitch.c_str());
        InputEventNotificationBus::MultiHandler::BusConnect(m_rotatePitchEventId);

        m_rotateYawEventId = StartingPointInput::InputEventNotificationId(m_strRotateYaw.c_str());
        InputEventNotificationBus::MultiHandler::BusConnect(m_rotateYawEventId);

        m_rotateRollEventId = StartingPointInput::InputEventNotificationId(m_strRotateRoll.c_str());
        InputEventNotificationBus::MultiHandler::BusConnect(m_rotateRollEventId);

        Camera::CameraNotificationBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();

        Physics::CollisionRequestBus::BroadcastResult(
            m_grabbedCollisionGroup, &Physics::CollisionRequests::GetCollisionGroupById, m_grabbedCollisionGroupId);

        // Connect to physics simulation start event
        Physics::DefaultWorldBus::BroadcastResult(m_attachedSceneHandle, &Physics::DefaultWorldRequests::GetDefaultSceneHandle);
        if (m_attachedSceneHandle == AzPhysics::InvalidSceneHandle)
        {
            AZ_Error("Object Interaction Component", false, "Failed to retrieve default scene.");
            return;
        }

        // Register m_sceneSimulationStartHandler to listen for the OnSceneSimulationStart event, which 
        // is triggered at the start of each physics simulation step
        m_sceneSimulationStartHandler = AzPhysics::SceneEvents::OnSceneSimulationStartHandler(
            [this]([[maybe_unused]] AzPhysics::SceneHandle sceneHandle, float fixedDeltaTime)
            {
                OnSceneSimulationStart(fixedDeltaTime);
            },
            aznumeric_cast<int32_t>(AzPhysics::SceneEvents::PhysicsStartFinishSimulationPriority::Physics));

        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        if (sceneInterface != nullptr)
        {
            sceneInterface->RegisterSceneSimulationStartHandler(m_attachedSceneHandle, m_sceneSimulationStartHandler);
        }

        // Connect the handler to the request bus
        ObjectInteractionComponentRequestBus::Handler::BusConnect(GetEntityId());

        // Delaying the assignment of Grabbing Entity to OnEntityActivated so the Entity is activated and ready
        AZ::EntityBus::Handler::BusConnect(m_grabbingEntityId);

        // Initialize m_grabDistance to the editor-specified m_initialGrabDistance
        m_grabDistance = m_initialGrabDistance;
    }

    // Called at the beginning of each physics tick
    void ObjectInteractionComponent::OnSceneSimulationStart(float physicsTimestep)
    {
        // Update physics timestep
        m_physicsTimestep = physicsTimestep;

        // Store previous physics transform
        if (m_lastGrabbedObjectEntityId.IsValid() && !m_isObjectKinematic && m_enableMeshSmoothing)
        {
            m_prevPhysicsTransform = m_currentPhysicsTransform;
            AZ::TransformBus::EventResult(m_currentPhysicsTransform, m_lastGrabbedObjectEntityId, &AZ::TransformInterface::GetWorldTM);
        }

        // Reset time accumulator
        m_physicsTimeAccumulator = 0.0f;
    }

    void ObjectInteractionComponent::OnEntityActivated([[maybe_unused]] const AZ::EntityId& entityId)
    {
        AZ::EntityBus::Handler::BusDisconnect();

        if (m_grabbingEntityId.IsValid())
        {
            m_grabbingEntityPtr = GetEntityPtr(entityId);
        }
    }

    void ObjectInteractionComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        InputEventNotificationBus::MultiHandler::BusDisconnect();
        ObjectInteractionComponentRequestBus::Handler::BusDisconnect();
        Camera::CameraNotificationBus::Handler::BusDisconnect();

        m_attachedSceneHandle = AzPhysics::InvalidSceneHandle;
        m_sceneSimulationStartHandler.Disconnect();
        m_meshEntityPtr = nullptr;
    }

    void ObjectInteractionComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("InputConfigurationService"));
        required.push_back(AZ_CRC_CE("TransformService"));
    }

    void ObjectInteractionComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("GrabService"));
    }

    void ObjectInteractionComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("GrabService"));
        incompatible.push_back(AZ_CRC_CE("InputService"));
    }

    void ObjectInteractionComponent::GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        dependent.push_back(AZ_CRC_CE("FirstPersonControllerService"));
    }

    void ObjectInteractionComponent::OnCameraAdded(const AZ::EntityId& cameraId)
    {
        if (!m_grabbingEntityId.IsValid())
        {
            m_grabbingEntityId = cameraId;
            m_grabbingEntityPtr = GetEntityPtr(cameraId);
        }
    }

    AZ::Entity* ObjectInteractionComponent::GetActiveCameraEntityPtr() const
    {
        AZ::EntityId activeCameraId;
        Camera::CameraSystemRequestBus::BroadcastResult(activeCameraId, &Camera::CameraSystemRequestBus::Events::GetActiveCamera);

        auto ca = AZ::Interface<AZ::ComponentApplicationRequests>::Get();
        return ca->FindEntity(activeCameraId);
    }

    // Recieve the input event in OnPressed method
    void ObjectInteractionComponent::OnPressed(float value)
    {
        const InputEventNotificationId* inputId = InputEventNotificationBus::GetCurrentBusId();
        if (inputId == nullptr)
        {
            return;
        }

        if (*inputId == m_grabEventId)
        {
            m_grabKeyValue = value;
            // AZ_Printf("Player", "Grab value %f", value);
        }

        if (*inputId == m_grabDistanceEventId)
        {
            m_grabDistanceKeyValue = value;
            // AZ_Printf("Object", "Grab Distance value %f", value);
        }

        if (*inputId == m_throwEventId)
        {
            m_throwKeyValue = value;
            // AZ_Printf("Player", "Throw value %f", value);
        }

        if (*inputId == m_rotateEventId)
        {
            m_rotateKeyValue = value;
            // AZ_Printf("Player", "rotate value %f", value);
        }

        if (*inputId == m_rotatePitchEventId)
        {
            m_pitchKeyValue = value;
            // AZ_Printf("Object", "Grab Object pitch value %f", value);
        }

        if (*inputId == m_rotateYawEventId)
        {
            m_yawKeyValue = value;
            // AZ_Printf("Object", "Grab Object yaw value %f", value);
        }

        if (*inputId == m_rotateRollEventId)
        {
            m_rollKeyValue = value;
            // AZ_Printf("Object", "Grab Object roll value %f", value);
        }
    }

    // Recieve the input event in OnReleased method
    void ObjectInteractionComponent::OnReleased(float value)
    {
        const InputEventNotificationId* inputId = InputEventNotificationBus::GetCurrentBusId();
        if (inputId == nullptr)
        {
            return;
        }
        if (*inputId == m_grabEventId)
        {
            m_grabKeyValue = value;
            // AZ_Printf("Player", "Grab released value %f", value);
        }
        if (*inputId == m_grabDistanceEventId)
        {
            m_grabDistanceKeyValue = value;
            // AZ_Printf("Player", "Grab Distance released value %f", value);
        }
        if (*inputId == m_throwEventId)
        {
            m_throwKeyValue = value;
            // AZ_Printf("Player", "Throw released value %f", value);
        }
        if (*inputId == m_rotateEventId)
        {
            m_rotateKeyValue = value;
            // AZ_Printf("Player", "Throw released value %f", value);
        }

        if (*inputId == m_rotatePitchEventId)
        {
            m_pitchKeyValue = value;
            // AZ_Printf("Object", "Grab Object pitch released value %f", value);
        }

        if (*inputId == m_rotateYawEventId)
        {
            m_yawKeyValue = value;
            // AZ_Printf("Object", "Grab Object yaw released value %f", value);
        }

        if (*inputId == m_rotateRollEventId)
        {
            m_rollKeyValue = value;
            // AZ_Printf("Object", "Grab Object roll released value %f", value);
        }
    }

    void ObjectInteractionComponent::OnHeld(float value)
    {
        const InputEventNotificationId* inputId = InputEventNotificationBus::GetCurrentBusId();
        if (inputId == nullptr)
        {
            return;
        }
        if (*inputId == m_throwEventId)
        {
            m_throwKeyValue = value;
            // AZ_Printf("Player", "Throw held value %f", value);
        }
        
        if (*inputId == m_rotatePitchEventId)
        {
            m_pitchKeyValue = value;
            // AZ_Printf("Object", "Grab Object pitch held value %f", value);
        }
        else if (*inputId == m_rotateYawEventId)
        {
            m_yawKeyValue = value;
            // AZ_Printf("Object", "Grab Object yaw held value %f", value);
        }
        else if (*inputId == m_rotateRollEventId)
        {
            m_rollKeyValue = value;
            // AZ_Printf("Object", "Grab Object roll held value %f", value);
        }
    }

    void ObjectInteractionComponent::ProcessStates(const float& deltaTime)
    {
        switch(m_state)
        {
            case ObjectInteractionStates::idleState:
                IdleState();
                break;
            case ObjectInteractionStates::checkState:
                CheckForObjectsState();
                break;
            case ObjectInteractionStates::holdState:
                HoldObjectState();
                break;
            case ObjectInteractionStates::rotateState:
                RotateObjectState();
                break;
            case ObjectInteractionStates::throwState:
                ThrowObjectState(deltaTime);
                break;
            default:
                m_state = ObjectInteractionStates::idleState;
                IdleState();
        }

        m_prevGrabKeyValue = m_grabKeyValue;
        m_prevRotateKeyValue = m_rotateKeyValue;
    }

    void ObjectInteractionComponent::OnTick(float deltaTime, AZ::ScriptTimePoint)
    {
        ProcessStates(deltaTime);
        if (m_enableMeshSmoothing)
        {
            InterpolateMeshTransform(deltaTime);
        }
    }
    
    // Smoothly update the visual transform of m_meshEntityPtr based on physics transforms
    void ObjectInteractionComponent::InterpolateMeshTransform(float deltaTime)
    {
        if (!m_lastGrabbedObjectEntityId.IsValid() || !m_meshEntityPtr || m_isObjectKinematic)
        {
            return;
        }

        // Update time accumulator
        m_physicsTimeAccumulator += deltaTime;

        // Calculate interpolation factor
        float alpha = AZ::GetClamp(m_physicsTimeAccumulator / m_physicsTimestep, 0.0f, 1.0f);

        // Interpolate position
        AZ::Vector3 interpolatedPosition = m_prevPhysicsTransform.GetTranslation().Lerp(m_currentPhysicsTransform.GetTranslation(), alpha);

        // Interpolate rotation
        AZ::Quaternion interpolatedRotation = m_prevPhysicsTransform.GetRotation().Slerp(m_currentPhysicsTransform.GetRotation(), alpha);

        // Create interpolated transform
        AZ::Transform interpolatedTransform = AZ::Transform::CreateFromQuaternionAndTranslation(interpolatedRotation, interpolatedPosition);

        // Update mesh entity transform
        AZ::TransformBus::Event(m_meshEntityPtr->GetId(), &AZ::TransformInterface::SetWorldTM, interpolatedTransform);
    }

    void ObjectInteractionComponent::IdleState()
    {
        if ((m_forceTransition && m_targetState == ObjectInteractionStates::checkState) ||
            (!m_isStateLocked && m_prevGrabKeyValue == 0.f && m_grabKeyValue != 0.f && !m_stayInIdleState))
        {
            m_state = ObjectInteractionStates::checkState;
            m_forceTransition = false;
        }
    }

    void ObjectInteractionComponent::CheckForObjectsState()
    {
        CheckForObjects();
        // Check if sphere cast hits a valid object before transitioning to holdState.
        // Other conditionals allow forced state transition to bypass inputs with m_forceTransition, or 
        // prevent state transition with m_isStateLocked
        if ((m_forceTransition && m_targetState == ObjectInteractionStates::holdState && m_objectSphereCastHit) ||
            (!m_isStateLocked && m_objectSphereCastHit))
        {
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnHoldStart);
            m_lastEntityRotation = GetEntity()->GetTransform()->GetWorldRotation();

            // Check if Grabbed Object is a Dynamic Rigid Body when first interacting with it
            m_isInitialObjectKinematic = GetGrabbedObjectKinematicElseDynamic();
            
            // Store initial collision layer
            m_prevGrabbedCollisionLayer = GetCurrentGrabbedCollisionLayer();
            
            // Set Object Current Layer variable to Temp Layer
            SetCurrentGrabbedCollisionLayer(m_tempGrabbedCollisionLayer);

            // Set Grabbed Object as Kinematic Rigid Body if set to be kinematic while held
            if (m_kinematicWhileHeld)
            {
                SetGrabbedObjectKinematicElseDynamic(true);
                m_isObjectKinematic = true;
            }
            // Set Grabbed Object as Dynamic Rigid Body if set to be dynamic while held
            else
            {
                SetGrabbedObjectKinematicElseDynamic(false);
                m_isObjectKinematic = false;
            }
           
            // Store object's original Angular Damping value
            m_prevObjectAngularDamping = GetCurrentGrabbedObjectAngularDamping();

            // Initialize physics transforms for dynamic objects
            if (!m_isObjectKinematic && m_enableMeshSmoothing)
            {
                AZ::TransformBus::EventResult(m_prevPhysicsTransform, m_lastGrabbedObjectEntityId, &AZ::TransformInterface::GetWorldTM);
                m_currentPhysicsTransform = m_prevPhysicsTransform;
                m_physicsTimeAccumulator = 0.0f;
            }

            // Find child entity with name containing m_meshEntityName
            m_meshEntityPtr = nullptr;
            if (m_enableMeshSmoothing && !m_isObjectKinematic && !m_meshEntityName.empty())
            {
                AZ::Entity* grabbedEntity = GetEntityPtr(m_lastGrabbedObjectEntityId);
                if (grabbedEntity)
                {
                    AZStd::vector<AZ::EntityId> children;
                    AZ::TransformBus::EventResult(children, m_lastGrabbedObjectEntityId, &AZ::TransformInterface::GetChildren);
                    for (const AZ::EntityId& childId : children)
                    {
                        AZ::Entity* childEntity = GetEntityPtr(childId);
                        if (childEntity && childEntity->GetName().find(m_meshEntityName) != AZStd::string::npos)
                        {
                            m_meshEntityPtr = childEntity;
                            break;
                        }
                    }
                }
            }

            // Fallback to grabbed object if no matching child found or smoothing disabled
            if (!m_meshEntityPtr)
            {
                m_meshEntityPtr = GetEntityPtr(m_lastGrabbedObjectEntityId);
            }

            m_state = ObjectInteractionStates::holdState;
            // Broadcast a grab start notification event
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnHoldStart);

            // Set angular velocity to 0 when first picking up a dynamic rigid body object
            if (!m_kinematicWhileHeld && m_initialAngularVelocityZero)
            {
                SetGrabbedObjectAngularVelocity(AZ::Vector3::CreateZero());
            }
            m_forceTransition = false;
        }
        // Go back to idleState if grab key is not pressed
        // Other conditionals allow forced state transition to bypass inputs with m_forceTransition, or 
        // prevent state transition with m_isStateLocked
        else if (
            (m_forceTransition && m_targetState == ObjectInteractionStates::idleState) ||
            (!m_isStateLocked && !(m_prevGrabKeyValue == 0.f && m_grabKeyValue != 0.f)))
        {
            m_state = ObjectInteractionStates::idleState;
            m_forceTransition = false;
        }
        else
        {
            m_state = ObjectInteractionStates::checkState;
        }
    }

    void ObjectInteractionComponent::HoldObjectState()
    {
        if (!m_grabMaintained)
        {
            CheckForObjects();
        }
        // Drop the object and go back to idle state if sphere cast doesn't hit
        // Other conditionals allow forced state transition to bypass inputs with m_forceTransition, or 
        // prevent state transition with m_isStateLocked
        if ((m_forceTransition && m_targetState == ObjectInteractionStates::idleState) || 
            (!m_isStateLocked && !m_objectSphereCastHit))
        {
            m_state = ObjectInteractionStates::idleState;
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnHoldStop);
            m_forceTransition = false;
            m_meshEntityPtr = nullptr;
            return;
        }

        HoldObject();

        // Go back to idle state if grab key is pressed again because we want to stop holding the 
        // object on the second key press. Other conditionals allow forced state transition to bypass 
        // inputs with m_forceTransition, or prevent state transition with m_isStateLocked
        if ((m_forceTransition && m_targetState == ObjectInteractionStates::idleState) ||
            (!m_isStateLocked &&
             ((m_grabEnableToggle && m_prevGrabKeyValue == 0.f && m_grabKeyValue != 0.f) ||
              (!m_grabEnableToggle && m_grabKeyValue == 0.f))))
        {
            // Reset current grabbed distance to m_initialGrabDistance if Grab key is not pressed
            m_grabDistance = m_initialGrabDistance;

            // Set Object Current Layer variable back to initial layer if Grab Key is not pressed
            SetCurrentGrabbedCollisionLayer(m_prevGrabbedCollisionLayer);
            
            // Set Angular Damping back to original value if Grab Key is not pressed
            SetCurrentGrabbedObjectAngularDamping(m_prevObjectAngularDamping);

            // Set Grabbed Object back to Dynamic Rigid Body if previously dynamic
            if (!m_isInitialObjectKinematic)
            {
                SetGrabbedObjectKinematicElseDynamic(false);
                m_isObjectKinematic = false;
            }
            // Set Grabbed Object back to Kinematic Rigid Body if previously kinematic
            else if (m_isInitialObjectKinematic)
            {
                SetGrabbedObjectKinematicElseDynamic(true);
                m_isObjectKinematic = true;
            }

            m_objectSphereCastHit = false;
            m_meshEntityPtr = nullptr;

            m_state = ObjectInteractionStates::idleState;
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnHoldStop);
            m_forceTransition = false;
        }
        // Enter Rotate State if rotate key is pressed.
        // Other conditionals allow forced state transition to 
        // bypass inputs with m_forceTransition, or prevent state transition with m_isStateLocked
        else if (
            (m_forceTransition && m_targetState == ObjectInteractionStates::rotateState) ||
            (!m_isStateLocked &&
             ((m_rotateEnableToggle && m_prevRotateKeyValue == 0.f && m_rotateKeyValue != 0.f) ||
              (!m_rotateEnableToggle && m_rotateKeyValue != 0.f))))
        {
            // Store object's original Angular Damping value before rotating
            m_prevObjectAngularDamping = GetCurrentGrabbedObjectAngularDamping();
            // Set new Angular Damping before rotating object
            SetCurrentGrabbedObjectAngularDamping(m_tempObjectAngularDamping);

            m_state = ObjectInteractionStates::rotateState;
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnRotateStart);
            m_forceTransition = false;
        }
        // Enter throw state if throw key is pressed.
        // Other conditionals allow forced state transition to 
        // bypass inputs with m_forceTransition, or prevent state transition with m_isStateLocked
        else if (
            (m_forceTransition && m_targetState == ObjectInteractionStates::throwState && !m_isInitialObjectKinematic) ||
            (!m_isStateLocked && m_throwKeyValue != 0.f && !m_isInitialObjectKinematic))
        {
            // Start throw counter
            m_throwStateCounter = m_throwStateMaxTime;

            // Set Kinematic Rigid Body to dynamic if it was held as kinematic
            if (m_isObjectKinematic)
            {
                SetGrabbedObjectKinematicElseDynamic(false);
                m_isObjectKinematic = false;
            }

            m_objectSphereCastHit = false;
            m_meshEntityPtr = nullptr;

            m_state = ObjectInteractionStates::throwState;
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnHoldStop);
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnThrowStart);
            m_forceTransition = false;
        }
        else
        {
            m_state = ObjectInteractionStates::holdState;
        }
    }

    void ObjectInteractionComponent::RotateObjectState()
    {
        if (!m_grabMaintained)
        {
            CheckForObjects();
        }
        if ((m_forceTransition && m_targetState == ObjectInteractionStates::idleState) || (!m_isStateLocked && !m_objectSphereCastHit))
        {
            m_state = ObjectInteractionStates::idleState;
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnRotateStop);
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnHoldStop);
            m_forceTransition = false;
            m_meshEntityPtr = nullptr;
            return;
        }

        HoldObject();

#ifdef FIRST_PERSON_CONTROLLER
        FreezeCharacterRotation();
#endif

        RotateObject();
        // Go back to hold state if rotate key is pressed again because we want to stop rotating the 
        // object on the second key press. Other conditionals allow forced state transition to 
        // bypass inputs with m_forceTransition, or prevent state transition with m_isStateLocked
        if ((m_forceTransition && m_targetState == ObjectInteractionStates::holdState) ||
            (!m_isStateLocked &&
             ((m_rotateEnableToggle && m_prevRotateKeyValue == 0.f && m_rotateKeyValue != 0.f) ||
              (!m_rotateEnableToggle && m_rotateKeyValue == 0.f))))
        {
            // Set Angular Damping back to original value when no longer rotating
            SetCurrentGrabbedObjectAngularDamping(m_prevObjectAngularDamping);
            // Set Angular Velocity to zero when no longer rotating
            SetGrabbedObjectAngularVelocity(AZ::Vector3::CreateZero());

            m_state = ObjectInteractionStates::holdState;
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnRotateStop);
            m_forceTransition = false;
        }
        // Go back to idle state if grab key is pressed again because we want to stop holding the 
        // object on the second key press. Other conditionals allow forced state transition to 
        // bypass inputs with m_forceTransition, or prevent state transition with m_isStateLocked
        else if (
            (m_forceTransition && m_targetState == ObjectInteractionStates::idleState) ||
            (!m_isStateLocked &&
             ((m_grabEnableToggle && m_prevGrabKeyValue == 0.f && m_grabKeyValue != 0.f) ||
              (!m_grabEnableToggle && m_prevGrabKeyValue == 0.f))))
        {
            // Set Angular Damping back to original value
            SetCurrentGrabbedObjectAngularDamping(m_prevObjectAngularDamping);
            // Set Angular Velocity to zero when no longer rotating
            SetGrabbedObjectAngularVelocity(AZ::Vector3::CreateZero());

            // Reset current grabbed distance to m_initialGrabDistance if grab key is not pressed
            m_grabDistance = m_initialGrabDistance;
            
            // Set Object Current Layer variable back to initial layer if Grab Key is not pressed
            SetCurrentGrabbedCollisionLayer(m_prevGrabbedCollisionLayer);
            
            // Set Angular Damping back to original value if Grab Key is not pressed
            SetCurrentGrabbedObjectAngularDamping(m_prevObjectAngularDamping);

            // Set Grabbed Object back to Dynamic Rigid Body if previously dynamic
            if (!m_isInitialObjectKinematic)
            {
                SetGrabbedObjectKinematicElseDynamic(false);
                m_isObjectKinematic = false;
            }
            // Set Grabbed Object back to Kinematic Rigid Body if previously kinematic
            else if (m_isInitialObjectKinematic)
            {
                SetGrabbedObjectKinematicElseDynamic(true);
                m_isObjectKinematic = true;
            }

            m_objectSphereCastHit = false;
            m_meshEntityPtr = nullptr;

            m_state = ObjectInteractionStates::idleState;
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnRotateStop);
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnHoldStop);
            m_forceTransition = false;
        }
        // Transition to throwState if Throw key is pressed
        // Other conditionals allow forced state transition to bypass inputs with 
        // m_forceTransition, or prevent state transition with m_isStateLocked
        else if (
            (m_forceTransition && m_targetState == ObjectInteractionStates::throwState && !m_isInitialObjectKinematic) ||
            (!m_isStateLocked && m_throwKeyValue != 0.f && !m_isInitialObjectKinematic))
        {
            // Set Angular Damping back to original value
            SetCurrentGrabbedObjectAngularDamping(m_prevObjectAngularDamping);
            // Set Angular Velocity to zero when no longer rotating
            SetGrabbedObjectAngularVelocity(AZ::Vector3::CreateZero());

            m_throwStateCounter = m_throwStateMaxTime;

            // Set Kinematic Rigid Body to dynamic if it was held as kinematic
            if (m_isObjectKinematic)
            {
                SetGrabbedObjectKinematicElseDynamic(false);
                m_isObjectKinematic = false;
            }

            m_objectSphereCastHit = false;
            m_meshEntityPtr = nullptr;

            m_state = ObjectInteractionStates::throwState;
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnRotateStop);
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnHoldStop);
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnThrowStart);
            m_forceTransition = false;
        }
        else
        {
            m_state = ObjectInteractionStates::rotateState;
        }
    }

    void ObjectInteractionComponent::ThrowObjectState(const float &deltaTime)
    {
        // ThrowObject() is only executed once. If setting m_throwStateCounter value via ebus, it 
        // is recommended to assign a value equal to m_throwStateMaxTime in order to properly execute ThrowObject()
        if (m_throwStateCounter == m_throwStateMaxTime)
        {
            ThrowObject();
        }

        m_throwStateCounter -= deltaTime;

        // Escape from the throw state if the thrown grabbed object is more than the distance of m_sphereCastDistance away
        if (m_grabReference.GetTranslation().GetDistance(
                GetEntityPtr(m_thrownGrabbedObjectEntityId)->GetTransform()->GetWorldTM().GetTranslation()) > m_sphereCastDistance)
        {
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnThrowStop);
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnMaxThrowDistance);
            m_state = ObjectInteractionStates::idleState;
        }
        // Escape from the throw state if grabbed object is in throw state longer than m_throwStateMaxTime
        else if (m_throwStateCounter <= 0.f)
        {
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnThrowStop);
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnThrowStateCounterZero);
            m_state = ObjectInteractionStates::idleState;
        }
        else
        {
            m_state = ObjectInteractionStates::throwState;
        }
    }

    // Perform a spherecast query to check if colliding with a grabbable object, then assign the first returned hit to
    // m_grabbedObjectEntityId
    void ObjectInteractionComponent::CheckForObjects()
    {
        // Get forward vector relative to the grabbing entity's transform
        m_forwardVector = m_grabbingEntityPtr->GetTransform()->GetWorldTM().GetBasisY();

        // Get our grabbing entity's world transform
        m_grabbingEntityTransform = m_grabbingEntityPtr->GetTransform()->GetWorldTM();

        // Perform a spherecast query to check if colliding with object
        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();

        AzPhysics::ShapeCastRequest request = AzPhysics::ShapeCastRequestHelpers::CreateSphereCastRequest(
            m_sphereCastRadius,
            m_grabbingEntityTransform,
            m_forwardVector,
            m_sphereCastDistance,
            AzPhysics::SceneQuery::QueryType::StaticAndDynamic,
            m_grabbedCollisionGroup,
            nullptr);

        AzPhysics::SceneHandle sceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        AzPhysics::SceneQueryHits hits = sceneInterface->QueryScene(sceneHandle, &request);

        const bool prevObjectSphereCastHit = m_objectSphereCastHit;
        m_objectSphereCastHit = hits ? true : false;

        // Prevents Grabbing new object if currently grabbing
        if (!prevObjectSphereCastHit && m_objectSphereCastHit)
        {
            // Takes first hit from spherecast query vector, and assigns this EntityID to m_grabbedObjectEntityId
            m_grabbedObjectEntityId = hits.m_hits.at(0).m_entityId;
            m_lastGrabbedObjectEntityId = m_grabbedObjectEntityId;
        }
    }

    // Hold and move object using physics or translation, based on object's 
    // starting Rigid Body type, or if KinematicWhileHeld is enabled
    void ObjectInteractionComponent::HoldObject()
    {
        // Grab distance value depends on whether grab distance input key is ignored via SetGrabbedDistanceKeyValue()
        const float grabDistanceValue = m_ignoreGrabDistanceKeyInputValue ? m_grabDistanceKeyValue : m_combinedGrabDistance;
        
        // Changes distance between Grabbing Entity and Grabbed object. Minimum and
        // maximum grab distances determined by m_minGrabDistance and m_maxGrabDistance, respectively
        m_grabDistance =
            AZ::GetClamp(m_grabDistance + ((grabDistanceValue * 0.01f) * m_grabDistanceSpeed), m_minGrabDistance, m_maxGrabDistance);

        // Get forward vector relative to the grabbing entity's transform
        m_forwardVector = m_grabbingEntityPtr->GetTransform()->GetWorldTM().GetBasisY();
        
        // Creates a reference point for the Grabbed Object translation in front of the Grabbing Entity
        m_grabReference = m_grabbingEntityPtr->GetTransform()->GetWorldTM();
        m_grabReference.SetTranslation(
            m_grabbingEntityPtr->GetTransform()->GetWorldTM().GetTranslation() + m_forwardVector * m_grabDistance);

        m_grabbedObjectTranslation = GetEntityPtr(m_lastGrabbedObjectEntityId)->GetTransform()->GetWorldTM().GetTranslation();

        // Move the object using Translation (Transform) if it is a Kinematic Rigid Body
        if (m_isObjectKinematic)
        {
            // Move object by setting its Translation
            AZ::TransformBus::Event(
                m_lastGrabbedObjectEntityId, &AZ::TransformInterface::SetWorldTranslation, m_grabReference.GetTranslation());
            
            // If object is NOT in rotate state, couple the grabbed entity's rotation to 
            // the controlling entity's local z rotation (causing object to face controlling entity)
            if (m_tidalLock && m_kinematicTidalLock)
            {
                TidalLock();
            }
            // Update mesh entity transform if smoothing is disabled
            if (!m_enableMeshSmoothing && m_meshEntityPtr && m_meshEntityPtr != GetEntityPtr(m_lastGrabbedObjectEntityId))
            {
                AZ::TransformBus::Event(
                    m_meshEntityPtr->GetId(),
                    &AZ::TransformInterface::SetWorldTM,
                    GetEntityPtr(m_lastGrabbedObjectEntityId)->GetTransform()->GetWorldTM());
            }
        }
        // Move the object using SetLinearVelocity (PhysX) if it is a Dynamic Rigid Body
        else
        {
            // Subtract object's translation from our reference position, which gives you a vector pointing from 
            // the object to the reference. Then apply a linear velocity to move the object toward the reference
            Physics::RigidBodyRequestBus::Event(
                m_lastGrabbedObjectEntityId,
                &Physics::RigidBodyRequests::SetLinearVelocity,
                (m_grabReference.GetTranslation() - m_grabbedObjectTranslation) * m_grabResponse);

            // If object is NOT in rotate state, couple the grabbed entity's rotation to the controlling entity's local z rotation
            if (m_tidalLock && m_dynamicTidalLock)
            {
                Physics::RigidBodyRequestBus::Event(m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::DisablePhysics);
                TidalLock();
                Physics::RigidBodyRequestBus::Event(m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::EnablePhysics);
            }

            // Update current physics transform for interpolation
            if (m_enableMeshSmoothing)
            {
                AZ::TransformBus::EventResult(m_currentPhysicsTransform, m_lastGrabbedObjectEntityId, &AZ::TransformInterface::GetWorldTM);
            }
            // Update mesh entity transform if smoothing is disabled
            else if (m_meshEntityPtr && m_meshEntityPtr != GetEntityPtr(m_lastGrabbedObjectEntityId))
            {
                AZ::TransformBus::Event(
                    m_meshEntityPtr->GetId(),
                    &AZ::TransformInterface::SetWorldTM,
                    GetEntityPtr(m_lastGrabbedObjectEntityId)->GetTransform()->GetWorldTM());
            }
        }
    }

    // Rotate object using physics or transforms, based on object's starting 
    // Rigid Body type, or if KinematicWhileHeld is enabled.
    void ObjectInteractionComponent::RotateObject()
    {
        // Get right vector relative to the grabbing entity's transform
        m_rightVector = m_grabbingEntityPtr->GetTransform()->GetWorldTM().GetBasisX();
        
        // Get up vector relative to the grabbing entity's transform
        m_upVector = m_grabbingEntityPtr->GetTransform()->GetWorldTM().GetBasisZ();

        // Pitch value depends on whether pitch input key is ignored via SetPitchKeyValue()
        const float pitchValue = m_ignorePitchKeyInputValue ? m_pitchKeyValue : m_pitch;
        // Yaw value depends on whether yaw input key is ignored via SetYawKeyValue()
        const float yawValue = m_ignoreYawKeyInputValue ? m_yawKeyValue : m_yaw;
        // Roll value depends on whether roll input key is ignored via SetRollKeyValue()
        const float rollValue = m_ignoreRollKeyInputValue ? m_rollKeyValue : m_roll;

        // Rotate the object using SetRotation (Transform) if it is a Kinematic Rigid Body
        if (m_isObjectKinematic)
        {
            AZ::Quaternion rotation = 
                AZ::Quaternion::CreateFromAxisAngle(m_upVector, yawValue * (m_kinematicRotateScale * 0.01f)) +
                AZ::Quaternion::CreateFromAxisAngle(m_rightVector, pitchValue * (m_kinematicRotateScale * 0.01f)) +
                AZ::Quaternion::CreateFromAxisAngle(m_forwardVector, rollValue * (m_kinematicRotateScale * 0.01f));

            AZ::Transform transform = AZ::Transform::CreateIdentity();
            AZ::TransformBus::EventResult(transform, m_lastGrabbedObjectEntityId, &AZ::TransformInterface::GetWorldTM);

            transform.SetRotation((rotation * transform.GetRotation()).GetNormalized());

            AZ::TransformBus::Event(m_lastGrabbedObjectEntityId, &AZ::TransformInterface::SetWorldTM, transform);

            // Update mesh entity transform if smoothing is disabled
            if (!m_enableMeshSmoothing && m_meshEntityPtr && m_meshEntityPtr != GetEntityPtr(m_lastGrabbedObjectEntityId))
            {
                AZ::TransformBus::Event(m_meshEntityPtr->GetId(), &AZ::TransformInterface::SetWorldTM, transform);
            }
        }
        // Rotate the object using SetAngularVelocity (PhysX) if it is a Dynamic Rigid Body
        else
        {
            SetGrabbedObjectAngularVelocity(
                GetGrabbedObjectAngularVelocity() + (m_rightVector * pitchValue * m_dynamicRotateScale) +
                (m_forwardVector * rollValue * m_dynamicRotateScale) + (m_upVector * yawValue * m_dynamicRotateScale));

            // Update current physics transform for interpolation
            if (m_enableMeshSmoothing)
            {
                AZ::TransformBus::EventResult(m_currentPhysicsTransform, m_lastGrabbedObjectEntityId, &AZ::TransformInterface::GetWorldTM);
            }
            // Update mesh entity transform if smoothing is disabled
            else if (m_meshEntityPtr && m_meshEntityPtr != GetEntityPtr(m_lastGrabbedObjectEntityId))
            {
                AZ::TransformBus::Event(
                    m_meshEntityPtr->GetId(),
                    &AZ::TransformInterface::SetWorldTM,
                    GetEntityPtr(m_lastGrabbedObjectEntityId)->GetTransform()->GetWorldTM());
            }
        }
    }

    // Apply linear impulse to object if it is a Dynamic Rigid Body
    void ObjectInteractionComponent::ThrowObject()
    {
        // Apply a Linear Impulse to the grabbed object
        Physics::RigidBodyRequestBus::Event(
            m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequestBus::Events::ApplyLinearImpulse, m_forwardVector * m_throwImpulse);

        // Trigger an event notification when object enters Throw State
        ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnThrowStart);

        m_thrownGrabbedObjectEntityId = m_lastGrabbedObjectEntityId;

        // Reset current grabbed distance to m_initialGrabDistance if grab key is not pressed
        m_grabDistance = m_initialGrabDistance;

        // Set Object Current Layer variable back to Prev Layer when thrown
        SetCurrentGrabbedCollisionLayer(m_prevGrabbedCollisionLayer);

        // Set Angular Damping back to original value
        SetCurrentGrabbedObjectAngularDamping(m_prevObjectAngularDamping);
    }

    // Apply tidal lock to grabbed object while grabbing it. This keeps the object facing you in its last rotation while in grabbed state
    void ObjectInteractionComponent::TidalLock()
    {
        AZ::Vector3 entityRotation = GetEntity()->GetTransform()->GetWorldRotation();

        const AZ::Vector3 entityUpVector = GetEntity()->GetTransform()->GetWorldTM().GetBasisZ();

        const AZ::Quaternion Rotation =
            AZ::Quaternion::CreateFromAxisAngle(entityUpVector, (entityRotation.GetZ() - m_lastEntityRotation.GetZ()));

        AZ::Transform Transform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(Transform, m_lastGrabbedObjectEntityId, &AZ::TransformInterface::GetWorldTM);

        Transform.SetRotation((Rotation * Transform.GetRotation()).GetNormalized());

        AZ::TransformBus::Event(m_lastGrabbedObjectEntityId, &AZ::TransformInterface::SetWorldTM, Transform);

        m_lastEntityRotation = entityRotation;
    }

    #ifdef FIRST_PERSON_CONTROLLER
    void ObjectInteractionComponent::FreezeCharacterRotation()
    {
        if (FirstPersonController::FirstPersonControllerComponentRequestBus::HasHandlers() && m_freezeCharacterRotation)
        {
            FirstPersonController::FirstPersonControllerComponentRequestBus::Event(
                GetEntityId(), &FirstPersonController::FirstPersonControllerComponentRequestBus::Events::UpdateCharacterAndCameraYaw, 0.f, false);
            FirstPersonController::FirstPersonControllerComponentRequestBus::Event(
                GetEntityId(), &FirstPersonController::FirstPersonControllerComponentRequestBus::Events::UpdateCameraPitch, 0.f, false);
        }
        else if (m_freezeCharacterRotation)
        {
            AZ_Warning(
                "Object Interaction Component",
                false,
                "No First Person Controller Component handler available to freeze character rotation.")
        }
    }
    #endif

    AZ::Entity* ObjectInteractionComponent::GetEntityPtr(AZ::EntityId pointer) const
    {
        auto ca = AZ::Interface<AZ::ComponentApplicationRequests>::Get();
        return ca->FindEntity(pointer);
    }

    // Handles disconnecting, updating, and reconnecting input bindings
    void ObjectInteractionComponent::UpdateInputBinding(
        StartingPointInput::InputEventNotificationId& eventId, AZStd::string& binding, const AZStd::string& newValue)
    {
        if (binding == newValue)
            return;

        // Disconnect from the old binding
        InputEventNotificationBus::MultiHandler::BusDisconnect(eventId);

        // Update the binding string and event ID
        binding = newValue;
        eventId = StartingPointInput::InputEventNotificationId(binding.c_str());

        // Reconnect with the new binding
        InputEventNotificationBus::MultiHandler::BusConnect(eventId);
    }

    // Event Notification methods for use in scripts
    void ObjectInteractionComponent::OnObjectSphereCastHit()
    {
    }
    void ObjectInteractionComponent::OnHoldStart()
    {
    }
    void ObjectInteractionComponent::OnHoldStop()
    {
    }
    void ObjectInteractionComponent::OnRotateStart()
    {
    }
    void ObjectInteractionComponent::OnRotateStop()
    {
    }
    void ObjectInteractionComponent::OnThrowStart()
    {
    }
    void ObjectInteractionComponent::OnThrowStop()
    {
    }
    void ObjectInteractionComponent::OnMaxThrowDistance()
    {
    }
    void ObjectInteractionComponent::OnThrowStateCounterZero()
    {
    }

    // Request Bus getter and setter methods for use in scripts
    AZ::EntityId ObjectInteractionComponent::GetGrabbingEntityId() const
    {
        return m_grabbingEntityPtr->GetId();
    }

    AZ::EntityId ObjectInteractionComponent::GetActiveCameraEntityId() const
    {
        return GetActiveCameraEntityPtr()->GetId();
    }

    AZ::EntityId ObjectInteractionComponent::GetGrabbedObjectEntityId() const
    {
        return m_grabbedObjectEntityId;
    }

    AZ::EntityId ObjectInteractionComponent::GetLastGrabbedObjectEntityId() const
    {
        return m_lastGrabbedObjectEntityId;
    }

    AZ::EntityId ObjectInteractionComponent::GetThrownGrabbedObjectEntityId() const
    {
        return m_thrownGrabbedObjectEntityId;
    }

    void ObjectInteractionComponent::SetThrownGrabbedObjectEntityId(const AZ::EntityId new_thrownGrabbedObjectEntityId)
    {
        m_thrownGrabbedObjectEntityId = new_thrownGrabbedObjectEntityId;
    }

    void ObjectInteractionComponent::SetGrabbingEntity(const AZ::EntityId new_grabbingEntityId)
    {
        m_grabbingEntityPtr = GetEntityPtr(new_grabbingEntityId);
    }

    AZStd::string ObjectInteractionComponent::GetStateString() const
    {
        return m_statesMap.find(m_state)->second;
    }

    bool ObjectInteractionComponent::GetIsInIdleState() const
    {
        return (m_state == ObjectInteractionStates::idleState);
    }

    bool ObjectInteractionComponent::GetIsInCheckState() const
    {
        return (m_state == ObjectInteractionStates::checkState);
    }

    bool ObjectInteractionComponent::GetIsInHeldState() const
    {
        return (m_state == ObjectInteractionStates::holdState);
    }

    bool ObjectInteractionComponent::GetIsInRotateState() const
    {
        return (m_state == ObjectInteractionStates::rotateState);
    }

    bool ObjectInteractionComponent::GetIsInThrowState() const
    {
        return (m_state == ObjectInteractionStates::throwState);
    }

    bool ObjectInteractionComponent::GetObjectSphereCastHit() const
    {
        return m_objectSphereCastHit;
    }

    bool ObjectInteractionComponent::GetStayInIdleState() const
    {
        return m_stayInIdleState;
    }
    
    void ObjectInteractionComponent::SetStayInIdleState(const bool& new_stayInIdleState)
    {
        m_stayInIdleState = new_stayInIdleState;
    }

    bool ObjectInteractionComponent::GetGrabEnableToggle() const
    {
        return m_grabEnableToggle;
    }

    void ObjectInteractionComponent::SetGrabEnableToggle(const bool& new_grabEnableToggle)
    {
        m_grabEnableToggle = new_grabEnableToggle;
    }

    bool ObjectInteractionComponent::GetRotateEnableToggle() const
    {
        return m_rotateEnableToggle;
    }

    void ObjectInteractionComponent::SetRotateEnableToggle(const bool& new_rotateEnableToggle)
    {
        m_rotateEnableToggle = new_rotateEnableToggle;
    }

    float ObjectInteractionComponent::GetGrabKeyValue() const
    {
        return m_grabKeyValue;
    }

    void ObjectInteractionComponent::SetGrabKeyValue(const float& new_grabKeyValue)
    {
        m_grabKeyValue = new_grabKeyValue;
    }

    float ObjectInteractionComponent::GetThrowKeyValue() const
    {
        return m_throwKeyValue;
    }

    void ObjectInteractionComponent::SetThrowKeyValue(const float& new_throwKeyValue)
    {
        m_throwKeyValue = new_throwKeyValue;
    }

    float ObjectInteractionComponent::GetRotateKeyValue() const
    {
        return m_rotateKeyValue;
    }

    void ObjectInteractionComponent::SetRotateKeyValue(const float& new_rotateKeyValue)
    {
        m_rotateKeyValue = new_rotateKeyValue;
    }

    float ObjectInteractionComponent::GetPitchKeyValue() const
    {
         return m_pitchKeyValue;
    }

    void ObjectInteractionComponent::SetPitchKeyValue(const float& new_pitchKeyValue, const bool& new_ignorePitchKeyInputValue)
    {
        if (new_ignorePitchKeyInputValue)
        {
            m_pitchKeyValue = new_pitchKeyValue;
            m_ignorePitchKeyInputValue = true;
        }
        else
        {
            const float newPitch = m_pitchKeyValue;
            m_pitch = new_pitchKeyValue + newPitch;
            m_ignorePitchKeyInputValue = false;
        }
    }

    float ObjectInteractionComponent::GetYawKeyValue() const
    {
        return m_yawKeyValue;
    }

    void ObjectInteractionComponent::SetYawKeyValue(const float& new_yawKeyValue, const bool& new_ignoreYawKeyInputValue)
    {
        if (new_ignoreYawKeyInputValue)
        {
            m_yawKeyValue = new_yawKeyValue;
            m_ignoreYawKeyInputValue = true;
        }
        else
        {
            const float newYaw = m_yawKeyValue;
            m_yaw = new_yawKeyValue + newYaw;
            m_ignoreYawKeyInputValue = false;
        }
    }

    float ObjectInteractionComponent::GetRollKeyValue() const
    {
        return m_rollKeyValue;
    }

    void ObjectInteractionComponent::SetRollKeyValue(const float& new_rollKeyValue, const bool& new_ignoreRollKeyInputValue)
    {
        if (new_ignoreRollKeyInputValue)
        {
            m_rollKeyValue = new_rollKeyValue;
            m_ignoreRollKeyInputValue = true;
        }
        else
        {
            const float newRoll = m_rollKeyValue;
            m_roll = new_rollKeyValue + newRoll;
            m_ignoreRollKeyInputValue = false;
        }
    }
    /*
    AZ::EntityId ObjectInteractionComponent::GetMeshEntityId() const
    {
        return m_meshEntityId;
    }

    void ObjectInteractionComponent::SetMeshEntityId(const AZ::EntityId& new_meshEntityId)
    {
        m_meshEntityId = new_meshEntityId;
        m_meshEntityPtr = m_meshEntityId.IsValid() ? GetEntityPtr(m_meshEntityId) : nullptr;
    }
    */
    AZStd::string ObjectInteractionComponent::GetMeshEntityName() const
    {
        return m_meshEntityName;
    }

    void ObjectInteractionComponent::SetMeshEntityName(const AZStd::string& new_meshEntityName)
    {
        m_meshEntityName = new_meshEntityName;
    }

    float ObjectInteractionComponent::GetGrabbedDistanceKeyValue() const
    {
        return m_grabDistanceKeyValue;
    }

    void ObjectInteractionComponent::SetGrabbedDistanceKeyValue(const float& new_grabDistanceKeyValue, const bool& new_ignoreGrabDistanceKeyInputValue)
    {
        if (new_ignoreGrabDistanceKeyInputValue)
        {
            m_grabDistanceKeyValue = new_grabDistanceKeyValue;
            m_ignoreGrabDistanceKeyInputValue = true;
        }
        else
        {
            const float newGrabDistance = m_grabDistanceKeyValue;
            m_combinedGrabDistance = new_grabDistanceKeyValue + newGrabDistance;
            m_ignoreGrabDistanceKeyInputValue = false;
        }
    }

    float ObjectInteractionComponent::GetGrabbedObjectDistance() const
    {
        return m_grabDistance;
    }

    void ObjectInteractionComponent::SetGrabbedObjectDistance(const float& new_grabDistance)
    {
        m_grabDistance = AZ::GetClamp(new_grabDistance, m_minGrabDistance, m_maxGrabDistance);
    }

    float ObjectInteractionComponent::GetMinGrabbedObjectDistance() const
    {
        return m_minGrabDistance;
    }

    void ObjectInteractionComponent::SetMinGrabbedObjectDistance(const float& new_minGrabDistance)
    {
        m_minGrabDistance = new_minGrabDistance;
    }

    float ObjectInteractionComponent::GetMaxGrabbedObjectDistance() const
    {
        return m_maxGrabDistance;
    }

    void ObjectInteractionComponent::SetMaxGrabbedObjectDistance(const float& new_maxGrabDistance)
    {
        m_maxGrabDistance = new_maxGrabDistance;
    }

    float ObjectInteractionComponent::GetInitialGrabbedObjectDistance() const
    {
        return m_initialGrabDistance;
    }

    void ObjectInteractionComponent::SetInitialGrabbedObjectDistance(const float& new_initialGrabDistance)
    {
        m_initialGrabDistance = new_initialGrabDistance;
    }

    float ObjectInteractionComponent::GetGrabbedObjectDistanceSpeed() const
    {
        return m_grabDistanceSpeed;
    }

    void ObjectInteractionComponent::SetGrabbedObjectDistanceSpeed(const float& new_grabDistanceSpeed)
    {
        m_grabDistanceSpeed = new_grabDistanceSpeed;
    }

    float ObjectInteractionComponent::GetGrabResponse() const
    {
        return m_grabResponse;
    }

    void ObjectInteractionComponent::SetGrabResponse(const float& new_grabResponse)
    {
        m_grabResponse = new_grabResponse;
    }

    bool ObjectInteractionComponent::GetDynamicTidalLock() const
    {
        return m_dynamicTidalLock;
    }

    void ObjectInteractionComponent::SetDynamicTidalLock(const bool& new_dynamicTidalLock)
    {
        m_dynamicTidalLock = new_dynamicTidalLock;
    }

    bool ObjectInteractionComponent::GetKinematicTidalLock() const
    {
        return m_kinematicTidalLock;
    }

    void ObjectInteractionComponent::SetKinematicTidalLock(const bool& new_kinematicTidalLock)
    {
        m_kinematicTidalLock = new_kinematicTidalLock;
    }

    bool ObjectInteractionComponent::GetTidalLock() const
    {
        return m_tidalLock;
    }

    void ObjectInteractionComponent::SetTidalLock(const bool& new_tidalLock)
    {
        m_tidalLock = new_tidalLock;
    }

    float ObjectInteractionComponent::GetDynamicRotateScale() const
    {
        return m_dynamicRotateScale;
    }

    void ObjectInteractionComponent::SetDynamicRotateScale(const float& new_dynamicRotateScale)
    {
        m_dynamicRotateScale = new_dynamicRotateScale;
    }

    float ObjectInteractionComponent::GetKinematicRotateScale() const
    {
        return m_kinematicRotateScale;
    }

    void ObjectInteractionComponent::SetKinematicRotateScale(const float& new_kinematicRotateScale)
    {
        m_kinematicRotateScale = new_kinematicRotateScale;
    }

    float ObjectInteractionComponent::GetThrowImpulse() const
    {
        return m_throwImpulse;
    }

    void ObjectInteractionComponent::SetThrowImpulse(const float& new_throwImpulse)
    {
        m_throwImpulse = new_throwImpulse;
    }

    float ObjectInteractionComponent::GetGrabbedObjectThrowStateCounter() const
    {
        return m_throwStateCounter;
    }
    
    void ObjectInteractionComponent::SetGrabbedObjectThrowStateCounter(const float& new_throwStateCounter)
    {
        m_throwStateCounter = new_throwStateCounter;
    }

    float ObjectInteractionComponent::GetGrabbedObjectThrowStateTime() const
    {
        return m_throwStateMaxTime;
    }

    void ObjectInteractionComponent::SetGrabbedObjectThrowStateTime(const float& new_throwStateMaxTime)
    {
        m_throwStateMaxTime = new_throwStateMaxTime;
    }

    float ObjectInteractionComponent::GetSphereCastRadius() const
    {
        return m_sphereCastRadius;
    }

    void ObjectInteractionComponent::SetSphereCastRadius(const float& new_sphereCastRadius)
    {
        m_sphereCastRadius = new_sphereCastRadius;
    }

    float ObjectInteractionComponent::GetSphereCastDistance() const
    {
        return m_sphereCastDistance;
    }

    void ObjectInteractionComponent::SetSphereCastDistance(const float& new_sphereCastDistance)
    {
        m_sphereCastDistance = new_sphereCastDistance;
    }

    AZStd::string ObjectInteractionComponent::GetGrabbedCollisionGroup() const
    {
        AZStd::string groupName;
        Physics::CollisionRequestBus::BroadcastResult(
            groupName, &Physics::CollisionRequests::GetCollisionGroupName, m_grabbedCollisionGroup);
        return groupName;
    }

    void ObjectInteractionComponent::SetGrabbedCollisionGroup(const AZStd::string& new_grabbedCollisionGroupName)
    {
        bool success = false;
        AzPhysics::CollisionGroup collisionGroup;
        Physics::CollisionRequestBus::BroadcastResult(
            success, &Physics::CollisionRequests::TryGetCollisionGroupByName, new_grabbedCollisionGroupName, collisionGroup);
        if (success)
        {
            m_grabbedCollisionGroup = collisionGroup;
            const AzPhysics::CollisionConfiguration& configuration =
                AZ::Interface<AzPhysics::SystemInterface>::Get()->GetConfiguration()->m_collisionConfig;
            m_grabbedCollisionGroupId = configuration.m_collisionGroups.FindGroupIdByName(new_grabbedCollisionGroupName);
        }
    }

    AZStd::string ObjectInteractionComponent::GetCurrentGrabbedCollisionLayerName() const
    {
        AZStd::string currentGrabbedCollisionLayerName;
        Physics::CollisionFilteringRequestBus::EventResult(
            currentGrabbedCollisionLayerName,
            m_lastGrabbedObjectEntityId,
            &Physics::CollisionFilteringRequestBus::Events::GetCollisionLayerName);
        return currentGrabbedCollisionLayerName;
    }

    void ObjectInteractionComponent::SetCurrentGrabbedCollisionLayerByName(const AZStd::string& new_currentGrabbedCollisionLayerName)
    {
        bool success = false;
        AzPhysics::CollisionLayer grabbedCollisionLayer;
        Physics::CollisionRequestBus::BroadcastResult(
            success, &Physics::CollisionRequests::TryGetCollisionLayerByName, new_currentGrabbedCollisionLayerName, grabbedCollisionLayer);
        if (success)
        {
            m_currentGrabbedCollisionLayerName = new_currentGrabbedCollisionLayerName;
            m_currentGrabbedCollisionLayer = grabbedCollisionLayer;
            Physics::CollisionFilteringRequestBus::Event(
                m_lastGrabbedObjectEntityId,
                &Physics::CollisionFilteringRequestBus::Events::SetCollisionLayer,
                m_currentGrabbedCollisionLayerName,
                AZ::Crc32());
        }
    }

    AzPhysics::CollisionLayer ObjectInteractionComponent::GetCurrentGrabbedCollisionLayer() const
    {
        AZStd::string grabbedCollisionLayerName;
        Physics::CollisionFilteringRequestBus::EventResult(
            grabbedCollisionLayerName, m_lastGrabbedObjectEntityId, &Physics::CollisionFilteringRequestBus::Events::GetCollisionLayerName);
        AzPhysics::CollisionLayer grabbedCollisionLayer;
        Physics::CollisionRequestBus::BroadcastResult(
            grabbedCollisionLayer, &Physics::CollisionRequests::GetCollisionLayerByName, grabbedCollisionLayerName);
        return grabbedCollisionLayer;
    }

    void ObjectInteractionComponent::SetCurrentGrabbedCollisionLayer(const AzPhysics::CollisionLayer& new_currentGrabbedCollisionLayer)
    {
        m_currentGrabbedCollisionLayer = new_currentGrabbedCollisionLayer;
        const AzPhysics::CollisionConfiguration& configuration =
            AZ::Interface<AzPhysics::SystemInterface>::Get()->GetConfiguration()->m_collisionConfig;
        m_currentGrabbedCollisionLayerName = configuration.m_collisionLayers.GetName(m_currentGrabbedCollisionLayer);
        Physics::CollisionFilteringRequestBus::Event(
            m_lastGrabbedObjectEntityId,
            &Physics::CollisionFilteringRequestBus::Events::SetCollisionLayer,
            m_currentGrabbedCollisionLayerName,
            AZ::Crc32());
    }

    AZStd::string ObjectInteractionComponent::GetPrevGrabbedCollisionLayerName() const
    {
        const AzPhysics::CollisionConfiguration& configuration =
            AZ::Interface<AzPhysics::SystemInterface>::Get()->GetConfiguration()->m_collisionConfig;
        return configuration.m_collisionLayers.GetName(m_prevGrabbedCollisionLayer);
    }

    void ObjectInteractionComponent::SetPrevGrabbedCollisionLayerByName(const AZStd::string& new_prevGrabbedCollisionLayerName)
    {
        bool success = false;
        AzPhysics::CollisionLayer prevGrabbedCollisionLayer;
        Physics::CollisionRequestBus::BroadcastResult(
            success, &Physics::CollisionRequests::TryGetCollisionLayerByName, new_prevGrabbedCollisionLayerName, prevGrabbedCollisionLayer);
        if (success)
            m_prevGrabbedCollisionLayer = prevGrabbedCollisionLayer;
    }

    AzPhysics::CollisionLayer ObjectInteractionComponent::GetPrevGrabbedCollisionLayer() const
    {
        return m_prevGrabbedCollisionLayer;
    }

    void ObjectInteractionComponent::SetPrevGrabbedCollisionLayer(const AzPhysics::CollisionLayer& new_prevGrabbedCollisionLayer)
    {
        m_prevGrabbedCollisionLayer = new_prevGrabbedCollisionLayer;
    }

    AZStd::string ObjectInteractionComponent::GetTempGrabbedCollisionLayerName() const
    {
        return m_tempGrabbedCollisionLayerName;
    }

    void ObjectInteractionComponent::SetTempGrabbedCollisionLayerByName(const AZStd::string& new_tempGrabbedCollisionLayerName)
    {
        bool success = false;
        AzPhysics::CollisionLayer tempGrabbedCollisionLayer;
        Physics::CollisionRequestBus::BroadcastResult(
            success, &Physics::CollisionRequests::TryGetCollisionLayerByName, new_tempGrabbedCollisionLayerName, tempGrabbedCollisionLayer);
        if (success)
        {
            m_tempGrabbedCollisionLayerName = new_tempGrabbedCollisionLayerName;
            m_tempGrabbedCollisionLayer = tempGrabbedCollisionLayer;
        }
    }

    AzPhysics::CollisionLayer ObjectInteractionComponent::GetTempGrabbedCollisionLayer() const
    {
        return m_tempGrabbedCollisionLayer;
    }

    void ObjectInteractionComponent::SetTempGrabbedCollisionLayer(const AzPhysics::CollisionLayer& new_tempGrabbedCollisionLayer)
    {
        m_tempGrabbedCollisionLayer = new_tempGrabbedCollisionLayer;
        const AzPhysics::CollisionConfiguration& configuration =
            AZ::Interface<AzPhysics::SystemInterface>::Get()->GetConfiguration()->m_collisionConfig;
        m_tempGrabbedCollisionLayerName = configuration.m_collisionLayers.GetName(m_tempGrabbedCollisionLayer);
    }

    bool ObjectInteractionComponent::GetGrabbedObjectKinematicElseDynamic() const
    {
        bool isObjectKinematic = false;
        Physics::RigidBodyRequestBus::EventResult(
            isObjectKinematic, m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequestBus::Events::IsKinematic);

        return isObjectKinematic;
    }

    void ObjectInteractionComponent::SetGrabbedObjectKinematicElseDynamic(const bool& isKinematic)
    {
        Physics::RigidBodyRequestBus::Event(m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::SetKinematic, isKinematic);
    }

    bool ObjectInteractionComponent::GetInitialGrabbedObjectIsKinematic() const
    {
        return m_isInitialObjectKinematic;
    }

    float ObjectInteractionComponent::GetCurrentGrabbedObjectAngularDamping() const
    {
        float currentObjectAngularDamping = 0.f;

        Physics::RigidBodyRequestBus::EventResult(
            currentObjectAngularDamping, m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::GetAngularDamping);

        return currentObjectAngularDamping;
    }

    void ObjectInteractionComponent::SetCurrentGrabbedObjectAngularDamping(const float& new_currentObjectAngularDamping)
    {
        m_currentObjectAngularDamping = new_currentObjectAngularDamping;

        Physics::RigidBodyRequestBus::Event(
            m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::SetAngularDamping, new_currentObjectAngularDamping);
    }

    float ObjectInteractionComponent::GetPrevGrabbedObjectAngularDamping() const
    {
        return m_prevObjectAngularDamping;
    }

    void ObjectInteractionComponent::SetPrevGrabbedObjectAngularDamping(const float& new_prevObjectAngularDamping)
    {
        m_prevObjectAngularDamping = new_prevObjectAngularDamping;
    }

    float ObjectInteractionComponent::GetTempGrabbedObjectAngularDamping() const
    {
        return m_tempObjectAngularDamping;
    }

    void ObjectInteractionComponent::SetTempGrabbedObjectAngularDamping(const float& new_tempObjectAngularDamping)
    {
        m_tempObjectAngularDamping = new_tempObjectAngularDamping;
        Physics::RigidBodyRequestBus::Event(
            m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::SetAngularDamping, m_tempObjectAngularDamping);
    }

    AZ::Vector3 ObjectInteractionComponent::GetGrabbedObjectAngularVelocity() const
    {
        AZ::Vector3 grabbedObjectAngularVelocity = AZ::Vector3::CreateZero();

        Physics::RigidBodyRequestBus::EventResult(
            grabbedObjectAngularVelocity, m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::GetAngularVelocity);
        return grabbedObjectAngularVelocity;
    }

    void ObjectInteractionComponent::SetGrabbedObjectAngularVelocity(const AZ::Vector3& new_grabbedObjectAngularVelocity)
    {
        m_grabbedObjectAngularVelocity = new_grabbedObjectAngularVelocity;

        Physics::RigidBodyRequestBus::Event(
            m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::SetAngularVelocity, m_grabbedObjectAngularVelocity);
    }

    bool ObjectInteractionComponent::GetInitialAngularVelocityZero() const
    {
        return m_initialAngularVelocityZero;
    }

    void ObjectInteractionComponent::SetInitialAngularVelocityZero(const bool& new_initialAngularVelocityZero)
    {
        m_initialAngularVelocityZero = new_initialAngularVelocityZero;
    }
    
    void ObjectInteractionComponent::ForceTransition(const ObjectInteractionStates& targetState)
    {
        m_forceTransition = true;
        m_targetState = targetState;
    }

    void ObjectInteractionComponent::SetStateLocked(const bool& isLocked)
    {
        m_isStateLocked = isLocked;
    }

    bool ObjectInteractionComponent::GetStateLocked() const
    {
        return m_isStateLocked;
    }

    AZStd::string ObjectInteractionComponent::GetGrabInputKey() const
    {
        return m_strGrab;
    }
    void ObjectInteractionComponent::SetGrabInputKey(const AZStd::string& keyName)
    {
        UpdateInputBinding(m_grabEventId, m_strGrab, keyName);
    }

    AZStd::string ObjectInteractionComponent::GetThrowInputKey() const
    {
        return m_strThrow;
    }
    void ObjectInteractionComponent::SetThrowInputKey(const AZStd::string& keyName)
    {
        UpdateInputBinding(m_throwEventId, m_strThrow, keyName);
    }

    AZStd::string ObjectInteractionComponent::GetRotateInputKey() const
    {
        return m_strRotate;
    }
    void ObjectInteractionComponent::SetRotateInputKey(const AZStd::string& keyName)
    {
        UpdateInputBinding(m_rotateEventId, m_strRotate, keyName);
    }

    AZStd::string ObjectInteractionComponent::GetRotatePitchInputKey() const
    {
        return m_strRotatePitch;
    }
    void ObjectInteractionComponent::SetRotatePitchInputKey(const AZStd::string& keyName)
    {
        UpdateInputBinding(m_rotatePitchEventId, m_strRotatePitch, keyName);
    }

    AZStd::string ObjectInteractionComponent::GetRotateYawInputKey() const
    {
        return m_strRotateYaw;
    }
    void ObjectInteractionComponent::SetRotateYawInputKey(const AZStd::string& keyName)
    {
        UpdateInputBinding(m_rotateYawEventId, m_strRotateYaw, keyName);
    }

    AZStd::string ObjectInteractionComponent::GetRotateRollInputKey() const
    {
        return m_strRotateRoll;
    }
    void ObjectInteractionComponent::SetRotateRollInputKey(const AZStd::string& keyName)
    {
        UpdateInputBinding(m_rotateRollEventId, m_strRotateRoll, keyName);
    }

    AZStd::string ObjectInteractionComponent::GetGrabDistanceInputKey() const
    {
        return m_strGrabDistance;
    }
    void ObjectInteractionComponent::SetGrabDistanceInputKey(const AZStd::string& keyName)
    {
        UpdateInputBinding(m_grabDistanceEventId, m_strGrabDistance, keyName);
    }
} // namespace ObjectInteraction
