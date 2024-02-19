#include "Grab.h"

#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzFramework/Physics/CollisionBus.h>
#include <AzFramework/Physics/NameConstants.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <AzFramework/Physics/SystemBus.h>
#include <System/PhysXSystem.h>

namespace TestGem
{
    using namespace StartingPointInput;

    void Grab::Reflect(AZ::ReflectContext* rc)
    {
        if (auto sc = azrtti_cast<AZ::SerializeContext*>(rc))
        {
            sc->Class<Grab, AZ::Component>()

                // Grab Input Binding Keys
                ->Field("Grab Input Key", &Grab::m_strGrab)
                ->Field("Grab Distance Input Key", &Grab::m_strGrabDistance)
                ->Field("Throw Input Key", &Grab::m_strThrow)
                ->Field("Rotate Enable Input Key", &Grab::m_strRotate)
                ->Field("Rotate Pitch Key", &Grab::m_strRotatePitch)
                ->Field("Rotate Yaw Key", &Grab::m_strRotateYaw)
                ->Field("Rotate Roll Key", &Grab::m_strRotateRoll)

                ->Field("GrabbingEntityId", &Grab::m_grabbingEntityId)
                ->Field("Grab Enable Toggle", &Grab::m_grabEnableToggle)
                ->Field("Maintain Grab", &Grab::m_grabMaintained)
                ->Field("Kinematic While Grabbing", &Grab::m_kinematicWhileGrabbing)
                ->Field("Rotate Enable Toggle", &Grab::m_rotateEnableToggle)
                ->Field("Sphere Cast Radius", &Grab::m_sphereCastRadius)
                ->Attribute(AZ::Edit::Attributes::Suffix, " " + Physics::NameConstants::GetLengthUnit())
                ->Field("Sphere Cast Distance", &Grab::m_sphereCastDistance)
                ->Attribute(AZ::Edit::Attributes::Suffix, " " + Physics::NameConstants::GetLengthUnit())
                ->Field("Default Grab Distance", &Grab::m_initialGrabDistance)
                ->Attribute(AZ::Edit::Attributes::Suffix, " " + Physics::NameConstants::GetLengthUnit())
                ->Field("Min Grab Distance", &Grab::m_minGrabDistance)
                ->Attribute(AZ::Edit::Attributes::Suffix, " " + Physics::NameConstants::GetLengthUnit())
                ->Field("Max Grab Distance", &Grab::m_maxGrabDistance)
                ->Attribute(AZ::Edit::Attributes::Suffix, " " + Physics::NameConstants::GetLengthUnit())
                ->Field("Grab Distance Speed", &Grab::m_grabDistanceSpeed)
                ->Attribute(AZ::Edit::Attributes::Suffix, " " + Physics::NameConstants::GetSpeedUnit())
                ->Field("Throw Impulse", &Grab::m_throwImpulse)
                ->Attribute(AZ::Edit::Attributes::Suffix, AZStd::string::format(" N%ss", Physics::NameConstants::GetInterpunct().c_str()))
                ->Field("Grab Response", &Grab::m_grabResponse)
                ->Attribute(
                    AZ::Edit::Attributes::Suffix,
                    AZStd::string::format(
                        " s%s%s",
                        Physics::NameConstants::GetSuperscriptMinus().c_str(),
                        Physics::NameConstants::GetSuperscriptOne().c_str()))
                ->Field("Kinematic Rotate Scale", &Grab::m_kinematicRotateScale)
                ->Field("Dynamic Rotate Scale", &Grab::m_dynamicRotateScale)
                ->Field("Angular Damping", &Grab::m_tempObjectAngularDamping)
                ->Field("Grabbed Object Collision Group", &Grab::m_grabbedCollisionGroupId)
                ->Field("Grabbed Object Temporary Collision Layer", &Grab::m_tempGrabbedCollisionLayer)
                ->Version(1);

            if (AZ::EditContext* ec = sc->GetEditContext())
            {
                using namespace AZ::Edit;
                ec->Class<Grab>("Grab", "[Grab Interaction Component]")
                    ->ClassElement(ClassElements::EditorData, "")
                    ->Attribute(Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))

                    ->DataElement(
                        0,
                        &Grab::m_grabbingEntityId,
                        "Grab Entity",
                        "Reference entity that interacts with objects. If left blank, Camera entity will be used by default.")

                    // Input Binding Keys
                    ->ClassElement(AZ::Edit::ClassElements::Group, "Input Bindings")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(nullptr, &Grab::m_strGrab, "Grab Key", "Grab interaction input binding")
                    ->DataElement(nullptr, &Grab::m_strGrabDistance, "Grab Distance Key", "Grab distance input binding")
                    ->DataElement(nullptr, &Grab::m_strThrow, "Throw Input Key", "Throw object input binding")
                    ->DataElement(nullptr, &Grab::m_strRotate, "Rotate Enable Key", "Enable rotate object input binding")
                    ->DataElement(nullptr, &Grab::m_strRotatePitch, "Rotate Pitch Key", "Rotate object about X axis input binding")
                    ->DataElement(nullptr, &Grab::m_strRotateYaw, "Rotate Yaw Key", "Rotate object about Z axis input binding")
                    ->DataElement(nullptr, &Grab::m_strRotateRoll, "Rotate Roll Key", "Rotate object about Y axis input binding")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Toggle Preferences")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        nullptr,
                        &Grab::m_grabEnableToggle,
                        "Grab Enable Toggle",
                        "Determines whether pressing Grab Key toggles Grab mode. Disabling this requires the Grab key to be held to "
                        "maintain Grab mode.")
                    ->DataElement(
                        nullptr,
                        &Grab::m_grabMaintained,
                        "Maintain Grab",
                        "Grabbed Object remains held even if sphere cast no longer intersects it. This prevents the Grabbed Object from "
                        "flying off when quickly changing directions.")
                    ->DataElement(
                        nullptr, &Grab::m_kinematicWhileGrabbing, "Kinematic Grabbed Object", "Sets the grabbed object to kinematic.")
                    ->DataElement(
                        nullptr,
                        &Grab::m_rotateEnableToggle,
                        "Rotate Enable Toggle",
                        "Determines whether pressing Rotate Key toggles Rotate mode. Disabling this requires the Rotate key to be held to "
                        "maintain Rotate mode.")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Scaling Factors")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        nullptr, &Grab::m_throwImpulse, "Throw Impulse", "Linear Impulse scale applied when throwing grabbed object")
                    ->DataElement(
                        nullptr, &Grab::m_grabResponse, "Grab Response", "Linear velocity scale applied when holding grabbed object")
                    ->DataElement(
                        nullptr,
                        &Grab::m_kinematicRotateScale,
                        "Kinematic Rotate Scale",
                        "Rotation speed scale applied when rotating kinematic object")
                    ->DataElement(
                        nullptr,
                        &Grab::m_dynamicRotateScale,
                        "Dynamic Rotate Scale",
                        "Angular Velocity scale applied when rotating dynamic object")
                    ->DataElement(
                        nullptr, &Grab::m_tempObjectAngularDamping, "Angular Damping", "Angular Damping of Grabbed Object while Grabbing")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Sphere Cast Parameters")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        nullptr,
                        &Grab::m_grabbedCollisionGroupId,
                        "Sphere Cast Collision Group",
                        "The collision group which will be used for detecting grabbable objects.")
                    ->DataElement(
                        nullptr,
                        &Grab::m_tempGrabbedCollisionLayer,
                        "Grabbed Object Temporary Collision Layer",
                        "The temporary collision layer assigned to the grabbed object while it is being grabbed/held.")
                    ->DataElement(nullptr, &Grab::m_sphereCastRadius, "Sphere Cast Radius", "Sphere Cast radius used for grabbing objects")
                    ->DataElement(
                        nullptr, &Grab::m_sphereCastDistance, "Sphere Cast Distance", "Sphere Cast distance along m_sphereCastDirection")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Grab Distance Parameters")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        nullptr,
                        &Grab::m_initialGrabDistance,
                        "Default Grab Distance",
                        "Distance the grabbed object will default to when letting go of the object")
                    ->DataElement(
                        nullptr,
                        &Grab::m_minGrabDistance,
                        "Min Grab Distance",
                        "Minimum allowable grab distance. Grabbed object cannot get closer than this distance.")
                    ->DataElement(
                        nullptr,
                        &Grab::m_maxGrabDistance,
                        "Max Grab Distance",
                        "Maximum allowable grab distance. Grabbed object cannot get further than this distance.")
                    ->DataElement(
                        nullptr,
                        &Grab::m_grabDistanceSpeed,
                        "Grab Distance Speed",
                        "The speed at which you move the grabbed object closer or away.");
            }
        }

        if (auto bc = azrtti_cast<AZ::BehaviorContext*>(rc))
        {
            bc->EBus<TestGemNotificationBus>("GrabNotificationBus", "GrabComponentNotificationBus", "Notifications for Grab Component")
                ->Handler<TestGemNotificationHandler>();

            bc->EBus<TestGemComponentRequestBus>("TestGemComponentRequestBus")
                ->Attribute(AZ::Script::Attributes::Scope, AZ::Script::Attributes::ScopeFlags::Common)
                ->Attribute(AZ::Script::Attributes::Module, "interaction")
                ->Attribute(AZ::Script::Attributes::Category, "Grab")
                ->Event("Get Grabbing EntityId", &TestGemComponentRequests::GetGrabbingEntityId)
                ->Event("Get Active Camera EntityId", &TestGemComponentRequests::GetActiveCameraEntityId)
                ->Event("Get Grabbed Object EntityId", &TestGemComponentRequests::GetGrabbedObjectEntityId)
                ->Event("Get Last Grabbed Object EntityId", &TestGemComponentRequests::GetLastGrabbedObjectEntityId)
                ->Event("Set Grabbing Entity", &TestGemComponentRequests::SetGrabbingEntity)
                ->Event("Get Grabbed Collision Group", &TestGemComponentRequests::GetGrabbedCollisionGroup)
                ->Event("Set Grabbed Collision Group", &TestGemComponentRequests::SetGrabbedCollisionGroup)
                ->Event("Get Current Grabbed Layer Name", &TestGemComponentRequests::GetCurrentGrabbedCollisionLayerName)
                ->Event("Set Current Grabbed Layer By Name", &TestGemComponentRequests::SetCurrentGrabbedCollisionLayerByName)
                ->Event("Get Current Grabbed Layer", &TestGemComponentRequests::GetCurrentGrabbedCollisionLayer)
                ->Event("Set Current Grabbed Layer", &TestGemComponentRequests::SetCurrentGrabbedCollisionLayer)
                ->Event("Get Previous Grabbed Layer Name", &TestGemComponentRequests::GetPrevGrabbedCollisionLayerName)
                ->Event("Set Previous Grabbed Layer Name By Name", &TestGemComponentRequests::SetPrevGrabbedCollisionLayerByName)
                ->Event("Get Previous Grabbed Layer", &TestGemComponentRequests::GetPrevGrabbedCollisionLayer)
                ->Event("Set Previous Grabbed Layer", &TestGemComponentRequests::SetPrevGrabbedCollisionLayer)
                ->Event("Get Temporary Grabbed Layer Name", &TestGemComponentRequests::GetTempGrabbedCollisionLayerName)
                ->Event("Set Temporary Grabbed Layer By Name", &TestGemComponentRequests::SetTempGrabbedCollisionLayerByName)
                ->Event("Get Temporary Grabbed Layer", &TestGemComponentRequests::GetTempGrabbedCollisionLayer)
                ->Event("Set Temporary Grabbed Layer", &TestGemComponentRequests::SetTempGrabbedCollisionLayer)
                ->Event("Get Is In Grab State", &TestGemComponentRequests::GetIsInGrabState)
                ->Event("Get Is In Throw State", &TestGemComponentRequests::GetIsInThrowState)
                ->Event("Get Is In Rotate State", &TestGemComponentRequests::GetIsInRotateState)
                ->Event("Get Object Sphere Cast Hit", &TestGemComponentRequests::GetObjectSphereCastHit)
                ->Event("Get Grabbed Object Distance", &TestGemComponentRequests::GetGrabbedObjectDistance)
                ->Event("Set Grabbed Object Distance", &TestGemComponentRequests::SetGrabbedObjectDistance)
                ->Event("Get Minimum Grabbed Object Distance", &TestGemComponentRequests::GetMinGrabbedObjectDistance)
                ->Event("Set Minimum Grabbed Object Distance", &TestGemComponentRequests::SetMinGrabbedObjectDistance)
                ->Event("Get Maximum Grabbed Objectt Distance", &TestGemComponentRequests::GetMaxGrabbedObjectDistance)
                ->Event("Set Maximum Grabbed Object Distance", &TestGemComponentRequests::SetMaxGrabbedObjectDistance)
                ->Event("Get Initial Grabbed Object Distance", &TestGemComponentRequests::GetInitialGrabbedObjectDistance)
                ->Event("Set Initial Grabbed Objectt Distance", &TestGemComponentRequests::SetInitialGrabbedObjectDistance)
                ->Event("Get Grabbed Object Distance Speed", &TestGemComponentRequests::GetGrabbedObjectDistanceSpeed)
                ->Event("Set Grabbed Object Distance Speed", &TestGemComponentRequests::SetGrabbedObjectDistanceSpeed)
                ->Event("Get Grab Response", &TestGemComponentRequests::GetGrabResponse)
                ->Event("Set Grab Response", &TestGemComponentRequests::SetGrabResponse)
                ->Event("Get Grabbed Dynamic Object Rotation Scale", &TestGemComponentRequests::GetDynamicRotateScale)
                ->Event("Set Grabbed Dynamic Object Rotation Scale", &TestGemComponentRequests::SetDynamicRotateScale)
                ->Event("Get Grabbed Kinematic Object Rotation Scale", &TestGemComponentRequests::GetKinematicRotateScale)
                ->Event("Set Grabbed Kinematic Object Rotation Scale", &TestGemComponentRequests::SetKinematicRotateScale)
                ->Event("Get Grab Throw Impulse", &TestGemComponentRequests::GetThrowImpulse)
                ->Event("Set Grab Throw Impulse", &TestGemComponentRequests::SetThrowImpulse)
                ->Event("Get Grabbed Object Throw State Counter", &TestGemComponentRequests::GetGrabbedObjectThrowStateCounter)
                ->Event("Get Grabbed Object Throw State Max Time", &TestGemComponentRequests::GetGrabbedObjectThrowStateTime)
                ->Event("Set Grabbed Object Throw State Max Time", &TestGemComponentRequests::SetGrabbedObjectThrowStateTime)
                ->Event("Get Grab Sphere Cast Radius", &TestGemComponentRequests::GetSphereCastRadius)
                ->Event("Set Grab Sphere Cast Radius", &TestGemComponentRequests::SetSphereCastRadius)
                ->Event("Get Grab Sphere Cast Distance", &TestGemComponentRequests::GetSphereCastDistance)
                ->Event("Set Grab Sphere Cast Distance", &TestGemComponentRequests::SetSphereCastDistance)
                ->Event("Get Grabbed Object Is Kinematic", &TestGemComponentRequests::GetGrabbedObjectIsKinematic)
                ->Event("Set Grabbed Object Is Kinematic", &TestGemComponentRequests::SetGrabbedObjectIsKinematic)
                ->Event("Get Current Grabbed Object Angular Damping", &TestGemComponentRequests::GetCurrentGrabbedObjectAngularDamping)
                ->Event("Set Current Grabbed Object Angular Damping", &TestGemComponentRequests::SetCurrentGrabbedObjectAngularDamping)
                ->Event("Get Previous Grabbed Object Angular Damping", &TestGemComponentRequests::GetPrevGrabbedObjectAngularDamping)
                ->Event("Set Previous Grabbed Object Angular Damping", &TestGemComponentRequests::SetPrevGrabbedObjectAngularDamping)
                ->Event("Get Temporary Grabbed Object Angular Damping", &TestGemComponentRequests::GetTempGrabbedObjectAngularDamping)
                ->Event("Set Temporary Grabbed Object Angular Damping", &TestGemComponentRequests::SetTempGrabbedObjectAngularDamping);

            bc->Class<Grab>()->RequestBus("TestGemComponentRequestBus");
        }
    }
    void Grab::Activate()
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

        // Connect the handler to the request bus
        TestGemComponentRequestBus::Handler::BusConnect(GetEntityId());

        // Delaying the assignment of Grabbing Entity to OnEntityActivated so the Entity is activated and ready.
        AZ::EntityBus::Handler::BusConnect(m_grabbingEntityId);
    }

    void Grab::OnEntityActivated([[maybe_unused]] const AZ::EntityId& entityId)
    {
        AZ::EntityBus::Handler::BusDisconnect();

        if (m_grabbingEntityId.IsValid())
        {
            m_grabbingEntityPtr = GetEntityPtr(entityId);
        }
    }

    void Grab::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        InputEventNotificationBus::MultiHandler::BusDisconnect();

        // Disconnect the handler from the request bus
        TestGemComponentRequestBus::Handler::BusDisconnect();

        Camera::CameraNotificationBus::Handler::BusDisconnect();
    }

    void Grab::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("InputConfigurationService"));
        required.push_back(AZ_CRC_CE("TransformService"));
    }

    void Grab::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("GrabService"));
    }

    void Grab::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("GrabService"));
        incompatible.push_back(AZ_CRC_CE("InputService"));
    }

    void Grab::GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        dependent.push_back(AZ_CRC_CE("FirstPersonControllerService"));
    }

    void Grab::OnCameraAdded(const AZ::EntityId& cameraId)
    {
        if (!m_grabbingEntityId.IsValid())
        {
            m_grabbingEntityId = cameraId;
            m_grabbingEntityPtr = GetEntityPtr(cameraId);
        }
    }

    AZ::Entity* Grab::GetActiveCameraEntityPtr() const
    {
        AZ::EntityId activeCameraId;
        Camera::CameraSystemRequestBus::BroadcastResult(activeCameraId, &Camera::CameraSystemRequestBus::Events::GetActiveCamera);

        auto ca = AZ::Interface<AZ::ComponentApplicationRequests>::Get();
        return ca->FindEntity(activeCameraId);
    }

    // Recieve the input event in OnPressed method
    void Grab::OnPressed(float value)
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
    void Grab::OnReleased(float value)
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

    void Grab::OnHeld(float value)
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

    void Grab::OnTick(float deltaTime, AZ::ScriptTimePoint)
    {
        CheckForObjects();
        GrabObject(deltaTime);
        RotateObject(deltaTime);
        ThrowObject(deltaTime);

        m_prevGrabKeyValue = m_grabKeyValue;
        m_prevRotateKeyValue = m_rotateKeyValue;
    }

    AZ::Entity* Grab::GetEntityPtr(AZ::EntityId pointer) const
    {
        auto ca = AZ::Interface<AZ::ComponentApplicationRequests>::Get();
        return ca->FindEntity(pointer);
    }

    // Perform a spherecast query to check if colliding with a grabbable object, then assign the first returned hit to
    // m_grabbedObjectEntityId
    void Grab::CheckForObjects()
    {
        // Do not execute the function if Grab Key is not pressed and not currently in Grab State
        if (!m_grabKeyValue && !m_isInGrabState)
        {
            return;
        }

        const bool prevObjectSphereCastHit = m_objectSphereCastHit;

        // Get forward vector relative to the grabbing entity's transform
        m_forwardVector =
            AZ::Quaternion(m_grabbingEntityPtr->GetTransform()->GetWorldRotationQuaternion()).TransformVector(AZ::Vector3::CreateAxisY());

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

        m_objectSphereCastHit = hits ? true : false;

        // Prevents Grabbing new object if currently grabbing.
        if (!m_isInGrabState && m_objectSphereCastHit)
        {
            // Takes first hit from spherecast query vector, and assigns this EntityID to m_grabbedObjectEntityId
            m_grabbedObjectEntityId = hits.m_hits.at(0).m_entityId;
            m_lastGrabbedObjectEntityId = m_grabbedObjectEntityId;
        }

        // Trigger an event notification if SphereCast query succesfully hits an object
        if (!prevObjectSphereCastHit && m_objectSphereCastHit)
        {
            TestGemNotificationBus::Broadcast(&TestGemNotificationBus::Events::OnObjectSphereCastHit);
        }
    }

    // Hold and move object using physics or translation, based on object's starting Rigid Body type, or if KinematicWhileGrabbing is
    // enabled.
    void Grab::GrabObject(const float& deltaTime)
    {
        // Do not execute the function if there is no SphereCast hit and not in Grab State, or if the object is being thrown
        if ((!m_objectSphereCastHit && (!m_grabMaintained || (m_grabMaintained && !m_isInGrabState))) || m_isInThrowState)
        {
            m_grabDistance = m_initialGrabDistance;
            m_isInGrabState = false;
            return;
        }

        const bool prevGrabState = m_isInGrabState;

        // Changes distance between Grabbing Entity and Grabbed object. Minimum and maximum grab distances determined by m_minGrabDistance
        // and m_maxGrabDistance, respectively.
        m_grabDistance =
            AZ::GetClamp(m_grabDistance + (m_grabDistanceKeyValue * deltaTime * m_grabDistanceSpeed), m_minGrabDistance, m_maxGrabDistance);

        // Creates a reference point for the Grabbed Object translation in front of the Grabbing Entity. This can be thought of as the
        // "hand".
        m_grabReference = m_grabbingEntityPtr->GetTransform()->GetWorldTM();
        m_grabReference.SetTranslation(
            m_grabbingEntityPtr->GetTransform()->GetWorldTM().GetTranslation() + m_forwardVector * m_grabDistance);

        m_grabbedObjectTranslation = GetEntityPtr(m_lastGrabbedObjectEntityId)->GetTransform()->GetWorldTM().GetTranslation();

        // Set object's Grab state if m_grabEnableToggle is true
        if (m_grabEnableToggle && m_prevGrabKeyValue == 0.f && m_grabKeyValue != 0.f)
        {
            if (!m_isInGrabState)
            {
                // Check if Grabbed Object is a Dynamic Rigid Body when first interacting with it
                m_isInitialObjectKinematic = GetGrabbedObjectIsKinematic();

                // Store initial collision layer
                m_prevGrabbedCollisionLayer = GetCurrentGrabbedCollisionLayer();

                // Set Object Current Layer variable to Temp Layer
                SetCurrentGrabbedCollisionLayer(m_tempGrabbedCollisionLayer);

                // Store object's original Angular Damping value
                m_prevObjectAngularDamping = GetCurrentGrabbedObjectAngularDamping();
            }
            else
            {
                // Reset current grabbed distance to m_initialGrabDistance if grab key is not pressed
                m_grabDistance = m_initialGrabDistance;

                // Set Object Current Layer variable back to initial layer if Grab Key is not pressed
                SetCurrentGrabbedCollisionLayer(m_prevGrabbedCollisionLayer);

                // Set Angular Damping back to original value if Grab Key is not pressed
                SetCurrentGrabbedObjectAngularDamping(m_prevObjectAngularDamping);

                m_isInRotateState = false;
                m_hasRotated = false;
            }

            m_isInGrabState = !m_isInGrabState;
        }
        // Set object's Grab state if m_grabEnableToggle is false
        else if (m_prevGrabKeyValue != m_grabKeyValue && !m_grabEnableToggle)
        {
            if (m_grabKeyValue != 0.f)
            {
                // Check if Grabbed Object is a Dynamic Rigid Body when first interacting with it
                m_isInitialObjectKinematic = GetGrabbedObjectIsKinematic();

                // Store initial collision layer
                m_prevGrabbedCollisionLayer = GetCurrentGrabbedCollisionLayer();

                // Set Object Current Layer variable to Temp Layer
                SetCurrentGrabbedCollisionLayer(m_tempGrabbedCollisionLayer);

                m_isInGrabState = true;

                // Store object's original Angular Damping value
                m_prevObjectAngularDamping = GetCurrentGrabbedObjectAngularDamping();
            }
            else
            {
                // Reset current grabbed distance to m_initialGrabDistance if grab key is not pressed
                m_grabDistance = m_initialGrabDistance;

                // Set Object Current Layer variable back to Prev Layer if Grab Key is not pressed
                SetCurrentGrabbedCollisionLayer(m_prevGrabbedCollisionLayer);

                // Set Angular Damping back to original value if Grab Key is not pressed
                SetCurrentGrabbedObjectAngularDamping(m_prevObjectAngularDamping);

                m_isInGrabState = false;
                m_isInRotateState = false;
                m_hasRotated = false;
            }
        }

        // Set Grabbed Object to Dynamic Rigid Body if previously Kinematic
        if (!m_isInGrabState && (GetGrabbedObjectIsKinematic() && m_kinematicWhileGrabbing && !m_isInitialObjectKinematic))
        {
            SetGrabbedObjectIsKinematic(m_lastGrabbedObjectEntityId, false);
        }

        // Move the object using Translation (Transform) if it is a Kinematic Rigid Body
        if (m_isInGrabState && (m_kinematicWhileGrabbing || m_isInitialObjectKinematic))
        {
            // Set Grabbed Object to Kinematic Rigid Body if previously Dynamic
            if (!GetGrabbedObjectIsKinematic())
            {
                SetGrabbedObjectIsKinematic(m_lastGrabbedObjectEntityId, true);
            }

            // If object is NOT in rotate state, move object by setting its Translation directly. Only rotate object if it has been in
            // rotate state
            if (!m_isInRotateState)
            {
                GetEntityPtr(m_lastGrabbedObjectEntityId)->GetTransform()->SetWorldTranslation(m_grabReference.GetTranslation());
                // Keep Object rotation facing the grabbing entity if it has not be rotated. If it has been rotated, object will keep that
                // rotation while translating
                if (!m_hasRotated)
                {
                    GetEntityPtr(m_lastGrabbedObjectEntityId)
                        ->GetTransform()
                        ->SetWorldRotation(m_grabReference.GetRotation().GetEulerRadians());
                }
            }
            // If object is in rotate state, move object by setting its Translation, but do not rotate it
            else
            {
                GetEntityPtr(m_lastGrabbedObjectEntityId)->GetTransform()->SetWorldTranslation(m_grabReference.GetTranslation());
            }
        }
        // Move the object using SetLinearVelocity (PhysX) if it is a Dynamic Rigid Body
        else if (m_isInGrabState && !GetGrabbedObjectIsKinematic() && (!m_kinematicWhileGrabbing || !m_isInitialObjectKinematic))
        {
            // Subtract object's translation from our reference position, which gives you a vector pointing from the object to the
            // reference. Then apply a linear velocity to move the object toward the reference.
            Physics::RigidBodyRequestBus::Event(
                m_lastGrabbedObjectEntityId,
                &Physics::RigidBodyRequests::SetLinearVelocity,
                (m_grabReference.GetTranslation() - m_grabbedObjectTranslation) * m_grabResponse);
        }

        // Trigger an event notification when object enters Grab state or exits Grab state
        if (!prevGrabState && m_isInGrabState)
        {
            TestGemNotificationBus::Broadcast(&TestGemNotificationBus::Events::OnGrabObject);
        }
        else if (prevGrabState && !m_isInGrabState)
        {
            TestGemNotificationBus::Broadcast(&TestGemNotificationBus::Events::OnEndGrabObject);
        }
    }

    // Rotate object using physics or transforms, based on object's starting Rigid Body type, or if KinematicWhileGrabbing is enabled.
    // It is recommended to stop player character camera rotation while rotating an object.
    void Grab::RotateObject(const float& deltaTime)
    {
        // Do not execute the function if Rotate Key is not pressed, the object is not in Grab State, or if the object is being thrown
        if ((!m_rotateKeyValue && !m_isInRotateState) || !m_isInGrabState || m_isInThrowState)
        {
            return;
        }

        const bool prevRotateState = m_isInRotateState;

        // Get right vector relative to the grabbing entity's transform
        m_rightVector =
            AZ::Quaternion(m_grabbingEntityPtr->GetTransform()->GetWorldRotationQuaternion()).TransformVector(AZ::Vector3::CreateAxisX());

        // Get up vector relative to the grabbing entity's transform
        m_upVector =
            AZ::Quaternion(m_grabbingEntityPtr->GetTransform()->GetWorldRotationQuaternion()).TransformVector(AZ::Vector3::CreateAxisZ());

        // Set object's Rotate state if m_rotateEnableToggle is true
        if (m_rotateEnableToggle && m_prevRotateKeyValue == 0.f && m_rotateKeyValue != 0.f)
        {
            if (!m_isInRotateState)
            {
                // Store object's original Angular Damping value before rotating
                m_prevObjectAngularDamping = GetCurrentGrabbedObjectAngularDamping();
                // Set new Angular Damping before rotating object
                SetCurrentGrabbedObjectAngularDamping(m_tempObjectAngularDamping);
            }
            else if (m_isInRotateState)
            {
                // Set Angular Damping back to original value
                SetCurrentGrabbedObjectAngularDamping(m_prevObjectAngularDamping);
            }
            m_isInRotateState = !m_isInRotateState;
        }
        // Set object's Rotate state if m_rotateEnableToggle is false
        else if (m_prevRotateKeyValue != m_rotateKeyValue && !m_rotateEnableToggle)
        {
            if (m_rotateKeyValue != 0.f)
            {
                // Store initial Angular Damping value
                m_prevObjectAngularDamping = GetCurrentGrabbedObjectAngularDamping();

                // Set new Angular Damping before rotating object
                SetCurrentGrabbedObjectAngularDamping(m_tempObjectAngularDamping);

                m_isInRotateState = true;
            }
            else
            {
                // Set Angular Damping back to original value
                SetCurrentGrabbedObjectAngularDamping(m_prevObjectAngularDamping);

                m_isInRotateState = false;
            }
        }

        // Rotate the object using SetAngularVelocity (PhysX) if it is a Dynamic Rigid Body
        if (!m_kinematicWhileGrabbing && !m_isInitialObjectKinematic)
        {
            AZ::Vector3 currentAngularVelocity = AZ::Vector3::CreateZero();

            currentAngularVelocity = GetGrabbedObjectAngularVelocity();

            m_delta_yaw = m_upVector * m_yawKeyValue;
            m_delta_pitch = m_rightVector * m_pitchKeyValue;
            m_delta_roll = m_forwardVector * m_rollKeyValue;

            m_delta_yaw *= m_dynamicRotateScale;
            m_delta_pitch *= m_dynamicRotateScale;
            m_delta_roll *= m_dynamicRotateScale;

            AZ::Vector3 newAngularVelocity = AZ::Vector3::CreateZero();

            newAngularVelocity = currentAngularVelocity + (m_delta_yaw + m_delta_pitch + m_delta_roll);

            SetGrabbedObjectAngularVelocity(newAngularVelocity);

            m_hasRotated = true;
        }
        // Rotate the object using SetRotation (Transform) if it is a Kinematic Rigid Body
        else
        {
            AZ::Vector3 current_Rotation = GetEntityPtr(m_lastGrabbedObjectEntityId)->GetTransform()->GetLocalRotation();

            m_delta_yaw = AZ::Vector3::CreateAxisZ(m_yawKeyValue);
            m_delta_pitch = AZ::Vector3::CreateAxisX(m_pitchKeyValue);
            m_delta_roll = AZ::Vector3::CreateAxisY(m_rollKeyValue);

            m_delta_yaw *= m_kinematicRotateScale;
            m_delta_pitch *= m_kinematicRotateScale;
            m_delta_pitch *= m_kinematicRotateScale;

            AZ::Vector3 newRotation = AZ::Vector3::CreateZero();
            newRotation = current_Rotation + ((m_delta_yaw + m_delta_pitch + m_delta_roll) * deltaTime);

            GetEntityPtr(m_lastGrabbedObjectEntityId)->GetTransform()->SetLocalRotation(newRotation);

            m_hasRotated = true;
        }

        // Set Angular Velocity to zero when no longer rotating
        if (!m_isInRotateState)
        {
            SetGrabbedObjectAngularVelocity(AZ::Vector3::CreateZero());
        }

        // Trigger an event notification when object enters Rotate State or exits Rotate state
        if (!prevRotateState && m_isInRotateState)
        {
            TestGemNotificationBus::Broadcast(&TestGemNotificationBus::Events::OnRotateObject);
        }
        else if (prevRotateState && !m_isInRotateState)
        {
            TestGemNotificationBus::Broadcast(&TestGemNotificationBus::Events::OnEndRotateObject);
        }
    }
    // Apply linear impulse to object if it is a Dynamic Rigid Body
    void Grab::ThrowObject(const float& deltaTime)
    {
        // Do not execute the function if the object is not in Grab State, or if the object is initially a Kinematic Rigid Body
        if ((!m_isInGrabState && !m_isInThrowState) || m_isInitialObjectKinematic)
        {
            return;
        }

        // Set Kinematic Rigid Body to Dynamic in order to prevent PhysX Rigid Body Warning in console when throwing object while rotating.
        // This only happens temporarily, as the object will be set to Dynamic in order to throw.
        if (m_throwKeyValue && GetGrabbedObjectIsKinematic())
        {
            SetGrabbedObjectIsKinematic(m_lastGrabbedObjectEntityId, false);
        }

        if (m_throwKeyValue && !m_isInThrowState)
        {
            m_throwStateCounter = m_throwStateMaxTime;

            // Apply a Linear Impulse to the grabbed object.
            Physics::RigidBodyRequestBus::Event(
                m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequestBus::Events::ApplyLinearImpulse, m_forwardVector * m_throwImpulse);

            // Trigger an event notification when object enters Throw State
            TestGemNotificationBus::Broadcast(&TestGemNotificationBus::Events::OnThrowObject);

            m_isInThrowState = true;
            m_isInGrabState = false;
            m_isInRotateState = false;
            m_hasRotated = false;

            m_thrownGrabbedObjectEntityId = m_lastGrabbedObjectEntityId;

            // Set Object Current Layer variable back to Prev Layer if in throw state
            SetCurrentGrabbedCollisionLayer(m_prevGrabbedCollisionLayer);

            // Set Angular Damping back to original value
            SetCurrentGrabbedObjectAngularDamping(m_prevObjectAngularDamping);
        }

        if (m_isInThrowState)
        {
            m_throwStateCounter -= deltaTime;

            // Set throw state to false if the thrown grabbed object is more than the distance of m_sphereCastDistance away.
            if (m_grabReference.GetTranslation().GetDistance(
                    GetEntityPtr(m_thrownGrabbedObjectEntityId)->GetTransform()->GetWorldTM().GetTranslation()) > m_sphereCastDistance)
            {
                m_isInThrowState = false;
                TestGemNotificationBus::Broadcast(&TestGemNotificationBus::Events::OnMaxThrowDistance);
            }
            // Set throw state to false if grabbed object is in throw state longer than m_throwStateMaxTime
            else if (m_throwStateCounter <= 0.f)
            {
                m_isInThrowState = false;
                TestGemNotificationBus::Broadcast(&TestGemNotificationBus::Events::OnThrowStateCounterZero);
            }
        }
    }

    // Event Notification methods for use in scripts
    void Grab::OnObjectSphereCastHit()
    {
    }
    void Grab::OnGrabObject()
    {
    }
    void Grab::OnEndGrabObject()
    {
    }
    void Grab::OnRotateObject()
    {
    }
    void Grab::OnEndRotateObject()
    {
    }
    void Grab::OnThrowObject()
    {
    }
    void Grab::OnMaxThrowDistance()
    {
    }
    void Grab::OnThrowStateCounterZero()
    {
    }

    // Request Bus getter and setter methods for use in scripts
    AZ::EntityId Grab::GetGrabbingEntityId() const
    {
        return m_grabbingEntityPtr->GetId();
    }

    AZ::EntityId Grab::GetActiveCameraEntityId() const
    {
        return GetActiveCameraEntityPtr()->GetId();
    }

    AZ::EntityId Grab::GetGrabbedObjectEntityId() const
    {
        return m_grabbedObjectEntityId;
    }

    AZ::EntityId Grab::GetLastGrabbedObjectEntityId() const
    {
        return m_lastGrabbedObjectEntityId;
    }

    AZ::EntityId Grab::GetThrownGrabbedObjectEntityId() const
    {
        return m_thrownGrabbedObjectEntityId;
    }

    void Grab::SetGrabbingEntity(const AZ::EntityId new_grabbingEntityId)
    {
        m_grabbingEntityPtr = GetEntityPtr(new_grabbingEntityId);
    }

    bool Grab::GetIsInGrabState() const
    {
        return m_isInGrabState;
    }

    bool Grab::GetIsInThrowState() const
    {
        return m_isInThrowState;
    }

    bool Grab::GetIsInRotateState() const
    {
        return m_isInRotateState;
    }

    bool Grab::GetObjectSphereCastHit() const
    {
        return m_objectSphereCastHit;
    }

    float Grab::GetGrabbedObjectDistance() const
    {
        return m_grabDistance;
    }

    void Grab::SetGrabbedObjectDistance(const float& new_grabDistance)
    {
        m_grabDistance = AZ::GetClamp(new_grabDistance, m_minGrabDistance, m_maxGrabDistance);
    }

    float Grab::GetMinGrabbedObjectDistance() const
    {
        return m_minGrabDistance;
    }

    void Grab::SetMinGrabbedObjectDistance(const float& new_minGrabDistance)
    {
        m_minGrabDistance = new_minGrabDistance;
    }

    float Grab::GetMaxGrabbedObjectDistance() const
    {
        return m_maxGrabDistance;
    }

    void Grab::SetMaxGrabbedObjectDistance(const float& new_maxGrabDistance)
    {
        m_maxGrabDistance = new_maxGrabDistance;
    }

    float Grab::GetInitialGrabbedObjectDistance() const
    {
        return m_initialGrabDistance;
    }

    void Grab::SetInitialGrabbedObjectDistance(const float& new_initialGrabDistance)
    {
        m_initialGrabDistance = new_initialGrabDistance;
    }

    float Grab::GetGrabbedObjectDistanceSpeed() const
    {
        return m_grabDistanceSpeed;
    }

    void Grab::SetGrabbedObjectDistanceSpeed(const float& new_grabDistanceSpeed)
    {
        m_grabDistanceSpeed = new_grabDistanceSpeed;
    }

    float Grab::GetGrabResponse() const
    {
        return m_grabResponse;
    }

    void Grab::SetGrabResponse(const float& new_grabResponse)
    {
        m_grabResponse = new_grabResponse;
    }

    float Grab::GetDynamicRotateScale() const
    {
        return m_dynamicRotateScale;
    }

    void Grab::SetDynamicRotateScale(const float& new_dynamicRotateScale)
    {
        m_dynamicRotateScale = new_dynamicRotateScale;
    }

    float Grab::GetKinematicRotateScale() const
    {
        return m_kinematicRotateScale;
    }

    void Grab::SetKinematicRotateScale(const float& new_kinematicRotateScale)
    {
        m_kinematicRotateScale = new_kinematicRotateScale;
    }

    float Grab::GetThrowImpulse() const
    {
        return m_throwImpulse;
    }

    void Grab::SetThrowImpulse(const float& new_throwImpulse)
    {
        m_throwImpulse = new_throwImpulse;
    }

    float Grab::GetGrabbedObjectThrowStateCounter() const
    {
        return m_throwStateCounter;
    }

    float Grab::GetGrabbedObjectThrowStateTime() const
    {
        return m_throwStateMaxTime;
    }

    void Grab::SetGrabbedObjectThrowStateTime(const float& new_throwStateMaxTime)
    {
        m_throwStateMaxTime = new_throwStateMaxTime;
    }

    float Grab::GetSphereCastRadius() const
    {
        return m_sphereCastRadius;
    }

    void Grab::SetSphereCastRadius(const float& new_sphereCastRadius)
    {
        m_sphereCastRadius = new_sphereCastRadius;
    }

    float Grab::GetSphereCastDistance() const
    {
        return m_sphereCastDistance;
    }

    void Grab::SetSphereCastDistance(const float& new_sphereCastDistance)
    {
        m_sphereCastDistance = new_sphereCastDistance;
    }

    AZStd::string Grab::GetGrabbedCollisionGroup() const
    {
        AZStd::string groupName;
        Physics::CollisionRequestBus::BroadcastResult(
            groupName, &Physics::CollisionRequests::GetCollisionGroupName, m_grabbedCollisionGroup);
        return groupName;
    }

    void Grab::SetGrabbedCollisionGroup(const AZStd::string& new_grabbedCollisionGroupName)
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

    AZStd::string Grab::GetCurrentGrabbedCollisionLayerName() const
    {
        AZStd::string currentGrabbedCollisionLayerName;
        Physics::CollisionFilteringRequestBus::EventResult(
            currentGrabbedCollisionLayerName,
            m_lastGrabbedObjectEntityId,
            &Physics::CollisionFilteringRequestBus::Events::GetCollisionLayerName);
        return currentGrabbedCollisionLayerName;
    }

    void Grab::SetCurrentGrabbedCollisionLayerByName(const AZStd::string& new_currentGrabbedCollisionLayerName)
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

    AzPhysics::CollisionLayer Grab::GetCurrentGrabbedCollisionLayer() const
    {
        AZStd::string grabbedCollisionLayerName;
        Physics::CollisionFilteringRequestBus::EventResult(
            grabbedCollisionLayerName, m_lastGrabbedObjectEntityId, &Physics::CollisionFilteringRequestBus::Events::GetCollisionLayerName);
        AzPhysics::CollisionLayer grabbedCollisionLayer;
        Physics::CollisionRequestBus::BroadcastResult(
            grabbedCollisionLayer, &Physics::CollisionRequests::GetCollisionLayerByName, grabbedCollisionLayerName);
        return grabbedCollisionLayer;
    }

    void Grab::SetCurrentGrabbedCollisionLayer(const AzPhysics::CollisionLayer& new_currentGrabbedCollisionLayer)
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

    AZStd::string Grab::GetPrevGrabbedCollisionLayerName() const
    {
        const AzPhysics::CollisionConfiguration& configuration =
            AZ::Interface<AzPhysics::SystemInterface>::Get()->GetConfiguration()->m_collisionConfig;
        return configuration.m_collisionLayers.GetName(m_prevGrabbedCollisionLayer);
    }

    void Grab::SetPrevGrabbedCollisionLayerByName(const AZStd::string& new_prevGrabbedCollisionLayerName)
    {
        bool success = false;
        AzPhysics::CollisionLayer prevGrabbedCollisionLayer;
        Physics::CollisionRequestBus::BroadcastResult(
            success, &Physics::CollisionRequests::TryGetCollisionLayerByName, new_prevGrabbedCollisionLayerName, prevGrabbedCollisionLayer);
        if (success)
            m_prevGrabbedCollisionLayer = prevGrabbedCollisionLayer;
    }

    AzPhysics::CollisionLayer Grab::GetPrevGrabbedCollisionLayer() const
    {
        return m_prevGrabbedCollisionLayer;
    }

    void Grab::SetPrevGrabbedCollisionLayer(const AzPhysics::CollisionLayer& new_prevGrabbedCollisionLayer)
    {
        m_prevGrabbedCollisionLayer = new_prevGrabbedCollisionLayer;
    }

    AZStd::string Grab::GetTempGrabbedCollisionLayerName() const
    {
        return m_tempGrabbedCollisionLayerName;
    }

    void Grab::SetTempGrabbedCollisionLayerByName(const AZStd::string& new_tempGrabbedCollisionLayerName)
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

    AzPhysics::CollisionLayer Grab::GetTempGrabbedCollisionLayer() const
    {
        return m_tempGrabbedCollisionLayer;
    }

    void Grab::SetTempGrabbedCollisionLayer(const AzPhysics::CollisionLayer& new_tempGrabbedCollisionLayer)
    {
        m_tempGrabbedCollisionLayer = new_tempGrabbedCollisionLayer;
        const AzPhysics::CollisionConfiguration& configuration =
            AZ::Interface<AzPhysics::SystemInterface>::Get()->GetConfiguration()->m_collisionConfig;
        m_tempGrabbedCollisionLayerName = configuration.m_collisionLayers.GetName(m_tempGrabbedCollisionLayer);
    }

    bool Grab::GetGrabbedObjectIsKinematic() const
    {
        bool isObjectKinematic = false;
        Physics::RigidBodyRequestBus::EventResult(
            isObjectKinematic, m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequestBus::Events::IsKinematic);

        return isObjectKinematic;
    }

    void Grab::SetGrabbedObjectIsKinematic(AZ::EntityId objectId, const bool& isKinematic)
    {
        Physics::RigidBodyRequestBus::Event(objectId, &Physics::RigidBodyRequests::SetKinematic, isKinematic);

        m_isObjectKinematic = isKinematic;
    }

    bool Grab::GetInitialGrabbedObjectIsKinematic() const
    {
        return m_isInitialObjectKinematic;
    }

    float Grab::GetCurrentGrabbedObjectAngularDamping() const
    {
        float currentObjectAngularDamping = 0.f;

        Physics::RigidBodyRequestBus::EventResult(
            currentObjectAngularDamping, m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::GetAngularDamping);

        return currentObjectAngularDamping;
    }

    void Grab::SetCurrentGrabbedObjectAngularDamping(const float& new_currentObjectAngularDamping)
    {
        m_currentObjectAngularDamping = new_currentObjectAngularDamping;

        Physics::RigidBodyRequestBus::Event(
            m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::SetAngularDamping, new_currentObjectAngularDamping);
    }

    float Grab::GetPrevGrabbedObjectAngularDamping() const
    {
        return m_prevObjectAngularDamping;
    }

    void Grab::SetPrevGrabbedObjectAngularDamping(const float& new_prevObjectAngularDamping)
    {
        m_prevObjectAngularDamping = new_prevObjectAngularDamping;
    }

    float Grab::GetTempGrabbedObjectAngularDamping() const
    {
        return m_tempObjectAngularDamping;
    }

    void Grab::SetTempGrabbedObjectAngularDamping(const float& new_tempObjectAngularDamping)
    {
        m_tempObjectAngularDamping = new_tempObjectAngularDamping;
        Physics::RigidBodyRequestBus::Event(
            m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::SetAngularDamping, m_tempObjectAngularDamping);
    }

    AZ::Vector3 Grab::GetGrabbedObjectAngularVelocity() const
    {
        AZ::Vector3 grabbedObjectAngularVelocity = AZ::Vector3::CreateZero();

        Physics::RigidBodyRequestBus::EventResult(
            grabbedObjectAngularVelocity, m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::GetAngularVelocity);
        return grabbedObjectAngularVelocity;
    }

    void Grab::SetGrabbedObjectAngularVelocity(const AZ::Vector3& new_grabbedObjectAngularVelocity)
    {
        m_grabbedObjectAngularVelocity = new_grabbedObjectAngularVelocity;

        Physics::RigidBodyRequestBus::Event(
            m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::SetAngularVelocity, m_grabbedObjectAngularVelocity);
    }
} // namespace TestGem