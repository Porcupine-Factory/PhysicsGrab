#include "Grab.h"

#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TransformBus.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzFramework/Physics/CollisionBus.h>
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

                ->Field("Use Camera As Grabbing Entity", &Grab::m_useCameraAsGrabbingEntity)
                ->Field("GrabbingEntityId", &Grab::m_grabbingEntityId)
                ->Field("Sphere Cast Radius", &Grab::m_sphereCastRadius)
                ->Field("Sphere Cast Distance", &Grab::m_sphereCastDistance)
                ->Field("Default Grab Distance", &Grab::m_grabInitialDistance)
                ->Field("Min Grab Distance", &Grab::m_minGrabDistance)
                ->Field("Max Grab Distance", &Grab::m_maxGrabDistance)
                ->Field("Grab Distance Speed", &Grab::m_grabDistanceSpeed)
                ->Field("Throw Strength", &Grab::m_throwStrength)
                ->Field("Grab Strength", &Grab::m_grabStrength)
                ->Field("Rotate Scale", &Grab::m_rotateScale)
                ->Field("Rotate Enable Toggle", &Grab::m_rotateEnableToggle)
                ->Field("Grab Object Collision Group", &Grab::m_grabCollisionGroupId)
                ->Version(1);

            if (AZ::EditContext* ec = sc->GetEditContext())
            {
                using namespace AZ::Edit;
                ec->Class<Grab>(
                    "Grab",
                    "[Grab Interaction Component]")
                    ->ClassElement(ClassElements::EditorData, "")
                    ->Attribute(
                        Attributes::AppearsInAddComponentMenu,
                        AZ_CRC_CE("Game"))

                    // Input Binding Keys
                    ->DataElement(nullptr,
                        &Grab::m_strGrab,
                        "Grab Key", "Grab interaction input binding")
                    ->DataElement(nullptr,
                        &Grab::m_strGrabDistance,
                        "Grab Distance Key", "Grab distance input binding")
                    ->DataElement(nullptr,
                        &Grab::m_strThrow,
                        "Throw Input Key", "Throw object input binding")
                    ->DataElement(nullptr,
                        &Grab::m_strRotate,
                        "Rotate Enable Key", "Enable rotate object input binding")
                    ->DataElement(nullptr,
                        &Grab::m_strRotatePitch,
                        "Rotate Pitch Key", "Rotate object about X axis input binding")
                    ->DataElement(nullptr,
                        &Grab::m_strRotateYaw,
                        "Rotate Yaw Key", "Rotate object about Z axis input binding")


                    ->DataElement(nullptr,
                        &Grab::m_useCameraAsGrabbingEntity,
                        "Use Camera As Grabbing Entity", "Sets Active Camera as the default Grabbing Entity.")
                    ->DataElement(0,
                        &Grab::m_grabbingEntityId, "Grab Entity", "Reference entity for initiating Grab spherecast. This would be the Camera Entity for a typical first-person game.")
                    ->DataElement(nullptr,
                        &Grab::m_throwStrength,
                        "Throw Strength", "Linear Impulse scale applied when throwing grabbed object")
                    ->DataElement(nullptr,
                        &Grab::m_grabStrength,
                        "Grab Strength", "Linear velocity scale applied when holding grabbed object")
                    ->DataElement(nullptr,
                        &Grab::m_rotateScale,
                        "Rotate Scale", "Angular Velocity scale applied when rotating object")
                    ->DataElement(nullptr,
                        &Grab::m_rotateEnableToggle,
                        "Rotate Enable Toggle", "Determines whether pressing Rotate Key toggles Rotate mode. Disabling this requires the Rotate key to be held to maintain Rotate mode.")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Sphere Cast Parameters")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(nullptr,
                        &Grab::m_grabCollisionGroupId,
                        "Sphere Cast Collision Group", "The collision group which will be used for the grabbing objects.")
                    ->DataElement(nullptr,
                        &Grab::m_sphereCastRadius,
                        "Sphere Cast Radius", "Sphere Cast radius used for grabbing objects")
                    ->DataElement(nullptr,
                        &Grab::m_sphereCastDistance,
                        "Sphere Cast Distance", "Sphere Cast distance along m_sphereCastDirection")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Grab Distances")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(nullptr,
                        &Grab::m_grabInitialDistance,
                        "Default Grab Distance", "Distance the grabbed object will default to when letting go of the object")
                    ->DataElement(nullptr,
                        &Grab::m_minGrabDistance,
                        "Min Grab Distance", "Minimum allowable grab distance. Grabbed object cannot get closer than this distance.")
                    ->DataElement(nullptr,
                        &Grab::m_maxGrabDistance,
                        "Max Grab Distance", "Maximum allowable grab distance. Grabbed object cannot get further than this distance.")
                    ->DataElement(nullptr,
                        &Grab::m_grabDistanceSpeed,
                        "Grab Distance Speed", "The speed at which you move the grabbed object closer or away.");
            }
        }

        if (auto bc = azrtti_cast<AZ::BehaviorContext*>(rc))
        {
            //bc->EBus<GrabNotificationBus>("GrabNotificationBus")
            //    ->Handler<GrabNotificationHandler>();

            bc->EBus<TestGemComponentRequestBus>("TestGemComponentRequestBus")
                ->Attribute(AZ::Script::Attributes::Scope, AZ::Script::Attributes::ScopeFlags::Common)
                ->Attribute(AZ::Script::Attributes::Module, "interaction")
                ->Attribute(AZ::Script::Attributes::Category, "Grab")
                ->Event("Get isThrowing", &TestGemComponentRequests::GetisThrowing)
                ->Event("Get isRotating", &TestGemComponentRequests::GetisRotating)
                ->Event("Get Grab Object Distance", &TestGemComponentRequests::GetGrabObjectDistance);

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

        Camera::CameraNotificationBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();

        Physics::CollisionRequestBus::BroadcastResult(
            m_grabCollisionGroup, &Physics::CollisionRequests::GetCollisionGroupById, m_grabCollisionGroupId);

        // Connect the handler to the request bus
        TestGemComponentRequestBus::Handler::BusConnect(GetEntityId());

        AZ::EntityBus::Handler::BusConnect(m_grabbingEntityId);
    }

    void Grab::OnEntityActivated([[maybe_unused]] const AZ::EntityId& entityId)
    {
        AZ::EntityBus::Handler::BusDisconnect();

        if (!m_useCameraAsGrabbingEntity)
        {
            m_grabbingEntityPtr = GetEntityPtr(m_grabbingEntityId);
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
    }

    void Grab::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("GrabService"));
    }

    void Grab::OnCameraAdded(const AZ::EntityId& cameraId)
    {
        if (m_useCameraAsGrabbingEntity)
            m_grabbingEntityPtr = GetEntityPtr(cameraId);
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
            //AZ_Printf("Player", "Grab value %f", value);
        }

        if (*inputId == m_grabDistanceEventId)
        {
            m_grabDistanceKeyValue = value;
            //AZ_Printf("Object", "Grab Distance value %f", value);
        }

        if (*inputId == m_throwEventId)
        {
            m_throwKeyValue = value;
            //AZ_Printf("Player", "Throw value %f", value);
        }

        if (*inputId == m_rotateEventId)
        {
            m_rotateKeyValue = value;
            //AZ_Printf("Player", "rotate value %f", value);
        }

        if (*inputId == m_rotatePitchEventId)
        {
            m_pitchKeyValue = value;
            //AZ_Printf("Object", "Grab Object pitch value %f", value);
        }

        if (*inputId == m_rotateYawEventId)
        {
            m_yawKeyValue = value;
            //AZ_Printf("Object", "Grab Object yaw value %f", value);
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
            //AZ_Printf("Player", "Grab released value %f", value);
        }
        if (*inputId == m_grabDistanceEventId)
        {
            m_grabDistanceKeyValue = value;
            //AZ_Printf("Player", "Grab Distance released value %f", value);
        }
        if (*inputId == m_throwEventId)
        {
            m_throwKeyValue = value;
            //AZ_Printf("Player", "Throw released value %f", value);
        }
        if (*inputId == m_rotateEventId)
        {
            m_rotateKeyValue = value;
            //AZ_Printf("Player", "Throw released value %f", value);
        }

        if (*inputId == m_rotatePitchEventId)
        {
            m_pitchKeyValue = value;
            //AZ_Printf("Object", "Grab Object pitch released value %f", value);
        }

        if (*inputId == m_rotateYawEventId)
        {
            m_yawKeyValue = value;
            //AZ_Printf("Object", "Grab Object yaw released value %f", value);
        }
    }
    void Grab::OnHeld(float value)
    {
        const InputEventNotificationId* inputId =
            InputEventNotificationBus::GetCurrentBusId();
        if (inputId == nullptr)
        {
            return;
        }
        if (*inputId == m_throwEventId)
        {
            m_throwKeyValue = value;
            //AZ_Printf("Player", "Throw held value %f", value);
        }

        if (*inputId == m_rotatePitchEventId)
        {
            m_pitchKeyValue = value;
            //AZ_Printf("Object", "Grab Object pitch held value %f", value);
        }

        else if (*inputId == m_rotateYawEventId)
        {
            m_yawKeyValue = value;
            //AZ_Printf("Object", "Grab Object yaw held value %f", value);
        }
    }

    void Grab::OnTick(float deltaTime, AZ::ScriptTimePoint)
    {
        // AZ_Printf("", "m_isThrowing = %s", isThrowing ? "true" : "false");
        // AZ_Printf("", "m_isRotating = %s", m_isRotating ? "true" : "false");
        // AZ_Printf("", "m_objectReset = %s", m_objectReset ? "true" : "false");
        // AZ_Printf("", "m_rotatePrevValue = %s", m_rotatePrevValue ? "true" : "false");
        CheckForObjects(deltaTime);
    }

    AZ::Entity* Grab::GetEntityPtr(AZ::EntityId pointer) const
    {
        auto ca = AZ::Interface<AZ::ComponentApplicationRequests>::Get();
        return ca->FindEntity(pointer);
    }

    void Grab::CheckForObjects(const float& deltaTime)
    {
        // Do not perform spherecast query if grab key is not pressed
        if (!m_grabKeyValue)
        {
            // Reset current grabbed distance to m_grabInitialDistance if grab key is not pressed
            m_grabDistance = m_grabInitialDistance;

            m_isThrowing = false;
            m_isRotating = false;

            // Might need a way to check if m_lastGrabbedObject has a valid ID as to not get null reference crash
            Physics::RigidBodyRequestBus::EventResult(m_isObjectKinematic, m_lastGrabbedObject,
                &Physics::RigidBodyRequestBus::Events::IsKinematic);

            if (m_isObjectKinematic)
            {
                Physics::RigidBodyRequestBus::Event(m_lastGrabbedObject,
                    &Physics::RigidBodyRequests::SetKinematic,
                    false);
            }
            return;
        }

        // Get our local forward vector relative to the camera transform
        m_forwardVector = AZ::Quaternion(m_grabbingEntityPtr->GetTransform()->GetWorldRotationQuaternion()).TransformVector(AZ::Vector3::CreateAxisY());

        // Get our Camera's world transform
        m_cameraTransform = m_grabbingEntityPtr->GetTransform()->GetWorldTM();

        // Perform a spherecast query to check if colliding with object
        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();

        AzPhysics::ShapeCastRequest request = AzPhysics::ShapeCastRequestHelpers::CreateSphereCastRequest(m_sphereCastRadius,
            m_cameraTransform,
            m_forwardVector,
            m_sphereCastDistance,
            AzPhysics::SceneQuery::QueryType::Dynamic,
            m_grabCollisionGroup,
            nullptr);

        AzPhysics::SceneHandle sceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        AzPhysics::SceneQueryHits hits = sceneInterface->QueryScene(sceneHandle, &request);

        m_grabbedObjectEntityIds.clear();

        if (!hits)
        {
            m_isThrowing = false;
            m_isRotating = false;

            Physics::RigidBodyRequestBus::EventResult(m_isObjectKinematic, m_lastGrabbedObject,
                &Physics::RigidBodyRequestBus::Events::IsKinematic);

            if (m_isObjectKinematic)
            {
                Physics::RigidBodyRequestBus::Event(m_lastGrabbedObject,
                    &Physics::RigidBodyRequests::SetKinematic,
                    false);
            }
            return;
        }

        // Takes the first object hit in m_hits vector and assigns it to m_grabbedObjectEntityIds vector.
        m_grabbedObjectEntityIds.push_back(hits.m_hits.at(0).m_entityId);

        m_lastGrabbedObject = m_grabbedObjectEntityIds.at(0);

        if ((m_rotateKeyValue != 0 || m_isRotating) && !m_isThrowing)
        {
            RotateObject(m_grabbedObjectEntityIds.at(0), deltaTime);
        }

        else
        {
            m_objectReset = false;

            Physics::RigidBodyRequestBus::Event(m_grabbedObjectEntityIds.at(0),
                &Physics::RigidBodyRequests::SetKinematic,
                false);

            m_grabDistance = AZ::GetClamp(m_grabDistance + (m_grabDistanceKeyValue * deltaTime * m_grabDistanceSpeed), m_minGrabDistance, m_maxGrabDistance);
        }

        m_rotatePrevValue = m_rotateKeyValue;
        
        m_grabReference = m_grabbingEntityPtr->GetTransform()->GetWorldTM();
        m_grabReference.SetTranslation(m_grabbingEntityPtr->GetTransform()->GetWorldTM().GetTranslation() + m_forwardVector * m_grabDistance);

        m_grabbedObjectTranslation = GetEntityPtr(m_grabbedObjectEntityIds.at(0))->GetTransform()->GetWorldTM().GetTranslation();

        if (m_throwKeyValue)
        {
            // May see PhysX Rigid Body Warning in console when throwing object while rotating. This only happens temporarily, as the object will be set to Dynamic in order to throw.
            ThrowObject(m_grabbedObjectEntityIds.at(0), deltaTime);
        }

        else if (!m_throwKeyValue && !m_isThrowing)
        {
            HoldObject(m_grabbedObjectEntityIds.at(0), deltaTime);
        }
    }

    void Grab::HoldObject(AZ::EntityId objectId, const float& deltaTime)
    {
        // Subtract object's translation from our reference position, which gives you a vector pointing from the object to the reference. Then apply a linear velocity to move the object toward the reference. 
        Physics::RigidBodyRequestBus::Event(objectId,
            &Physics::RigidBodyRequests::SetLinearVelocity,
            (m_grabReference.GetTranslation() - m_grabbedObjectTranslation) * m_grabStrength * deltaTime);
    }

    void Grab::ThrowObject(AZ::EntityId objectId, const float& deltaTime)
    {
        m_isThrowing = true;

        // Apply a Linear Impulse to our held object.
        Physics::RigidBodyRequestBus::Event(objectId,
            &Physics::RigidBodyRequestBus::Events::ApplyLinearImpulse,
            m_forwardVector * m_throwStrength * deltaTime);
    }

    void Grab::RotateObject(AZ::EntityId objectId, const float& deltaTime)
    {
        // It is recommended to stop player character camera rotation while rotating an object.

        if (m_rotateEnableToggle && m_rotatePrevValue == 0.f && m_rotateKeyValue != 0.f)
        {
            m_isRotating = !m_isRotating;
        }

        else if (!m_rotateEnableToggle)
        {
            if (m_rotateKeyValue != 0.f)
            {
                m_isRotating = true;
            }

            else
            {
                m_isRotating = false;
            }
        }

        if (!m_objectReset)
        {
            m_objectReset = true;

            Physics::RigidBodyRequestBus::Event(objectId,
                &Physics::RigidBodyRequests::SetKinematic,
                true);
        }

        else
        {
            AZ::Vector3 current_Rotation = GetEntityPtr(objectId)->GetTransform()->GetLocalRotation();

            m_delta_yaw = AZ::Vector3(0.f, 0.f, m_yawKeyValue);
            m_delta_pitch = AZ::Vector3(m_pitchKeyValue, 0.f, 0.f);

            m_delta_yaw *= m_rotateScale;
            m_delta_pitch *= m_rotateScale;

            AZ::Vector3 new_Rotation = AZ::Vector3::CreateZero();
            new_Rotation = current_Rotation + ((m_delta_yaw + m_delta_pitch) * deltaTime);

            GetEntityPtr(objectId)->GetTransform()->SetLocalRotation(new_Rotation);
        }
    }
    bool Grab::GetisThrowing() const
    {
        return m_isThrowing;
    }

    bool Grab::GetisRotating() const
    {
        return m_isRotating;
    }

    float Grab::GetGrabObjectDistance() const
    {
        return m_grabDistance;
    }
}