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

                ->Field("GrabbingEntityId", &Grab::m_grabbingEntityId)
                ->Field("Grab Enable Toggle", &Grab::m_grabEnableToggle)
                ->Field("Kinematic Grabbed Object", &Grab::m_kinematicDefaultEnable)
                ->Field("Rotate Enable Toggle", &Grab::m_rotateEnableToggle)
                ->Field("Sphere Cast Radius", &Grab::m_sphereCastRadius)
                ->Field("Sphere Cast Distance", &Grab::m_sphereCastDistance)
                ->Field("Default Grab Distance", &Grab::m_initialGrabDistance)
                ->Field("Min Grab Distance", &Grab::m_minGrabDistance)
                ->Field("Max Grab Distance", &Grab::m_maxGrabDistance)
                ->Field("Grab Distance Speed", &Grab::m_grabDistanceSpeed)
                ->Field("Throw Strength", &Grab::m_throwStrength)
                ->Field("Grab Strength", &Grab::m_grabStrength)
                ->Field("Rotate Scale", &Grab::m_rotateScale)
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


                    ->DataElement(0,
                        &Grab::m_grabbingEntityId, "Grab Entity", "Reference entity that interacts with objects. If left blank, Camera entity will be used by default.")

                    // Input Binding Keys
                    ->ClassElement(AZ::Edit::ClassElements::Group, "Input Bindings")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
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


                    ->ClassElement(AZ::Edit::ClassElements::Group, "Toggle Preferences")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(nullptr,
                        &Grab::m_grabEnableToggle,
                        "Grab Enable Toggle", "Determines whether pressing Grab Key toggles Grab mode. Disabling this requires the Grab key to be held to maintain Grab mode.")
                    ->DataElement(nullptr,
                        &Grab::m_kinematicDefaultEnable,
                        "Kinematic Grabbed Object", "Sets the grabbed object to kinematic.")
                    ->DataElement(nullptr,
                        &Grab::m_rotateEnableToggle,
                        "Rotate Enable Toggle", "Determines whether pressing Rotate Key toggles Rotate mode. Disabling this requires the Rotate key to be held to maintain Rotate mode.")
                    
                    
                    ->ClassElement(AZ::Edit::ClassElements::Group, "Scaling Factors")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(nullptr,
                        &Grab::m_throwStrength,
                        "Throw Strength", "Linear Impulse scale applied when throwing grabbed object")
                    ->DataElement(nullptr,
                        &Grab::m_grabStrength,
                        "Grab Strength", "Linear velocity scale applied when holding grabbed object")
                    ->DataElement(nullptr,
                        &Grab::m_rotateScale,
                        "Rotate Scale", "Rotation speed scale applied when rotating object")


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


                    ->ClassElement(AZ::Edit::ClassElements::Group, "Grab Distance Parameters")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(nullptr,
                        &Grab::m_initialGrabDistance,
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
                ->Event("Get Grabbing EntityId", &TestGemComponentRequests::GetGrabbingEntityId)
                ->Event("Get Active Camera EntityId", &TestGemComponentRequests::GetActiveCameraEntityId)
                ->Event("Get Grabbed Object EntityId", &TestGemComponentRequests::GetGrabbedObjectEntityId)
                ->Event("Set Grabbing Entity", &TestGemComponentRequests::SetGrabbingEntity)
                ->Event("Get Grab Collision Group", &TestGemComponentRequests::GetGrabCollisionGroup)
                ->Event("Set Grab Collision Group", &TestGemComponentRequests::SetGrabCollisionGroup)
                ->Event("Get isGrabbing", &TestGemComponentRequests::GetisGrabbing)
                ->Event("Get isThrowing", &TestGemComponentRequests::GetisThrowing)
                ->Event("Get isRotating", &TestGemComponentRequests::GetisRotating)
                ->Event("Get Grab Object Distance", &TestGemComponentRequests::GetGrabObjectDistance)
                ->Event("Set Grab Object Distance", &TestGemComponentRequests::SetGrabObjectDistance)
                ->Event("Get Minimum Grab Object Distance", &TestGemComponentRequests::GetMinGrabObjectDistance)
                ->Event("Set Minimum Grab Object Distance", &TestGemComponentRequests::SetMinGrabObjectDistance)
                ->Event("Get Maximum Grab Object Distance", &TestGemComponentRequests::GetMaxGrabObjectDistance)
                ->Event("Set Maximum Grab Object Distance", &TestGemComponentRequests::SetMaxGrabObjectDistance)
                ->Event("Get Initial Grab Object Distance", &TestGemComponentRequests::GetInitialGrabObjectDistance)
                ->Event("Set Initial Grab Object Distance", &TestGemComponentRequests::SetInitialGrabObjectDistance)
                ->Event("Get Grabbed Object Distance Speed", &TestGemComponentRequests::GetGrabObjectDistanceSpeed)
                ->Event("Set Grabbed Object Distance Speed", &TestGemComponentRequests::SetGrabObjectDistanceSpeed)
                ->Event("Get Grab Strength", &TestGemComponentRequests::GetGrabStrength)
                ->Event("Set Grab Strength", &TestGemComponentRequests::SetGrabStrength)
                ->Event("Get Grabbed Object Rotation Scale", &TestGemComponentRequests::GetRotateScale)
                ->Event("Set Grabbed Object Rotation Scale", &TestGemComponentRequests::SetRotateScale)
                ->Event("Get Grab Throw Strength", &TestGemComponentRequests::GetThrowStrength)
                ->Event("Set Grab Throw Strength", &TestGemComponentRequests::SetThrowStrength)
                ->Event("Get Grab Sphere Cast Radius", &TestGemComponentRequests::GetSphereCastRadius)
                ->Event("Set Grab Sphere Cast Radius", &TestGemComponentRequests::SetSphereCastRadius)
                ->Event("Get Grab Sphere Cast Distance", &TestGemComponentRequests::GetSphereCastDistance)
                ->Event("Set Grab Sphere Cast Distance", &TestGemComponentRequests::SetSphereCastDistance);

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
            m_grabbingEntityId = cameraId;
            m_grabbingEntityPtr = GetEntityPtr(cameraId);
    }
    
    AZ::Entity* Grab::GetActiveCameraEntityPtr() const
    {
        AZ::EntityId activeCameraId;
        Camera::CameraSystemRequestBus::BroadcastResult(activeCameraId,
            &Camera::CameraSystemRequestBus::Events::GetActiveCamera);

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
        CheckForObjects(deltaTime);
        m_grabPrevValue = m_grabKeyValue;
        //AZ_Printf("", "m_grabPrevValue = %.10f", m_grabPrevValue);
    }

    AZ::Entity* Grab::GetEntityPtr(AZ::EntityId pointer) const
    {
        auto ca = AZ::Interface<AZ::ComponentApplicationRequests>::Get();
        return ca->FindEntity(pointer);
    }

    void Grab::CheckForObjects(const float& deltaTime)
    {
        // Do not perform spherecast query if grab key is not pressed
        if (!m_grabKeyValue && !m_isGrabbing)
        {
            // Reset current grabbed distance to m_initialGrabDistance if grab key is not pressed
            m_grabDistance = m_initialGrabDistance;

            m_isThrowing = false;
            m_isRotating = false;
            m_hasRotated = false;

            // Set Grabbed Object to Dynamic Rigid Body if previously Kinematic
            if (m_isObjectKinematic)
            {
                Physics::RigidBodyRequestBus::Event(m_lastGrabbedObject,
                    &Physics::RigidBodyRequests::SetKinematic,
                    false);

                m_isObjectKinematic = false;
            }
            return;
        }

        // Get our local forward vector relative to the grabbing entity's transform
        m_forwardVector = AZ::Quaternion(m_grabbingEntityPtr->GetTransform()->GetWorldRotationQuaternion()).TransformVector(AZ::Vector3::CreateAxisY());

        // Get our grabbing entity's world transform
        m_grabbingEntityTransform = m_grabbingEntityPtr->GetTransform()->GetWorldTM();

        // Perform a spherecast query to check if colliding with object
        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();

        AzPhysics::ShapeCastRequest request = AzPhysics::ShapeCastRequestHelpers::CreateSphereCastRequest(m_sphereCastRadius,
            m_grabbingEntityTransform,
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
            m_isGrabbing = false;
            m_isThrowing = false;
            m_isRotating = false;
            m_hasRotated = false;

            // Set Object to Dynamic Rigid Body if it was previously Kinematic
            if (m_isObjectKinematic)
            {
                Physics::RigidBodyRequestBus::Event(m_lastGrabbedObject,
                    &Physics::RigidBodyRequests::SetKinematic,
                    false);

                m_isObjectKinematic = false;
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

        // Set Grabbed Object to Dynamic Rigid Body if previously Kinematic
        else if (m_isObjectKinematic)
        {
            Physics::RigidBodyRequestBus::Event(m_grabbedObjectEntityIds.at(0),
                &Physics::RigidBodyRequests::SetKinematic,
                false);

            m_isObjectKinematic = false;
        }

        m_rotatePrevValue = m_rotateKeyValue;

        // Call grab function only if object is not being thrown
        if (!m_throwKeyValue && !m_isThrowing)
        {
            GrabObject(m_grabbedObjectEntityIds.at(0), deltaTime);
        }

        // Call throw function if throw key is pressed
        else if (m_throwKeyValue)
        {
            //m_isGrabbing = false;
            ThrowObject(m_grabbedObjectEntityIds.at(0), deltaTime);
        }
    }

    void Grab::GrabObject(AZ::EntityId objectId, const float& deltaTime)
    {
        // Changes distance between Grabbing Entity and Grabbed object. Minimum and maximum grab distances determined by m_minGrabDistance and m_maxGrabDistance, respectively.
        m_grabDistance = AZ::GetClamp(m_grabDistance + (m_grabDistanceKeyValue * deltaTime * m_grabDistanceSpeed), m_minGrabDistance, m_maxGrabDistance);

        // Creates a reference point for the Grabbed Object translation in front of the Grabbing Entity. This can be thought of as the "hand".
        m_grabReference = m_grabbingEntityPtr->GetTransform()->GetWorldTM();
        m_grabReference.SetTranslation(m_grabbingEntityPtr->GetTransform()->GetWorldTM().GetTranslation() + m_forwardVector * m_grabDistance);

        m_grabbedObjectTranslation = GetEntityPtr(m_grabbedObjectEntityIds.at(0))->GetTransform()->GetWorldTM().GetTranslation();


        if (m_grabEnableToggle && m_grabPrevValue == 0.f && m_grabKeyValue != 0.f)
        {
            m_isGrabbing = !m_isGrabbing;
        }

        else if (!m_grabEnableToggle)
        {
            if (m_grabKeyValue != 0.f)
            {
                m_isGrabbing = true;
            }

            else
            {
                m_isGrabbing = false;
            }
        }

        if (m_kinematicDefaultEnable)
        {
            // Set Grabbed Object to Kinematic Rigid Body if previously Dynamic 
            if (!m_isObjectKinematic)
            {
                Physics::RigidBodyRequestBus::Event(objectId,
                    &Physics::RigidBodyRequests::SetKinematic,
                    true);

                m_isObjectKinematic = true;
            }

            if (!m_isRotating)
            {
                GetEntityPtr(objectId)->GetTransform()->SetWorldTranslation(m_grabReference.GetTranslation());
                if (!m_hasRotated)
                {
                    GetEntityPtr(objectId)->GetTransform()->SetWorldRotation(m_grabReference.GetRotation().GetEulerRadians());
                }
            }
            else
            {
                GetEntityPtr(objectId)->GetTransform()->SetWorldTranslation(m_grabReference.GetTranslation());
            }

        }

        else
        {
            // Subtract object's translation from our reference position, which gives you a vector pointing from the object to the reference. Then apply a linear velocity to move the object toward the reference. 
            Physics::RigidBodyRequestBus::Event(objectId,
                &Physics::RigidBodyRequests::SetLinearVelocity,
                (m_grabReference.GetTranslation() - m_grabbedObjectTranslation) * m_grabStrength);

            if (m_isRotating)
            {
                GetEntityPtr(objectId)->GetTransform()->SetWorldTranslation(m_grabReference.GetTranslation());
            }
        }
    }

    void Grab::ThrowObject(AZ::EntityId objectId, const float& deltaTime)
    {
        // Set Kinematic Rigid Body to Dynamic in order to prevent PhysX Rigid Body Warning in console when throwing object while rotating. This only happens temporarily, as the object will be set to Dynamic in order to throw.
        if (m_isObjectKinematic)
        {
            Physics::RigidBodyRequestBus::Event(m_grabbedObjectEntityIds.at(0),
                &Physics::RigidBodyRequests::SetKinematic,
                false);

            m_isObjectKinematic = false;
        }

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

        // Set Grabbed Object to Kinematic Rigid Body while rotating object
        if (!m_isObjectKinematic)
        {
            Physics::RigidBodyRequestBus::Event(objectId,
                &Physics::RigidBodyRequests::SetKinematic,
                true);

            m_isObjectKinematic = true;
        }

        else
        {
            AZ::Vector3 current_Rotation = GetEntityPtr(objectId)->GetTransform()->GetLocalRotation();

            m_delta_yaw = AZ::Vector3(0.f, 0.f, m_yawKeyValue);
            m_delta_pitch = AZ::Vector3(m_pitchKeyValue, 0.f, 0.f);

            m_delta_yaw *= m_rotateScale;
            m_delta_pitch *= m_rotateScale;

            AZ::Vector3 newRotation = AZ::Vector3::CreateZero();
            newRotation = current_Rotation + ((m_delta_yaw + m_delta_pitch) * deltaTime);

            GetEntityPtr(objectId)->GetTransform()->SetLocalRotation(newRotation);

            m_hasRotated = true;
        }
    }

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
        return m_lastGrabbedObject;
    }

    void Grab::SetGrabbingEntity(const AZ::EntityId new_grabbingEntityId)
    {
       m_grabbingEntityPtr = GetEntityPtr(new_grabbingEntityId);
    }

    bool Grab::GetisGrabbing() const
    {
        return m_isGrabbing;
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

    void Grab::SetGrabObjectDistance(const float& new_grabDistance)
    {
        m_grabDistance = AZ::GetClamp(new_grabDistance, m_minGrabDistance, m_maxGrabDistance);
    }

    float Grab::GetMinGrabObjectDistance() const
    {
        return m_minGrabDistance;
    }

    void Grab::SetMinGrabObjectDistance(const float& new_minGrabDistance)
    {
        m_minGrabDistance = new_minGrabDistance;
    }

    float Grab::GetMaxGrabObjectDistance() const
    {
        return m_maxGrabDistance;
    }

    void Grab::SetMaxGrabObjectDistance(const float& new_maxGrabDistance)
    {
        m_maxGrabDistance = new_maxGrabDistance;
    }

    float Grab::GetInitialGrabObjectDistance() const
    {
        return m_initialGrabDistance;
    }

    void Grab::SetInitialGrabObjectDistance(const float& new_initialGrabDistance)
    {
        m_initialGrabDistance = new_initialGrabDistance;
    }

    float Grab::GetGrabObjectDistanceSpeed() const
    {
        return m_grabDistanceSpeed;
    }

    void Grab::SetGrabObjectDistanceSpeed(const float& new_grabDistanceSpeed)
    {
        m_grabDistanceSpeed = new_grabDistanceSpeed;
    }

    float Grab::GetGrabStrength() const
    {
        return m_grabStrength;
    }

    void Grab::SetGrabStrength(const float& new_grabStrength)
    {
        m_grabStrength = new_grabStrength;
    }

    float Grab::GetRotateScale() const
    {
        return m_rotateScale;
    }

    void Grab::SetRotateScale(const float& new_rotateScale)
    {
        m_rotateScale = new_rotateScale;
    }

    float Grab::GetThrowStrength() const
    {
        return m_throwStrength;
    }

    void Grab::SetThrowStrength(const float& new_throwStrength)
    {
        m_throwStrength = new_throwStrength;
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

    AZStd::string Grab::GetGrabCollisionGroup() const
    {
        AZStd::string groupName;
        Physics::CollisionRequestBus::BroadcastResult(
            groupName, &Physics::CollisionRequests::GetCollisionGroupName, m_grabCollisionGroup);
        return groupName;
    }

    void Grab::SetGrabCollisionGroup(const AZStd::string& new_grabCollisionGroupName)
    {
        bool success = false;
        AzPhysics::CollisionGroup collisionGroup;
        Physics::CollisionRequestBus::BroadcastResult(success, &Physics::CollisionRequests::TryGetCollisionGroupByName, new_grabCollisionGroupName, collisionGroup);
        if (success)
        {
            m_grabCollisionGroup = collisionGroup;
            const AzPhysics::CollisionConfiguration& configuration = AZ::Interface<AzPhysics::SystemInterface>::Get()->GetConfiguration()->m_collisionConfig;
            m_grabCollisionGroupId = configuration.m_collisionGroups.FindGroupIdByName(new_grabCollisionGroupName);
        }
    }
}