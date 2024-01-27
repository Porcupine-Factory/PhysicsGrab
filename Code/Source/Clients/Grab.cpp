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
                //->Field("Maintain Grab", &Grab::m_grabMaintained)
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
                ->Field("Grabbed Object Collision Group", &Grab::m_grabbedCollisionGroupId)
                ->Field("Grabbed Object Temporary Collision Layer", &Grab::m_tempGrabbedCollisionLayer)
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
                    //->DataElement(nullptr,
                    //    &Grab::m_grabMaintained,
                    //    "Maintain Grab", "Grabbed Object remains held even if sphere cast no longer intersects it. This prevents the Grabbed Object from flying off when quickly changing directions.")
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
                        &Grab::m_grabbedCollisionGroupId,
                        "Sphere Cast Collision Group", "The collision group which will be used for detecting grabbable objects.")
                    ->DataElement(nullptr,
                        &Grab::m_tempGrabbedCollisionLayer,
                        "Grabbed Object Temporary Collision Layer", "The temporary collision layer assigned to the grabbed object while it is being grabbed/held.")
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

            m_isGrabbing = false;
            m_isThrowing = false;
            m_isRotating = false;
            m_hasRotated = false;

            // Set Grabbed Object to Dynamic Rigid Body if previously Kinematic
            if (m_isObjectKinematic)
            {
                Physics::RigidBodyRequestBus::Event(m_lastGrabbedObjectEntityId,
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
            m_grabbedCollisionGroup,
            nullptr);

        AzPhysics::SceneHandle sceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        AzPhysics::SceneQueryHits hits = sceneInterface->QueryScene(sceneHandle, &request);

        if (!hits/* && !m_grabMaintained*/)
        {
            m_isGrabbing = false;
            m_isThrowing = false;
            m_isRotating = false;
            m_hasRotated = false;

            // Set Object to Dynamic Rigid Body if it was previously Kinematic
            if (m_isObjectKinematic)
            {
                Physics::RigidBodyRequestBus::Event(m_lastGrabbedObjectEntityId,
                    &Physics::RigidBodyRequests::SetKinematic,
                    false);

                m_isObjectKinematic = false;
            }
            // Set Object Current Layer variable back to Prev Layer if sphere cast does not detect a hit
            SetCurrentGrabbedCollisionLayer(m_prevGrabbedCollisionLayer);
            return;
        }

        // Prevents Grabbing new object if currently grabbing.
        if (!m_isGrabbing)
        {
            // Takes the first object hit in m_hits vector and assigns it to m_grabbedObjectEntityId
            m_grabbedObjectEntityId = hits.m_hits.at(0).m_entityId;
            m_lastGrabbedObjectEntityId = m_grabbedObjectEntityId;
        }

        if ((m_rotateKeyValue != 0 || m_isRotating) && !m_isThrowing)
        {
            RotateObject(m_lastGrabbedObjectEntityId, deltaTime);
        }

        // Set Grabbed Object to Dynamic Rigid Body if previously Kinematic
        else if (m_isObjectKinematic)
        {
            Physics::RigidBodyRequestBus::Event(m_lastGrabbedObjectEntityId,
                &Physics::RigidBodyRequests::SetKinematic,
                false);

            m_isObjectKinematic = false;
        }

        m_rotatePrevValue = m_rotateKeyValue;

        // Call grab function only if object is not being thrown
        if (!m_throwKeyValue && !m_isThrowing)
        {
            GrabObject(m_lastGrabbedObjectEntityId, deltaTime);
        }

        // Call throw function if throw key is pressed
        else if (m_throwKeyValue)
        {
            m_isGrabbing = false;
            ThrowObject(m_lastGrabbedObjectEntityId);
        }
    }

    void Grab::GrabObject(AZ::EntityId objectId, const float& deltaTime)
    {
        // Changes distance between Grabbing Entity and Grabbed object. Minimum and maximum grab distances determined by m_minGrabDistance and m_maxGrabDistance, respectively.
        m_grabDistance = AZ::GetClamp(m_grabDistance + (m_grabDistanceKeyValue * deltaTime * m_grabDistanceSpeed), m_minGrabDistance, m_maxGrabDistance);

        // Creates a reference point for the Grabbed Object translation in front of the Grabbing Entity. This can be thought of as the "hand".
        m_grabReference = m_grabbingEntityPtr->GetTransform()->GetWorldTM();
        m_grabReference.SetTranslation(m_grabbingEntityPtr->GetTransform()->GetWorldTM().GetTranslation() + m_forwardVector * m_grabDistance);

        m_grabbedObjectTranslation = GetEntityPtr(m_grabbedObjectEntityId)->GetTransform()->GetWorldTM().GetTranslation();

        if (m_grabEnableToggle && m_grabPrevValue == 0.f && m_grabKeyValue != 0.f)
        {
            if (!m_isGrabbing)
            {
                // Set Object Current Layer variable to Temp Layer if Grabbing
                m_prevGrabbedCollisionLayer = GetCurrentGrabbedCollisionLayer();
                SetCurrentGrabbedCollisionLayer(m_tempGrabbedCollisionLayer);
            }
            else
            {
                // Set Object Current Layer variable back to Prev Layer if Grab Key is not pressed
                SetCurrentGrabbedCollisionLayer(m_prevGrabbedCollisionLayer);
            }
            m_isGrabbing = !m_isGrabbing;
        }
        else if (m_grabPrevValue != m_grabKeyValue && !m_grabEnableToggle)
        {
            if (m_grabKeyValue != 0.f)
            {
                // Set Object Current Layer variable to Temp Layer if Grabbing
                m_prevGrabbedCollisionLayer = GetCurrentGrabbedCollisionLayer();
                SetCurrentGrabbedCollisionLayer(m_tempGrabbedCollisionLayer);
                m_isGrabbing = true;
            }
            else
            {
                // Set Object Current Layer variable back to Prev Layer if Grab Key is not pressed
                SetCurrentGrabbedCollisionLayer(m_prevGrabbedCollisionLayer);
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

    void Grab::ThrowObject(AZ::EntityId objectId)
    {
        // Set Kinematic Rigid Body to Dynamic in order to prevent PhysX Rigid Body Warning in console when throwing object while rotating. This only happens temporarily, as the object will be set to Dynamic in order to throw.
        if (m_isObjectKinematic)
        {
            Physics::RigidBodyRequestBus::Event(m_grabbedObjectEntityId,
                &Physics::RigidBodyRequests::SetKinematic,
                false);

            m_isObjectKinematic = false;
        }

        m_isThrowing = true;

        // Apply a Linear Impulse to our held object.
        Physics::RigidBodyRequestBus::Event(objectId,
            &Physics::RigidBodyRequestBus::Events::ApplyLinearImpulse,
            m_forwardVector * m_throwStrength);
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
        return m_grabbedObjectEntityId;
    }

    AZ::EntityId Grab::GetLastGrabbedObjectEntityId() const
    {
        return m_lastGrabbedObjectEntityId;
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
        Physics::CollisionRequestBus::BroadcastResult(success, &Physics::CollisionRequests::TryGetCollisionGroupByName, new_grabbedCollisionGroupName, collisionGroup);
        if (success)
        {
            m_grabbedCollisionGroup = collisionGroup;
            const AzPhysics::CollisionConfiguration& configuration = AZ::Interface<AzPhysics::SystemInterface>::Get()->GetConfiguration()->m_collisionConfig;
            m_grabbedCollisionGroupId = configuration.m_collisionGroups.FindGroupIdByName(new_grabbedCollisionGroupName);
        }
    }

    AZStd::string Grab::GetCurrentGrabbedCollisionLayerName() const
    {
        AZStd::string currentGrabbedCollisionLayerName;
        Physics::CollisionFilteringRequestBus::EventResult(currentGrabbedCollisionLayerName, m_lastGrabbedObjectEntityId, &Physics::CollisionFilteringRequestBus::Events::GetCollisionLayerName);
        return currentGrabbedCollisionLayerName;
    }

    void Grab::SetCurrentGrabbedCollisionLayerByName(const AZStd::string& new_currentGrabbedCollisionLayerName)
    {
        bool success = false;
        AzPhysics::CollisionLayer grabbedCollisionLayer;
        Physics::CollisionRequestBus::BroadcastResult(success, &Physics::CollisionRequests::TryGetCollisionLayerByName, new_currentGrabbedCollisionLayerName, grabbedCollisionLayer);
        if (success)
        {
            m_currentGrabbedCollisionLayerName = new_currentGrabbedCollisionLayerName;
            m_currentGrabbedCollisionLayer = grabbedCollisionLayer;
            Physics::CollisionFilteringRequestBus::Event(m_lastGrabbedObjectEntityId, &Physics::CollisionFilteringRequestBus::Events::SetCollisionLayer, m_currentGrabbedCollisionLayerName, AZ::Crc32());
        }
    }

    AzPhysics::CollisionLayer Grab::GetCurrentGrabbedCollisionLayer() const
    {
        AZStd::string grabbedCollisionLayerName;
        Physics::CollisionFilteringRequestBus::EventResult(grabbedCollisionLayerName, m_lastGrabbedObjectEntityId, &Physics::CollisionFilteringRequestBus::Events::GetCollisionLayerName);
        AzPhysics::CollisionLayer grabbedCollisionLayer;
        Physics::CollisionRequestBus::BroadcastResult(grabbedCollisionLayer, &Physics::CollisionRequests::GetCollisionLayerByName, grabbedCollisionLayerName);
        return grabbedCollisionLayer;
    }

    void Grab::SetCurrentGrabbedCollisionLayer(const AzPhysics::CollisionLayer& new_currentGrabbedCollisionLayer)
    {
        m_currentGrabbedCollisionLayer = new_currentGrabbedCollisionLayer;
        const AzPhysics::CollisionConfiguration& configuration = AZ::Interface<AzPhysics::SystemInterface>::Get()->GetConfiguration()->m_collisionConfig;
        m_currentGrabbedCollisionLayerName = configuration.m_collisionLayers.GetName(m_currentGrabbedCollisionLayer);
        Physics::CollisionFilteringRequestBus::Event(m_lastGrabbedObjectEntityId, &Physics::CollisionFilteringRequestBus::Events::SetCollisionLayer, m_currentGrabbedCollisionLayerName, AZ::Crc32());
    }

    AZStd::string Grab::GetPrevGrabbedCollisionLayerName() const
    {
        const AzPhysics::CollisionConfiguration& configuration = AZ::Interface<AzPhysics::SystemInterface>::Get()->GetConfiguration()->m_collisionConfig;
        return configuration.m_collisionLayers.GetName(m_prevGrabbedCollisionLayer);
    }

    void Grab::SetPrevGrabbedCollisionLayerByName(const AZStd::string& new_prevGrabbedCollisionLayerName)
    {
        bool success = false;
        AzPhysics::CollisionLayer prevGrabbedCollisionLayer;
        Physics::CollisionRequestBus::BroadcastResult(success, &Physics::CollisionRequests::TryGetCollisionLayerByName, new_prevGrabbedCollisionLayerName, prevGrabbedCollisionLayer);
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
        Physics::CollisionRequestBus::BroadcastResult(success, &Physics::CollisionRequests::TryGetCollisionLayerByName, new_tempGrabbedCollisionLayerName, tempGrabbedCollisionLayer);
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
        const AzPhysics::CollisionConfiguration& configuration = AZ::Interface<AzPhysics::SystemInterface>::Get()->GetConfiguration()->m_collisionConfig;
        m_tempGrabbedCollisionLayerName = configuration.m_collisionLayers.GetName(m_tempGrabbedCollisionLayer);
    }
}
