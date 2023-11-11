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
                ->Field("Rotate Input Key", &Grab::m_strRotate)

                ->Field("Sphere Cast Radius", &Grab::m_sphereCastRadius)
                ->Field("Sphere Cast Distance", &Grab::m_sphereCastDistance)
                ->Field("Default Grab Distance", &Grab::m_grabInitialDistance)
                ->Field("Min Grab Distance", &Grab::m_minGrabDistance)
                ->Field("Max Grab Distance", &Grab::m_maxGrabDistance)
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

                    // Input Binding Keys
                    ->DataElement(nullptr,
                        &Grab::m_strGrab,
                        "Grab Input Key", "Grab interaction input binding")
                    ->DataElement(nullptr,
                        &Grab::m_strGrabDistance,
                        "Grab Distance Input Key", "Grab distance input binding")
                    ->DataElement(nullptr,
                        &Grab::m_strThrow,
                        "Throw Input Key", "Throw object input binding")
                    ->DataElement(nullptr,
                        &Grab::m_strRotate,
                        "Rotate Input Key", "Rotate object input binding")

                    ->DataElement(nullptr,
                        &Grab::m_throwStrength,
                        "Throw Strength", "Linear Impulse scale applied when throwing grabbed object")
                    ->DataElement(nullptr,
                        &Grab::m_grabStrength,
                        "Grab Strength", "Linear velocity scale applied when holding grabbed object")
                    ->DataElement(nullptr,
                        &Grab::m_rotateScale,
                        "Rotate Scale", "Angular Velocity scale applied when rotating object")

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
                        "Max Grab Distance", "Maximum allowable grab distance. Grabbed object cannot get further than this distance.");
            }
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

        Camera::CameraNotificationBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();

        Physics::CollisionRequestBus::BroadcastResult(
            m_grabCollisionGroup, &Physics::CollisionRequests::GetCollisionGroupById, m_grabCollisionGroupId);
    }

    void Grab::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        InputEventNotificationBus::MultiHandler::BusDisconnect();

        Camera::CameraNotificationBus::Handler::BusDisconnect();
    }

    void Grab::OnCameraAdded(const AZ::EntityId& cameraId)
    {
        m_cameraEntity = GetEntityPtr(cameraId);
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
            m_grabKey = value;
            //AZ_Printf("Player", "Grab value %f", value);
        }

        if (*inputId == m_grabDistanceEventId)
        {
            m_grabDistanceKey = value;
            //AZ_Printf("Player", "Grab Distance value %f", value);
        }

        if (*inputId == m_throwEventId)
        {
            m_throwKey = value;
            //AZ_Printf("Player", "Throw value value %f", value);
        }

        if (*inputId == m_rotateEventId)
        {
            m_rotateKey = value;
            //AZ_Printf("Player", "Throw value value %f", value);
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
            m_grabKey = value;
            //AZ_Printf("Player", "Grab released value %f", value);
        }
        if (*inputId == m_grabDistanceEventId)
        {
            m_grabDistanceKey = value;
            //AZ_Printf("Player", "Grab Distance released value %f", value);
        }
        if (*inputId == m_throwEventId)
        {
            m_throwKey = value;
            //AZ_Printf("Player", "Throw released value %f", value);
        }
        if (*inputId == m_rotateEventId)
        {
            m_rotateKey = value;
            //AZ_Printf("Player", "Throw released value %f", value);
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

        if (*inputId == RotateYawEventId)
        {
            m_grabKey = value;
        }
    }

    void Grab::OnTick(float deltaTime, AZ::ScriptTimePoint)
    {
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
        if (!m_grabKey)
        {
            // Reset current grabbed distance to m_grabInitialDistance if grab key is not pressed
            m_grabDistance = m_grabInitialDistance;
            isThrowing = false;
            return;
        }

        // Get our local forward vector relative to the camera transform
        m_forwardVector = AZ::Quaternion(m_cameraEntity->GetTransform()->GetWorldRotationQuaternion()).TransformVector(AZ::Vector3::CreateAxisY());
        
        // Get our Camera's world transform
        m_cameraTransform = m_cameraEntity->GetTransform()->GetWorldTM();

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

        // Print entity's collision state
        //AZ_Printf("", "%s", hits ? "Colliding with Object" : "");

        m_grabEntityIds.clear();

        if (!hits)
        {
            return;
        }

        // Create a vector of every object hit from spherecast 
        //for (AzPhysics::SceneQueryHit hit : hits.m_hits)
            //m_grabEntityIds.push_back(hit.m_entityId);

        // Takes the first object hit in m_hits vector and assigns it to m_grabEntityIds vector
        //AzPhysics::SceneQueryHit firstHitObject = hits.m_hits.at(0);
        //m_grabEntityIds.push_back(firstHitObject.m_entityId);
        m_grabEntityIds.push_back(hits.m_hits.at(0).m_entityId);

        //AZ_Printf("", "m_grabEntityIds.size() = %d", m_grabEntityIds.size());
        //AZ_Printf("", "m_grabEntityId.at(0) = %s", m_grabEntityIds.at(0).ToString().c_str());
        //AZ_Printf("", "m_grabEntityId name = %s", GetEntityPtr(m_grabEntityIds.at(0))->GetName().c_str());

        if (!m_rotateKey)
        {
            m_grabDistance = AZ::GetClamp(m_grabDistance + m_grabDistanceKey * 0.002f, m_minGrabDistance, m_maxGrabDistance);

            // Print grabbed object's distance from the camera
            //AZ_Printf("", "m_grabDistance = %.10f", m_grabDistance);
        }

        else 
        {
            RotateObject(m_grabEntityIds.at(0));
        }

        m_grabReference = m_cameraEntity->GetTransform()->GetWorldTM();
        m_grabReference.SetTranslation(m_cameraEntity->GetTransform()->GetWorldTM().GetTranslation() + m_forwardVector * m_grabDistance);

        m_grabbedObject = GetEntityPtr(m_grabEntityIds.at(0))->GetTransform()->GetWorldTM().GetTranslation();

        if (m_throwKey)
        {
            ThrowObject(m_grabEntityIds.at(0), deltaTime);
            isThrowing = true;
        }

        else if (!m_throwKey && !isThrowing)
        {
            HoldObject(m_grabEntityIds.at(0), deltaTime);
        }              
    }

    void Grab::HoldObject(AZ::EntityId objectId, const float& deltaTime)
    {
        // Subtract object's translation from our reference position, which gives you a vector pointing from the object to the reference. Then apply a linear velocity to move the object toward the reference. 
        Physics::RigidBodyRequestBus::Event(objectId, 
            &Physics::RigidBodyRequests::SetLinearVelocity, 
            (m_grabReference.GetTranslation() - m_grabbedObject) * m_grabStrength * deltaTime);
    }

    void Grab::ThrowObject(AZ::EntityId objectId, const float& deltaTime)
    {
        // Apply a Linear Impulse to our held object
        Physics::RigidBodyRequestBus::Event(objectId,
            &Physics::RigidBodyRequestBus::Events::ApplyLinearImpulse,
            m_forwardVector * m_throwStrength * deltaTime);
    }
    
    void Grab::RotateObject(AZ::EntityId objectId)
    {
        // Rotate grabbed object about X axis using Angular Velocity
        Physics::RigidBodyRequestBus::Event(objectId,
            &Physics::RigidBodyRequests::SetAngularVelocity,
            AZ::Vector3::CreateAxisX(m_grabDistanceKey) * m_rotateScale);

        //AZ_Printf("", "Object Rotation X = %.10f", GetEntityPtr(objectId)->GetTransform()->GetWorldRotation().GetX());
        //AZ_Printf("", "Object Rotation Y = %.10f", GetEntityPtr(objectId)->GetTransform()->GetWorldRotation().GetY());
        //AZ_Printf("", "Object Rotation Z = %.10f", GetEntityPtr(objectId)->GetTransform()->GetWorldRotation().GetZ());
    }
}