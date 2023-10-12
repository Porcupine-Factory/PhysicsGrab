#include "RaycastTest.h"

#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzFramework/Physics/CollisionBus.h>
#include <AzFramework/Physics/SystemBus.h>
#include <System/PhysXSystem.h>

namespace TestGem
{
    void RaycastTest::Reflect(AZ::ReflectContext* rc)
    {
        if (auto sc = azrtti_cast<AZ::SerializeContext*>(rc))
        {
            sc->Class<RaycastTest, AZ::Component>()
                ->Field("Grounded Collision Group", &RaycastTest::m_groundedCollisionGroupId)
                ->Field("Ground Check Radius", &RaycastTest::m_groundCheckRadius)
                ->Field("Sphere Cast Distance", &RaycastTest::m_sphereCastDistance)
                ->Field("Sphere Cast Distance", &RaycastTest::m_sphereCastDirection)
                ->Version(1);

            if (AZ::EditContext* ec = sc->GetEditContext())
            {
                using namespace AZ::Edit;
                ec->Class<RaycastTest>(
                    "Raycast Test",
                    "[Component to test raycast funcionality]")
                    ->ClassElement(ClassElements::EditorData, "")
                    ->Attribute(
                        Attributes::AppearsInAddComponentMenu,
                        AZ_CRC_CE("Game"))
                    ->DataElement(nullptr,
                        &RaycastTest::m_groundCheckRadius,
                        "Ground Check Radius", "Sphere Cast radius used for ground check")
                    ->DataElement(nullptr,
                        &RaycastTest::m_sphereCastDistance,
                        "Sphere Cast Distance", "Sphere Cast distance along m_sphereCastDirection")
                    ->DataElement(nullptr,
                        &RaycastTest::m_sphereCastDirection,
                        "Sphere Cast Direction", "Direction to cast Sphere")
                    ->DataElement(nullptr,
                        &RaycastTest::m_groundedCollisionGroupId,
                        "Grounded Collision Group", "The collision group which will be used for the ground detection.");
            }
        }
    }
	void RaycastTest::Activate()
	{
		AZ::TickBus::Handler::BusConnect();

        //Physics::CharacterNotificationBus::Handler::BusConnect(GetEntityId());

        Physics::CollisionRequestBus::BroadcastResult(
            m_groundedCollisionGroup, &Physics::CollisionRequests::GetCollisionGroupById, m_groundedCollisionGroupId);
	}

	void RaycastTest::Deactivate()
	{
		AZ::TickBus::Handler::BusDisconnect();
	}

	void RaycastTest::OnTick(float, AZ::ScriptTimePoint)
	{
		RaycastCheck();
	}

	void RaycastTest::RaycastCheck()
	{   
        // Get our entity's local translation   
        //AZ::Vector3 currentTranslation = GetEntity()->GetTransform()->GetLocalTranslation();

        /*
        // Perform a raycast query to check if entity is grounded
        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();

        AzPhysics::RayCastRequest request;
        request.m_start = currentTranslation;
        request.m_direction = AZ::Vector3(0.0f, 0.0f, -1.0f);
        request.m_distance = 0.3f;

        AzPhysics::SceneHandle sceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        AzPhysics::SceneQueryHits hits = sceneInterface->QueryScene(sceneHandle, &request);
        */
        
        // Get our entity's local transform and offset it along Z axis by m_groundCheckRadius distance
        AZ::Transform currentTransform = AZ::Transform::CreateIdentity();
        currentTransform.SetTranslation(GetEntity()->GetTransform()->GetLocalTM().GetTranslation() + AZ::Vector3::CreateAxisZ(m_groundCheckRadius));

        // Perform a spherecast query to check if entity is grounded
        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();

        AzPhysics::ShapeCastRequest request = AzPhysics::ShapeCastRequestHelpers::CreateSphereCastRequest(m_groundCheckRadius,
            currentTransform,
            AZ::Vector3(0.0f, 0.0f, m_sphereCastDirection),
            m_sphereCastDistance,
            AzPhysics::SceneQuery::QueryType::StaticAndDynamic,
            m_groundedCollisionGroup,
            nullptr);

        AzPhysics::SceneHandle sceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        AzPhysics::SceneQueryHits hits = sceneInterface->QueryScene(sceneHandle, &request);
       

        m_grounded = hits ? true : false;

        // Print entity's grounded state
        AZ_Printf("", "%s", m_grounded ? "Grounded" : "NOT Grounded");
	}
}


