#include "RaycastTest.h"

#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzFramework/Physics/SystemBus.h>
#include <System/PhysXSystem.h>

namespace TestGem
{
    void RaycastTest::Reflect(AZ::ReflectContext* rc)
    {
        if (auto sc = azrtti_cast<AZ::SerializeContext*>(rc))
        {
            sc->Class<RaycastTest, AZ::Component>()
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
                        AZ_CRC_CE("Game"));
            }
        }
    }
	void RaycastTest::Activate()
	{
		AZ::TickBus::Handler::BusConnect();
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
        AZ::Vector3 currentTranslation = GetEntity()->GetTransform()->GetLocalTranslation();

        // Perform a raycast query to check if entity is grounded
        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();

        AzPhysics::RayCastRequest request;
        request.m_start = currentTranslation;
        request.m_direction = AZ::Vector3(0.0f, 0.0f, -1.0f);
        request.m_distance = 0.3f;

        AzPhysics::SceneHandle sceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        AzPhysics::SceneQueryHits hits = sceneInterface->QueryScene(sceneHandle, &request);

        m_grounded = hits ? true : false;

        // Print entity's grounded state
        AZ_Printf("", "%s", m_grounded ? "Grounded" : "NOT Grounded");
	}
}


