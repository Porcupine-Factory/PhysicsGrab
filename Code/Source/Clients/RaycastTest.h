#pragma once
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Math/Vector3.h>
#include <AzFramework/Physics/Common/PhysicsSceneQueries.h>
//#include <PhysXCharacters/API/CharacterController.h>

namespace TestGem
{
	// An example of a simple O3DE component
	class RaycastTest 
		: public AZ::Component
		, public AZ::TickBus::Handler
	{
	public:
		AZ_COMPONENT(RaycastTest, "{81F030E1-6EBC-46E6-97D0-DA7B822239E6}");

		// Provide runtime reflection, if any
		static void Reflect(AZ::ReflectContext* rc);

		// AZ::Component overrides
		void Activate() override;
		void Deactivate() override;

		void OnTick(float deltaTime, AZ::ScriptTimePoint) override;

	private:
		void RaycastCheck();

		bool m_grounded = true;
	};
}