#pragma once
#include <AzCore/Component/Component.h>
#include <StartingPointInput/InputEventNotificationBus.h>

namespace TestGem
{
	// An example of a simple O3DE component
	class RaycastTest : public AZ::Component
	{
	public:
		AZ_COMPONENT(RaycastTest, "{81F030E1-6EBC-46E6-97D0-DA7B822239E6}");

		// AZ::Component overrides
		void Activate() override {}
		void Deactivate() override {}

		// Provide runtime reflection, if any
		static void Reflect(AZ::ReflectContext* reflection);
	};
}