#include "RaycastTest.h"
#include <AzCore/Serialization/EditContext.h>

using namespace TestGem;

void RaycastTest::Reflect(AZ::ReflectContext* reflection)
{
	//AZ_UNUSED(reflection);
	auto sc = azrtti_cast<AZ::SerializeContext*>(reflection);
	if (!sc) return;

	sc->Class<RaycastTest, Component>()
		->Version(1);

	AZ::EditContext* ec = sc->GetEditContext();
	if (!ec) return;

	using namespace AZ::Edit::Attributes;

		// reflection of this component for O3DE Editor

		ec->Class<RaycastTest>("Raycast Test", "[Test new component]")
		->ClassElement(AZ::Edit::ClassElements::EditorData, "")
		->Attribute(AppearsInAddComponentMenu, AZ_CRC("Game"))
		->Attribute(Category, "TestGem");
}


