#pragma once
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzFramework/Physics/CharacterBus.h>
#include <AzFramework/Components/CameraBus.h>
#include <FirstPersonController/FirstPersonControllerComponentBus.h>

namespace TestGem
{
	class HeadBob 
		: public AZ::Component
		, public AZ::TickBus::Handler
		, Camera::CameraNotificationBus::Handler
	{
	public:

		AZ_COMPONENT(HeadBob, "{AC40FA49-B4E9-414C-ACA2-A74290A83EBD}");

		// Provide runtime reflection
		static void Reflect(AZ::ReflectContext* rc);

		// AZ::Component overrides
		void Activate() override;
		void Deactivate() override;

		void OnTick(float deltaTime, AZ::ScriptTimePoint) override;
		
		AZ::Entity* GetEntityPtr(AZ::EntityId pointer) const;

	private:

		void OnCameraAdded(const AZ::EntityId& cameraId);
		void CalculateHeadbobOffset();

		AZ::Entity* m_cameraEntity = nullptr;

		AZ::Vector3 m_originalCameraTranslation = AZ::Vector3::CreateZero();
		AZ::Vector3 m_offset = AZ::Vector3::CreateZero();
		AZ::Vector3 m_rightLocalVector = AZ::Vector3::CreateZero();

		float m_walkingTime = 0.f;
		float m_bobFreqency = 7.f;
		float m_bobHorzAmplitude = 0.025f;
		float m_bobVertAmplitude = 0.06f;
		float m_headBobSmoothing = 0.2f;

		bool m_isWalking = false;
	};
}
