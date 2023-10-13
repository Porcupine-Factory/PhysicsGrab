#pragma once
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Math/Vector3.h>
#include <AzFramework/Physics/CharacterBus.h>


namespace TestGem
{
	class HeadBob 
		: public AZ::Component
		, public AZ::TickBus::Handler
	{
	public:
		AZ_COMPONENT(HeadBob, "{AC40FA49-B4E9-414C-ACA2-A74290A83EBD}");

		// Provide runtime reflection, if any
		static void Reflect(AZ::ReflectContext* rc);

		// AZ::Component overrides
		void Activate() override;
		void Deactivate() override;

		void OnTick(float deltaTime, AZ::ScriptTimePoint) override;
		
		AZ::EntityId m_headEntityId;
		AZ::Entity* GetEntityPtr() const;

	private:
		AZ::Entity* m_activeCameraEntity = nullptr;
		AZ::Entity* GetActiveCamera();
		AZ::Entity* m_headEntityPointer = nullptr;

		void CalculateHeadbobOffset();

		AZ::Transform m_headTransform = AZ::Transform::CreateIdentity();
		AZ::Transform m_cameraTransform = AZ::Transform::CreateIdentity();

		AZ::Vector3 m_offset = AZ::Vector3::CreateZero();

		float m_walkingTime = 0.f;
		float m_bobFreqency = 5.f;
		float m_bobHorzAmplitude = 0.1f;
		float m_bobVertAmplitude = 0.1f;
		float m_headBobSmoothing = 0.1f;

		bool m_isWalking = false;
	};
}