#pragma once
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Math/Random.h>
#include <External/FastNoise/FastNoise.h>
#include <StartingPointInput/InputEventNotificationBus.h>

namespace TestGem
{

	class CameraShake 
		: public AZ::Component
		, public AZ::TickBus::Handler
		, public StartingPointInput::InputEventNotificationBus::MultiHandler
	{
	public:
		AZ_COMPONENT(CameraShake, "{CE4A6EDA-BC0D-4524-AAD5-994370E67F67}");

		// Provide runtime reflection
		static void Reflect(AZ::ReflectContext* rc);

		// AZ::Component overrides
		void Activate() override;
		void Deactivate() override;

		// AZ::InputEventNotificationBus interface. Overrides OnPressed and OnReleased virtual methods. 
		void OnPressed(float value) override;
		void OnReleased(float value) override;

		void OnTick(float deltaTime, AZ::ScriptTimePoint) override;

		AZ::Entity* GetEntityPtr(AZ::EntityId pointer) const;

	private:
		AZ::Entity* m_cameraEntity = nullptr;
		AZ::Entity* m_activeCameraEntity = nullptr;
		AZ::Entity* GetActiveCamera() const;

		StartingPointInput::InputEventNotificationId m_shakeEventId;
		AZStd::string m_strShake = "Camera Shake";

		void OnCameraAdded(const AZ::EntityId& cameraId);
		void Shake(const float& deltaTime);
		float GenFastNoise(int genSeed);

		bool m_initiateShake = false;

		AZ::Vector3 m_originalCameraTranslation = AZ::Vector3::CreateZero();
		AZ::Vector3 m_currentCameraTranslation = AZ::Vector3::CreateZero();
		AZ::Vector3 m_currentCameraRotation = AZ::Vector3::CreateZero();
		AZ::Vector3 m_shakeTranslation = AZ::Vector3::CreateZero();
		AZ::Vector3 m_shakeRotation = AZ::Vector3::CreateZero();

		int m_Random = 0;

		float m_traumaInitial = 1.3f;
		float m_trauma = 0.f;
		float m_traumaDecay = 2.f;
		float m_freq = 19.f;
		float m_xTranslationAmplitude = 0.02f;
		float m_yTranslationAmplitude = 0.f;
		float m_zTranslationAmplitude = 0.02f;
		float m_xRotationAmplitude = 0.04f;
		float m_yRotationAmplitude = 0.01f;
		float m_zRotationAmplitude = 0.05f;
		float m_perlinFastNoise;
		float m_currentTime = 0.f;
		float m_perlinNoise = 0.f;
		float m_ShakeKey = 0.f;
	};
}