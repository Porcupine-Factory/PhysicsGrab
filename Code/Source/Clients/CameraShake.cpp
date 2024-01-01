#include "CameraShake.h"

#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzFramework/Components/CameraBus.h>

namespace TestGem
{
    using namespace StartingPointInput;

    void CameraShake::Reflect(AZ::ReflectContext* rc)
    {
        if (auto sc = azrtti_cast<AZ::SerializeContext*>(rc))
        {
            sc->Class<CameraShake, AZ::Component>()

                // Input Binding Key to perform shake
                ->Field("Shake Input Key", &CameraShake::m_strShake)

                // Main amplitude, frequency, and decay parameters of the shake noise functions.
                ->Field("Trauma", &CameraShake::m_traumaInitial)
                ->Field("Decay", &CameraShake::m_traumaDecay)
                ->Field("Speed", &CameraShake::m_freq)

                // Translational Shake group
                ->Field("X Translation Strength", &CameraShake::m_xTranslationAmplitude)
                ->Field("Y Translation Strength", &CameraShake::m_yTranslationAmplitude)
                ->Field("Z Translation Strength", &CameraShake::m_zTranslationAmplitude)

                // Rotational Shake group
                ->Field("X Rotation Strength", &CameraShake::m_xRotationAmplitude)
                ->Field("Y Rotation Strength", &CameraShake::m_yRotationAmplitude)
                ->Field("Z Rotation Strength", &CameraShake::m_zRotationAmplitude)
                ->Version(1);

            if (AZ::EditContext* ec = sc->GetEditContext())
            {
                using namespace AZ::Edit;
                ec->Class<CameraShake>(
                    "Camera Shake",
                    "[Component that shakes the active camera]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(
                        Attributes::AppearsInAddComponentMenu,
                        AZ_CRC_CE("Game"))

                    ->DataElement(nullptr,
                        &CameraShake::m_strShake,
                        "Shake Input Key", "Key to initiate or test Camera Shake.")

                    ->DataElement(nullptr,
                        &CameraShake::m_traumaInitial,
                        "Trauma", "Total shake strength")
                    ->DataElement(nullptr,
                        &CameraShake::m_traumaDecay,
                        "Decay", "Adjusts the length of the shake. Value of 0 produces infinite length.")
                    ->DataElement(nullptr,
                        &CameraShake::m_freq,
                        "Speed", "Frequency of the shake")

                    // Translational Shake group
                    ->ClassElement(AZ::Edit::ClassElements::Group, "Translational Shake Strength")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(nullptr,
                        &CameraShake::m_xTranslationAmplitude,
                        "X Translation Strength", "Amplitude of the translational camera shake along X")
                    ->DataElement(nullptr,
                        &CameraShake::m_yTranslationAmplitude,
                        "Y Translation Strength", "Amplitude of the translational camera shake along Y")
                    ->DataElement(nullptr,
                        &CameraShake::m_zTranslationAmplitude,
                        "Z Translation Strength", "Amplitude of the translational camera shake along Z")

                    // Rotational Shake group
                    ->ClassElement(AZ::Edit::ClassElements::Group, "Rotational Shake Strength")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(nullptr,
                        &CameraShake::m_xRotationAmplitude,
                        "X Rotation Strength", "Amplitude/Maximum Angle of the rotational camera shake about X")
                    ->DataElement(nullptr,
                        &CameraShake::m_yRotationAmplitude,
                        "Y Rotation Strength", "Amplitude/Maximum Angle of the rotational camera shake about Y")
                    ->DataElement(nullptr,
                        &CameraShake::m_zRotationAmplitude,
                        "Z Rotation Strength", "Amplitude/Maximum Angle of the rotational camera shake about Z");
            }
        }
    }

	void CameraShake::Activate()
	{
        m_shakeEventId = StartingPointInput::InputEventNotificationId(m_strShake.c_str());
        InputEventNotificationBus::MultiHandler::BusConnect(m_shakeEventId);

		AZ::TickBus::Handler::BusConnect();

        // When the entity is activated, set our current trauma level (m_trauma) to m_traumaInitial
        m_trauma = m_traumaInitial;
	}

	void CameraShake::Deactivate()
	{
		AZ::TickBus::Handler::BusDisconnect();
        InputEventNotificationBus::MultiHandler::BusDisconnect();
	}

    void CameraShake::OnCameraAdded(const AZ::EntityId& cameraId)
    {
        m_cameraEntity = GetEntityPtr(cameraId);
        m_originalCameraTranslation = m_cameraEntity->GetTransform()->GetLocalTM().GetTranslation();
    }

    AZ::Entity* CameraShake::GetEntityPtr(AZ::EntityId pointer) const
    {
        auto ca = AZ::Interface<AZ::ComponentApplicationRequests>::Get();
        return ca->FindEntity(pointer);
    }

    // Recieve the input event in OnPressed method
    void CameraShake::OnPressed(float value)
    {
        const InputEventNotificationId* inputId = InputEventNotificationBus::GetCurrentBusId();
        if (inputId == nullptr)
        {
            return;
        }

        if (*inputId == m_shakeEventId)
        {
            m_ShakeKey = value;
            m_initiateShake = true;
        }
    }

    // Recieve the input event in OnReleased method
    void CameraShake::OnReleased(float value)
    {
        const InputEventNotificationId* inputId = InputEventNotificationBus::GetCurrentBusId();
        if (inputId == nullptr)
        {
            return;
        }
        if (*inputId == m_shakeEventId)
        {
            m_ShakeKey = value;
        }
    }

    void CameraShake::OnTick(float deltaTime, AZ::ScriptTimePoint)
    {
        //AZ_Printf("", "X Local Camera Translation = %.10f", GetActiveCamera()->GetTransform()->GetLocalTranslation().GetX());
        //AZ_Printf("", "Y Local Camera Translation = %.10f", GetActiveCamera()->GetTransform()->GetLocalTranslation().GetY());
        //AZ_Printf("", "Z Local Camera Translation = %.10f", GetActiveCamera()->GetTransform()->GetLocalTranslation().GetZ());
        //AZ_Printf("", "Z m_originalCameraTranslation = %.10f", m_originalCameraTranslation.GetZ());
        Shake(deltaTime);
    }

    AZ::Entity* CameraShake::GetActiveCamera() const
    {
        AZ::EntityId activeCameraId;
        Camera::CameraSystemRequestBus::BroadcastResult(activeCameraId,
            &Camera::CameraSystemRequestBus::Events::GetActiveCamera);

        auto ca = AZ::Interface<AZ::ComponentApplicationRequests>::Get();
        return ca->FindEntity(activeCameraId);
    }

    void CameraShake::Shake(const float& deltaTime)
    {
        if (!m_initiateShake)
        {
            return;
        }

        m_currentTime += deltaTime;

        AZ::SimpleLcgRandom getSeed;
        m_Random = getSeed.GetRandom();

        // Getting our Camera's current local translation
        m_currentCameraTranslation = GetActiveCamera()->GetTransform()->GetLocalTM().GetTranslation();

        // Grabbing the Z local translation of the camera to add to the camera shake
        AZ::Vector3 zCameraTranslation = AZ::Vector3::CreateAxisZ(m_currentCameraTranslation.GetZ());

        // Subtracting the translation caused by shake to get a "clean" version of our current local camera translation
        AZ::Vector3 adjustedCameraTranslation = m_currentCameraTranslation - m_shakeTranslation;

        // Grabbing only the Z local translation of adjustedCameraTranslation to be used when setting our new translation with shake. 
        // This allows you to control the zheight of the camera while simultaniously allowing camera shake
        AZ::Vector3 zadjustedCameraTranslation = AZ::Vector3::CreateAxisZ(adjustedCameraTranslation.GetZ());

        // Getting our Camera's current rotation using Euler Radians to make the math a bit easier
        m_currentCameraRotation = GetActiveCamera()->GetTransform()->GetLocalTM().GetRotation().GetEulerRadians();

        // Grabbing the x local rotation of the camera to use when setting our camera position back after stopping shake
        AZ::Vector3 xCameraRotation = AZ::Vector3::CreateAxisX(m_currentCameraRotation.GetX());
        
        // Subtracting the rotation caused by shake to get a "clean" version of our current local camera rotation
        AZ::Vector3 adjustedCameraRotation = m_currentCameraRotation - m_shakeRotation;
        
        // Grabbing only the X local rotation of adjustedCameraRotation to be used when setting our new rotation with shake. 
        // This allows you to control the pitch of the camera while simultaniously allowing camera shake
        AZ::Vector3 xadjustedCameraRotation = AZ::Vector3::CreateAxisX(adjustedCameraRotation.GetX());

        if(m_trauma > 0)
        { 
            // Create a Vector3 with Perlin Noise values and amplitude multipliers for each axis. Used for translation. 
            m_shakeTranslation = AZ::Vector3(GenFastNoise(m_Random) * m_xTranslationAmplitude,
                GenFastNoise(m_Random + 2) * m_yTranslationAmplitude,
                GenFastNoise(m_Random + 3) * m_zTranslationAmplitude) * (m_trauma * m_trauma);

            // Create a Quaternion with Perlin Noise values and amplitude multipliers for each axis. Used for rotation.
            AZ::Quaternion shakeRotation = AZ::Quaternion::CreateFromEulerAnglesRadians(AZ::Vector3(GenFastNoise(m_Random + 4) * m_xRotationAmplitude,
                GenFastNoise(m_Random + 5) * m_yRotationAmplitude,
                GenFastNoise(m_Random + 6) * m_zRotationAmplitude) * (m_trauma * m_trauma));

            // Converting to Euler Radians to make the math a bit easier
            m_shakeRotation = shakeRotation.GetEulerRadians();

            // Set our active camera's translation to the current camera translation plus the shake values
            GetActiveCamera()->GetTransform()->SetLocalTranslation(zadjustedCameraTranslation + m_shakeTranslation);

            // Set our active camera's rotation to the current camera rotation plus the shake values
            GetActiveCamera()->GetTransform()->SetLocalRotation(xadjustedCameraRotation + m_shakeRotation);
            
            // Reducing the trauma amount over time. GetMax() ensures trauma is never less than 0
            m_trauma = AZ::GetMax(m_trauma - m_traumaDecay * deltaTime, 0.f);

            //AZ_Printf("", "m_trauma = %.10f", m_trauma)
        }
        
        else
        {
            m_initiateShake = false;
            m_trauma = m_traumaInitial;
            
            // Reset our shake/noise vectors back to 0
            m_shakeTranslation = AZ::Vector3::CreateZero();
            m_shakeRotation = AZ::Vector3::CreateZero();


            // NOTE: Camera does not reset back to original position. It still retains ztranslation, and xRotation values from shake. Need a better way 
            // of storing the original positions. 
            
            // Smoothly reset back to our camera's original translation with linear interpolation. Currently set to immediate reset.
            GetActiveCamera()->GetTransform()->SetLocalTranslation(m_currentCameraTranslation.Lerp(zCameraTranslation, 1.f));
            //GetActiveCamera()->GetTransform()->SetLocalTranslation(m_currentCameraTranslation.Lerp(m_originalCameraTranslation, 1.f));

            // Smoothly reset back to our camera's original rotation with linear interpolation. Currently set to immediate reset.
            //GetActiveCamera()->GetTransform()->SetLocalRotation(m_currentCameraRotation.Lerp(xCameraRotation, 1.f));
            GetActiveCamera()->GetTransform()->SetLocalRotation(m_currentCameraRotation.Lerp(xCameraRotation, 1.f));

        } 
    }

    float CameraShake::GenFastNoise(int Seed)
    {
        FastNoise noiseValues; 
        noiseValues.SetNoiseType(FastNoise::Perlin);
        noiseValues.SetSeed(Seed);
        noiseValues.SetFrequency(m_freq);

        m_perlinFastNoise = noiseValues.GetPerlin(m_currentTime, m_currentTime);

        //AZ_Printf("", "Perlin FAST Number = %.10f", m_perlinFastNoise);
        return m_perlinFastNoise;
    }
}