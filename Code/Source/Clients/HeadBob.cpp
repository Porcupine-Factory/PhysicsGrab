#include "HeadBob.h"

#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>

namespace TestGem
{
    void HeadBob::Reflect(AZ::ReflectContext* rc)
    {
        if (auto sc = azrtti_cast<AZ::SerializeContext*>(rc))
        {
            sc->Class<HeadBob, AZ::Component>()
                ->Field("Frequency", &HeadBob::m_bobFreqency)
                ->Field("Horizontal Amplitude", &HeadBob::m_bobHorzAmplitude)
                ->Field("Vertical Amplitude", &HeadBob::m_bobVertAmplitude)
                ->Field("Smoothing", &HeadBob::m_headBobSmoothing)
                ->Version(1);

            if (AZ::EditContext* ec = sc->GetEditContext())
            {
                using namespace AZ::Edit;
                ec->Class<HeadBob>("HeadBob", "[Component to add headbob to Player Controller]")
                    ->ClassElement(ClassElements::EditorData, "")
                    ->Attribute(Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(nullptr, &HeadBob::m_bobFreqency, "Frequency", "Speed of the headbob")
                    ->DataElement(
                        nullptr, &HeadBob::m_bobHorzAmplitude, "Horizontal Amplitude", "Distance of headbob along left/right axis")
                    ->DataElement(nullptr, &HeadBob::m_bobVertAmplitude, "Vertical Amplitude", "Distance of headbob along up/down axis")
                    ->DataElement(
                        nullptr,
                        &HeadBob::m_headBobSmoothing,
                        "Smoothing",
                        "Headbob smoothing. Lower values will decrease headbob intensity.");
            }
        }
    }
    void HeadBob::Activate()
    {
        Camera::CameraNotificationBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();
    }

    void HeadBob::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        Camera::CameraNotificationBus::Handler::BusDisconnect();
    }

    void HeadBob::OnCameraAdded(const AZ::EntityId& cameraId)
    {
        m_cameraEntity = GetEntityPtr(cameraId);
        m_originalCameraTranslation = m_cameraEntity->GetTransform()->GetLocalTM().GetTranslation();
    }

    void HeadBob::OnTick(float deltaTime, AZ::ScriptTimePoint)
    {
        // AZ_Printf("", "X Local Camera Translation = %.10f", m_cameraEntity->GetTransform()->GetLocalTranslation().GetX());
        // AZ_Printf("", "Y Local Camera Translation = %.10f", m_cameraEntity->GetTransform()->GetLocalTranslation().GetY());
        // AZ_Printf("", "Z Local Camera Translation = %.10f", m_cameraEntity->GetTransform()->GetLocalTranslation().GetZ());

        CalculateHeadbobOffset();

        AZ::Vector3 targetCameraPosition = AZ::Vector3::CreateZero();

        m_rightLocalVector =
            AZ::Quaternion(m_cameraEntity->GetTransform()->GetLocalRotationQuaternion()).TransformVector(AZ::Vector3::CreateAxisX());

        // Set walking time and target camera position to zero if player character is not moving.
        if (!m_isWalking)
        {
            m_walkingTime = 0.f;
            targetCameraPosition = AZ::Vector3::CreateAxisZ(m_originalCameraTranslation.GetZ());
        }
        // Calculate camera's new target position by adding offset.
        else
        {
            m_walkingTime += deltaTime;
            targetCameraPosition = m_originalCameraTranslation + m_offset;
        }

        // Get camera's current local position.
        AZ::Vector3 currentCameraPosition = m_cameraEntity->GetTransform()->GetLocalTranslation();

        // Interpolate camera's position from currentCameraPosition to targetCameraPosition, using m_headBobSmoothing as the interpolant.
        m_cameraEntity->GetTransform()->SetLocalTranslation(currentCameraPosition.Lerp(targetCameraPosition, m_headBobSmoothing));
        // Snap camera's current position to target position if it is close enough.
        if ((currentCameraPosition - targetCameraPosition).GetLength() <= 0.001)
        {
            m_cameraEntity->GetTransform()->SetLocalTranslation(targetCameraPosition);
        }
    }

    AZ::Entity* HeadBob::GetEntityPtr(AZ::EntityId pointer) const
    {
        auto ca = AZ::Interface<AZ::ComponentApplicationRequests>::Get();
        return ca->FindEntity(pointer);
    }

    void HeadBob::CalculateHeadbobOffset()
    {
        AZ::Vector3 currentVelocity = AZ::Vector3::CreateZero();

        Physics::CharacterRequestBus::EventResult(currentVelocity, GetEntityId(), &Physics::CharacterRequestBus::Events::GetVelocity);

        // Character is only in a walking state if current velocity in X or Y are NOT 0. Current Z velocity must also be 0 (not jumping).
        m_isWalking = (((currentVelocity.GetX() != 0.f) || (currentVelocity.GetY() != 0.f)) && currentVelocity.GetZ() == 0.f);

        float horizontalOffset = 0.f;
        float verticalOffset = 0.f;

        if (m_walkingTime > 0.f)
        {
            // Calculate offsets using Lemniscate of Gerono curve
            horizontalOffset = cos(m_walkingTime * m_bobFreqency) * m_bobHorzAmplitude;
            verticalOffset = sin(m_walkingTime * m_bobFreqency * 2) * m_bobVertAmplitude;

            // Combine offsets with the camera's local up and right vectors and calculate the camera's target position.
            // May need to test vertical offset factor with different gravity directions(e.g.walking on a ceiling) to confirm normal vector
            // direction with respect to the ground.
            m_offset = m_rightLocalVector * horizontalOffset + AZ::Vector3::CreateAxisZ(verticalOffset);
        }
    }
} // namespace TestGem