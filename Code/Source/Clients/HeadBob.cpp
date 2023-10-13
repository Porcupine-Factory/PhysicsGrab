#include "HeadBob.h"

#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzFramework/Components/CameraBus.h>
#include <AzFramework/Physics/SystemBus.h>
#include <System/PhysXSystem.h>

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
                ->Field("HeadEntityId", &HeadBob::m_headEntityId)
                ->Version(1);

            if (AZ::EditContext* ec = sc->GetEditContext())
            {
                using namespace AZ::Edit;
                ec->Class<HeadBob>(
                    "HeadBob",
                    "[Component to add headbob to Player Controller]")
                    ->ClassElement(ClassElements::EditorData, "")
                    ->Attribute(
                        Attributes::AppearsInAddComponentMenu,
                        AZ_CRC_CE("Game"))
                    ->DataElement(nullptr,
                        &HeadBob::m_bobFreqency,
                        "Frequency", "Speed of the headbob")
                    ->DataElement(nullptr,
                        &HeadBob::m_bobHorzAmplitude,
                        "Horizontal Amplitude", "Distance of headbob along left/right axis")
                    ->DataElement(nullptr,
                        &HeadBob::m_bobVertAmplitude,
                        "Vertical Amplitude", "Distance of headbob along up/down axis")
                    ->DataElement(nullptr,
                        &HeadBob::m_headBobSmoothing,
                        "Smoothing", "Headbob smoothing.")
                    ->DataElement(0,
                        &HeadBob::m_headEntityId, "Head Entity Id", "Head entity to reference.");
            }
        }
    }
	void HeadBob::Activate()
	{
		AZ::TickBus::Handler::BusConnect();

        GetEntity()->GetTransform()->GetLocalRotation();
	}

	void HeadBob::Deactivate()
	{
		AZ::TickBus::Handler::BusDisconnect();
	}

    AZ::Entity* HeadBob::GetActiveCamera()
    {
        AZ::EntityId activeCameraId;
        Camera::CameraSystemRequestBus::BroadcastResult(activeCameraId,
            &Camera::CameraSystemRequestBus::Events::GetActiveCamera);

        auto ca = AZ::Interface<AZ::ComponentApplicationRequests>::Get();
        return ca->FindEntity(activeCameraId);
    }

	void HeadBob::OnTick(float deltaTime, AZ::ScriptTimePoint)
	{
        CalculateHeadbobOffset();

        AZ::Vector3 targetCameraPosition = AZ::Vector3::CreateZero();

        // Set walking time and target camera position to zero if player character is not moving.
        if (!m_isWalking)
        {
            m_walkingTime = 0.f;
            targetCameraPosition = AZ::Vector3::CreateZero();
        }
        // Calculate camera's new target position by adding offset. 
        else 
        {
            m_walkingTime += deltaTime;
            targetCameraPosition = (m_headEntityPointer->GetTransform()->GetLocalTM().GetTranslation() - AZ::Vector3(0.f, 0.f, m_headEntityPointer->GetTransform()->GetLocalTM().GetTranslation().GetZ())) + m_offset;
        }

        // Prints accumulated walking time.
        //AZ_Printf("", "m_walkingTime = %.10f", m_walkingTime);

        //Prints camera's target local position.
        //AZ_Printf("", "Target X = %.10f", targetCameraPosition.GetX());
        //AZ_Printf("", "Target Y = %.10f", targetCameraPosition.GetY());
        //AZ_Printf("", "Target Z = %.10f", targetCameraPosition.GetZ());

        // Get camera's current local position.
        AZ::Vector3 currentCameraPosition = GetActiveCamera()->GetTransform()->GetLocalTM().GetTranslation();

        // Prints camera's current local position.
        //AZ_Printf("", "Current X = %.10f", currentCameraPosition.GetX());
        //AZ_Printf("", "Current Y = %.10f", currentCameraPosition.GetY());
        //AZ_Printf("", "Current Z = %.10f", currentCameraPosition.GetZ());

        // Interpolate camera's position from currentCameraPosition to targetCameraPosition, using m_headBobSmoothing as the interpolant.
        GetActiveCamera()->GetTransform()->SetLocalTranslation(currentCameraPosition.Lerp(targetCameraPosition, m_headBobSmoothing));

        // Snap camera's current position to target position if it is close enough.
        if ((currentCameraPosition - targetCameraPosition).GetLength() <= 0.001)
        {
            GetActiveCamera()->GetTransform()->SetLocalTranslation(targetCameraPosition);
        }    
	}

    AZ::Entity* HeadBob::GetEntityPtr() const
    {
        auto ca = AZ::Interface<AZ::ComponentApplicationRequests>::Get();
        return ca->FindEntity(m_headEntityId);
    }
    
    void HeadBob::CalculateHeadbobOffset()
    {
        m_headEntityPointer = GetEntityPtr();

        AZ::Vector3 currentVelocity = AZ::Vector3::CreateZero();

        Physics::CharacterRequestBus::EventResult(currentVelocity, GetEntityId(),
            &Physics::CharacterRequestBus::Events::GetVelocity);

        m_isWalking = (((currentVelocity.GetX() != 0.f) || (currentVelocity.GetY() != 0.f)) && currentVelocity.GetZ() == 0.f);
        
        // Print entity's walking state
        //AZ_Printf("", "%s", m_isWalking ? "Walking" : "NOT Walking");
        //AZ_Printf("", "X Velocity = %.10f", currentVelocity.GetX());
        //AZ_Printf("", "Y Velocity = %.10f", currentVelocity.GetY());
        
        AZ::Vector3 headTranslation = m_headEntityPointer->GetTransform()->GetLocalTM().GetTranslation();

        // Print's "head" entity's local position.
        //AZ_Printf("", "X headTranslation = %.10f", headTranslation.GetX());
        //AZ_Printf("", "Y headTranslation = %.10f", headTranslation.GetY());
        //AZ_Printf("", "Z headTranslation = %.10f", headTranslation.GetZ());

        float horizontalOffset = 0.f;
        float verticalOffset = 0.f;

        if (m_walkingTime > 0.f)
        {
            // Calculate offsets using Lemniscate of Gerono curve
            horizontalOffset = cos(m_walkingTime * m_bobFreqency) * m_bobHorzAmplitude;
            verticalOffset = sin(m_walkingTime * m_bobFreqency * 2) * m_bobVertAmplitude;

            // Combine offsets relative to the head's position and calculate the camera's target position
            m_offset = AZ::Vector3(((headTranslation + AZ::Vector3::CreateAxisX()) * AZ::Vector3(horizontalOffset, 0.f, 0.f)) + ((headTranslation/headTranslation.GetZ()) * AZ::Vector3(0.f, 0.f, verticalOffset))); 
        }

        // Prints the active Entity's name
        //AZ_Printf("", "Current Entity = %s", GetEntity()->GetName().c_str());

        // Prints the active Entity's ID
        //AZ_Printf("", "Current Id = %s", GetEntityId().ToString().c_str());
        
        // Prints m_headEntityPointer name
        //AZ_Printf("", "Pointer Entity Name = %s", m_headEntityPointer->GetName().c_str());

        // Prints m_headEntityPointer Entitys ID 
        //AZ_Printf("", "Pointer EntityId = %s", m_headEntityId.ToString().c_str());

    }
}


