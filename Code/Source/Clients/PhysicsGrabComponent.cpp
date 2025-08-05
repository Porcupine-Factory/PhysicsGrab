#include "PhysicsGrabComponent.h"

#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzFramework/Physics/CollisionBus.h>
#include <AzFramework/Physics/NameConstants.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <AzFramework/Physics/SystemBus.h>
#include <System/PhysXSystem.h>

namespace PhysicsGrab
{
    using namespace StartingPointInput;

    void PhysicsGrabComponent::Reflect(AZ::ReflectContext* rc)
    {
        if (auto sc = azrtti_cast<AZ::SerializeContext*>(rc))
        {
            sc->Class<PhysicsGrabComponent, AZ::Component>()

                // Grab Input Binding Keys
                ->Field("Grab Input Key", &PhysicsGrabComponent::m_strGrab)
                ->Field("Grab Distance Input Key", &PhysicsGrabComponent::m_strGrabDistance)
                ->Field("Throw Input Key", &PhysicsGrabComponent::m_strThrow)
                ->Field("Rotate Enable Input Key", &PhysicsGrabComponent::m_strRotate)
                ->Field("Rotate Pitch Key", &PhysicsGrabComponent::m_strRotatePitch)
                ->Field("Rotate Yaw Key", &PhysicsGrabComponent::m_strRotateYaw)
                ->Field("Rotate Roll Key", &PhysicsGrabComponent::m_strRotateRoll)

                ->Field("Sphere Cast Radius", &PhysicsGrabComponent::m_sphereCastRadius)
                ->Attribute(AZ::Edit::Attributes::Suffix, " " + Physics::NameConstants::GetLengthUnit())
                ->Field("Sphere Cast Distance", &PhysicsGrabComponent::m_sphereCastDistance)
                ->Attribute(AZ::Edit::Attributes::Suffix, " " + Physics::NameConstants::GetLengthUnit())
                ->Field("Grabbed Object Collision Group", &PhysicsGrabComponent::m_grabbedCollisionGroupId)
                ->Field("Grabbed Object Temporary Collision Layer", &PhysicsGrabComponent::m_tempGrabbedCollisionLayer)

                ->Field("GrabbingEntityId", &PhysicsGrabComponent::m_grabbingEntityId)
                #ifdef FIRST_PERSON_CONTROLLER
                ->Field("Use First Person Controller For Grab", &PhysicsGrabComponent::m_useFPControllerForGrab)
                #endif
                ->Field("Mesh Smoothing", &PhysicsGrabComponent::m_meshSmoothing)
                ->Field("Mesh Tag", &PhysicsGrabComponent::m_meshTagName)
                ->Field("Grab Response", &PhysicsGrabComponent::m_grabResponse)
                ->Attribute(
                    AZ::Edit::Attributes::Suffix,
                    AZStd::string::format(
                        " s%s%s",
                        Physics::NameConstants::GetSuperscriptMinus().c_str(),
                        Physics::NameConstants::GetSuperscriptOne().c_str()))
                ->Field("Grab Enable Toggle", &PhysicsGrabComponent::m_grabEnableToggle)
                ->Field("Maintain Grab", &PhysicsGrabComponent::m_grabMaintained)
                ->Field("Kinematic While Grabbing", &PhysicsGrabComponent::m_kinematicWhileHeld)
                ->Field("Disable Gravity", &PhysicsGrabComponent::m_disableGravityWhileHeld)
                ->Field("Offset Grab", &PhysicsGrabComponent::m_offsetGrab)
                ->Field("Tidal Lock Grabbed Object", &PhysicsGrabComponent::m_tidalLock)
                #ifdef FIRST_PERSON_CONTROLLER
                ->Field("Full Tidal Lock For First Person Controller", &PhysicsGrabComponent::m_fullTidalLockForFPC)
                #endif
                ->Field("Angular Damping", &PhysicsGrabComponent::m_tempObjectAngularDamping)
                ->Field("Linear Damping", &PhysicsGrabComponent::m_tempObjectLinearDamping)
                ->Field("Velocity Compensation", &PhysicsGrabComponent::m_velocityCompensation)
                ->Field("Velocity Compensation Damp Rate", &PhysicsGrabComponent::m_velocityCompDampRate)
                
                ->Field("Min Grab Distance", &PhysicsGrabComponent::m_minGrabDistance)
                ->Attribute(AZ::Edit::Attributes::Suffix, " " + Physics::NameConstants::GetLengthUnit())
                ->Field("Max Grab Distance", &PhysicsGrabComponent::m_maxGrabDistance)
                ->Attribute(AZ::Edit::Attributes::Suffix, " " + Physics::NameConstants::GetLengthUnit())
                ->Field("Grab Distance Speed", &PhysicsGrabComponent::m_grabDistanceSpeed)
                ->Attribute(AZ::Edit::Attributes::Suffix, " " + Physics::NameConstants::GetSpeedUnit())

                ->Field("Throw Impulse", &PhysicsGrabComponent::m_throwImpulse)
                ->Attribute(AZ::Edit::Attributes::Suffix, AZStd::string::format(" N%ss", Physics::NameConstants::GetInterpunct().c_str()))
                ->Field("Charge Throw", &PhysicsGrabComponent::m_enableChargeThrow)
                ->Field("Charge While Rotating", &PhysicsGrabComponent::m_enableChargeWhileRotating)
                ->Field("Min Throw Impulse", &PhysicsGrabComponent::m_minThrowImpulse)
                ->Attribute(AZ::Edit::Attributes::Suffix, AZStd::string::format(" N%ss", Physics::NameConstants::GetInterpunct().c_str()))
                ->Field("Max Throw Impulse", &PhysicsGrabComponent::m_maxThrowImpulse)
                ->Attribute(AZ::Edit::Attributes::Suffix, AZStd::string::format(" N%ss", Physics::NameConstants::GetInterpunct().c_str()))
                ->Field("Charge Time", &PhysicsGrabComponent::m_chargeTime)
                ->Attribute(AZ::Edit::Attributes::Suffix, " s")
                ->Field("Enable Mass Independent Throw", &PhysicsGrabComponent::m_massIndependentThrow)

                #ifdef FIRST_PERSON_CONTROLLER
                ->Field("Freeze Character Rotation", &PhysicsGrabComponent::m_freezeCharacterRotation)
                #endif
                ->Field("Dynamic Horizontal Rotate Scale", &PhysicsGrabComponent::m_dynamicYawRotateScale)
                ->Attribute(AZ::Edit::Attributes::Suffix, " rad/s")
                ->Field("Dynamic Vertical Rotate Scale", &PhysicsGrabComponent::m_dynamicPitchRotateScale)
                ->Attribute(AZ::Edit::Attributes::Suffix, " rad/s")
                ->Field("Kinematic Horizontal Rotate Scale", &PhysicsGrabComponent::m_kinematicYawRotateScale)
                ->Attribute(AZ::Edit::Attributes::Suffix, " rad/s")
                ->Field("Kinematic Vertical Rotate Scale", &PhysicsGrabComponent::m_kinematicPitchRotateScale)
                ->Attribute(AZ::Edit::Attributes::Suffix, " rad/s")
                ->Field("Rotate Enable Toggle", &PhysicsGrabComponent::m_rotateEnableToggle)
                ->Field("Gravity Applies To Point Rotation", &PhysicsGrabComponent::m_gravityAppliesToPointRotation)
                ->Field("Smooth Dynamic Rotation", &PhysicsGrabComponent::m_smoothDynamicRotation)
                ->Field("Angular Velocity Damp Rate", &PhysicsGrabComponent::m_angularVelocityDampRate)

                ->Field("PID Held Dynamics", &PhysicsGrabComponent::m_enablePIDHeldDynamics)
                ->Field("Mass Independent PID", &PhysicsGrabComponent::m_massIndependentHeldPID)
                ->Field("PID P Gain", &PhysicsGrabComponent::m_heldProportionalGain)
                ->Attribute(AZ::Edit::Attributes::Suffix, " N/m")
                ->Field("PID I Gain", &PhysicsGrabComponent::m_heldIntegralGain)
                ->Attribute(AZ::Edit::Attributes::Suffix, AZStd::string::format(" N/(m%ss)", Physics::NameConstants::GetInterpunct().c_str()))
                ->Field("PID D Gain", &PhysicsGrabComponent::m_heldDerivativeGain)
                ->Attribute(AZ::Edit::Attributes::Suffix, AZStd::string::format(" N%ss/m", Physics::NameConstants::GetInterpunct().c_str()))
                ->Field("PID Integral Limit", &PhysicsGrabComponent::m_heldIntegralWindupLimit)
                ->Attribute(AZ::Edit::Attributes::Suffix, " N")
                ->Field("PID Deriv Filter Alpha", &PhysicsGrabComponent::m_heldDerivativeFilterAlpha)

                ->Field("Enable PID Tidal Lock Dynamics", &PhysicsGrabComponent::m_enablePIDTidalLockDynamics)
                ->Field("Mass Independent Tidal Lock", &PhysicsGrabComponent::m_massIndependentTidalLock)
                ->Field("Scale Independent Tidal Lock", &PhysicsGrabComponent::m_scaleIndependentTidalLock)
                ->Field("Tidal Lock PID P Gain", &PhysicsGrabComponent::m_tidalLockProportionalGain)
                ->Attribute(AZ::Edit::Attributes::Suffix, AZStd::string::format(" N%sm/rad", Physics::NameConstants::GetInterpunct().c_str()))
                ->Field("Tidal Lock PID I Gain", &PhysicsGrabComponent::m_tidalLockIntegralGain)
                ->Attribute(AZ::Edit::Attributes::Suffix, AZStd::string::format(" N%sm/(rad%ss)", Physics::NameConstants::GetInterpunct().c_str(), Physics::NameConstants::GetInterpunct().c_str()))
                ->Field("Tidal Lock PID D Gain", &PhysicsGrabComponent::m_tidalLockDerivativeGain)
                ->Attribute(AZ::Edit::Attributes::Suffix, AZStd::string::format(" N%sm%ss/rad", Physics::NameConstants::GetInterpunct().c_str(), Physics::NameConstants::GetInterpunct().c_str()))
                ->Field("Tidal Lock PID Integral Limit", &PhysicsGrabComponent::m_tidalLockIntegralWindupLimit)
                ->Attribute(AZ::Edit::Attributes::Suffix, AZStd::string::format(" N%sm", Physics::NameConstants::GetInterpunct().c_str()))
                ->Field("Tidal Lock PID Deriv Filter Alpha", &PhysicsGrabComponent::m_tidalLockDerivativeFilterAlpha)
                ->Version(1);

            if (AZ::EditContext* ec = sc->GetEditContext())
            {
                using namespace AZ::Edit;
                ec->Class<PhysicsGrabComponent>("Physics Grab", "[Enables grabbing, holding, rotating, and throwing physics objects.]")
                    ->ClassElement(ClassElements::EditorData, "")
                    ->Attribute(Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "Physics Grab")

                    // Input Binding Keys
                    ->ClassElement(AZ::Edit::ClassElements::Group, "Input Configuration")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(nullptr, &PhysicsGrabComponent::m_strGrab, "Grab Key", "Grab interaction input binding")
                    ->DataElement(nullptr, &PhysicsGrabComponent::m_strGrabDistance, "Grab Distance Key", "Grab distance input binding")
                    ->DataElement(nullptr, &PhysicsGrabComponent::m_strThrow, "Throw Input Key", "Throw object input binding")
                    ->DataElement(nullptr, &PhysicsGrabComponent::m_strRotate, "Rotate Enable Key", "Enable rotate object input binding")
                    ->DataElement(nullptr, &PhysicsGrabComponent::m_strRotatePitch, "Rotate Pitch Key", "Rotate object about X axis input binding")
                    ->DataElement(nullptr, &PhysicsGrabComponent::m_strRotateYaw, "Rotate Yaw Key", "Rotate object about Z axis input binding")
                    ->DataElement(nullptr, &PhysicsGrabComponent::m_strRotateRoll, "Rotate Roll Key", "Rotate object about Y axis input binding")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Detection Parameters")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_grabbedCollisionGroupId,
                        "Sphere Cast Collision Group",
                        "The collision group which will be used for detecting grabbable objects.")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_tempGrabbedCollisionLayer,
                        "Grabbed Object Temporary Collision Layer",
                        "The temporary collision layer assigned to the grabbed object while it is being grabbed/held.")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_sphereCastRadius,
                        "Sphere Cast Radius",
                        "Sphere Cast radius used for grabbing objects")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_sphereCastDistance,
                        "Sphere Cast Distance",
                        "Sphere Cast distance along m_sphereCastDirection")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Hold Parameters")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        0,
                        &PhysicsGrabComponent::m_grabbingEntityId,
                        "Grab Entity",
                        "Reference entity that interacts with objects. If left blank, Camera entity will be used by default.")
                    ->DataElement(
                        0,
                        &PhysicsGrabComponent::m_meshSmoothing,
                        "Mesh Smoothing",
                        "Enables smooth interpolation of the mesh transform for dynamic objects to reduce stuttering.")
                    ->DataElement(
                        0,
                        &PhysicsGrabComponent::m_meshTagName,
                        "Mesh Tag",
                        "Tag name for the child entity to use for mesh interpolation (e.g., 'GrabMesh').")
                    #ifdef FIRST_PERSON_CONTROLLER
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_useFPControllerForGrab,
                        "Use First Person Controller For Grab",
                        "Use First Person Controller player character for grab reference. Enabling this creates tighter tracking by "
                        "bypassing "
                        "potential camera interpolation lag.")
                    #endif
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_grabResponse,
                        "Grab Response",
                        "Linear velocity scale applied when holding grabbed object. Only applies when PID Held Dynamics is disabled.")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_grabEnableToggle,
                        "Grab Enable Toggle",
                        "Determines whether pressing Grab Key toggles Grab mode. Disabling this requires the Grab key to be held to "
                        "maintain Grab mode.")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_grabMaintained,
                        "Maintain Grab",
                        "Grabbed Object remains held even if sphere cast no longer intersects it. This prevents the Grabbed Object from "
                        "flying off when quickly changing directions.")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_kinematicWhileHeld,
                        "Kinematic While Grabbing",
                        "Sets the grabbed object to kinematic.")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_disableGravityWhileHeld,
                        "Disable Gravity While Holding",
                        "Disables gravity for dynamic objects while being held.")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_offsetGrab,
                        "Offset Grab",
                        "When enabled for dynamic objects, applies forces at an offset point causing natural tilting. "
                        "Disable to apply at center of mass.")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_tidalLock,
                        "Tidal Lock Grabbed Object",
                        "Determines whether a Grabbed Object is tidal locked while being held. This means that the object will always "
                        "face the Grabbing Entity in it's current relative rotation.")
                    #ifdef FIRST_PERSON_CONTROLLER
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_fullTidalLockForFPC,
                        "Full Tidal Lock For First Person Controller",
                        "When enabled with First Person Controller, uses full camera rotation for tidal lock instead of character yaw "
                        "only.")
                    #endif
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_velocityCompensation,
                        "Velocity Compensation",
                        "Determines whether to compensate for velocity changes in grabbing entity. Enabling this will keep grab distance "
                        "the same whether you are walking, sprinting, or standing still.")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_velocityCompDampRate,
                        "Velocity Compensation Damp Rate",
                        "Gradually increase velocity compensation for dynamic object. Velocity compensation must be enabled for this to "
                        "take effect.")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_tempObjectLinearDamping,
                        "Linear Damping",
                        "Linear Damping of Grabbed Object while Grabbing")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_tempObjectAngularDamping,
                        "Angular Damping",
                        "Angular Damping of Grabbed Object while Holding or Rotating")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Distance Parameters")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_minGrabDistance,
                        "Min Grab Distance",
                        "Minimum allowable grab distance. Grabbed object cannot get closer than this distance.")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_maxGrabDistance,
                        "Max Grab Distance",
                        "Maximum allowable grab distance. Grabbed object cannot get further than this distance.")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_grabDistanceSpeed,
                        "Grab Distance Speed",
                        "The speed at which you move the grabbed object closer or away.")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Rotate Parameters")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_dynamicYawRotateScale,
                        "Dynamic Horizontal Rotate Scale",
                        "Angular velocity scale for horizontal (yaw) rotation of dynamic objects")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_dynamicPitchRotateScale,
                        "Dynamic Vertical Rotate Scale",
                        "Angular velocity scale for vertical (pitch) rotation of dynamic objects")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_kinematicYawRotateScale,
                        "Kinematic Horizontal Rotate Scale",
                        "Rotation speed scale for horizontal (yaw) rotation of kinematic objects")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_kinematicPitchRotateScale,
                        "Kinematic Vertical Rotate Scale",
                        "Rotation speed scale for vertical (pitch) rotation of kinematic objects")
                    #ifdef FIRST_PERSON_CONTROLLER
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_freezeCharacterRotation,
                        "Freeze Character Rotation",
                        "Enables character controller rotation while in Rotate State.")
                    #endif
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_rotateEnableToggle,
                        "Rotate Enable Toggle",
                        "Determines whether pressing Rotate Key toggles Rotate mode. Disabling this requires the Rotate key to be held to "
                        "maintain Rotate mode.") 
                        ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_smoothDynamicRotation,
                        "Smooth Dynamic Rotation",
                        "Enables smooth rotation for dynamic objects. Angular velocity is dampened and interpolated for smooth rotations.")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_angularVelocityDampRate,
                        "Angular Velocity Damp Rate",
                        "Gradually increase angular velocity for dynamic object. Smooth dynamic rotation must be enabled for this to take "
                        "effect.")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_gravityAppliesToPointRotation,
                        "Gravity Applies To Point Rotation",
                        "Enables gravity when rotating objects if Offset Grab enabled. "
                        "Only applies to when PID Tidal Lock Dynamics is enabled.")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Throw Parameters")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        nullptr, &PhysicsGrabComponent::m_throwImpulse, "Throw Impulse", "Throwing strength. The linear impulse scale applied when throwing grabbed " 
                        "object. Used when Chargeable Throw is disabled.")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_enableChargeThrow,
                        "Enable Chargeable Throw",
                        "If true, hold throw key to charge impulse from min to max.")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_enableChargeWhileRotating,
                        "Enable Charge While Rotating",
                        "If true, allow charging throw in rotate state; if false, only in hold state. Relevant when Chargeable Throw is "
                        "enabled.")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_minThrowImpulse,
                        "Min Throw Impulse",
                        "Minimum throw impulse for quick release. Used when Chargeable Throw is enabled.")
                    ->Attribute(
                        AZ::Edit::Attributes::Suffix, AZStd::string::format(" N%ss", Physics::NameConstants::GetInterpunct().c_str()))
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_maxThrowImpulse,
                        "Max Throw Impulse",
                        "Maximum throw impulse after full charge. Used when Chargeable Throw is enabled.")
                    ->Attribute(
                        AZ::Edit::Attributes::Suffix, AZStd::string::format(" N%ss", Physics::NameConstants::GetInterpunct().c_str()))
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_chargeTime,
                        "Charge Time",
                        "Time (seconds) to reach max impulse while holding. Used when Chargeable Throw is enabled.")
                    ->Attribute(AZ::Edit::Attributes::Suffix, " s")    
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_massIndependentThrow,
                        "Enable Mass Independent Throw",
                        "When enabled, throw impulse is scaled by object mass for consistent throw velocity regardless of mass "
                        "(mass-independent). Disable for realistic mass-dependent throws where heavier objects fly slower/shorter.")
                        
                    ->ClassElement(AZ::Edit::ClassElements::Group, "Advanced Hold Dynamics")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, false)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_enablePIDHeldDynamics,
                        "Enable PID Held Dynamics",
                        "Enables PID controller for dynamic held objects, creating spring-like (underdamped/overdamped) motion. Disabling "
                        "uses simple linear velocity.")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_massIndependentHeldPID,
                        "Enable Mass Independent PID",
                        "When enabled, PID controller scales forces by object mass for consistent behavior regardless of mass "
                        "(mass-independent). "
                        "Disable for realistic mass-dependent motion where heavier objects feel slower and harder to move.")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_heldProportionalGain,
                        "Held PID P Gain",
                        "Proportional gain when holding objects. Controls stiffness/pull strength (higher = stronger spring).")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_heldIntegralGain,
                        "Held PID I Gain",
                        "Integral gain when holding objects. Corrects persistent errors (e.g., gravity offset; usually low or 0).")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_heldDerivativeGain,
                        "Held PID D Gain",
                        "Derivative gain when holding objects. Controls damping (low = underdamped/oscillatory; high = overdamped/slow).")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_heldIntegralWindupLimit,
                        "Held PID Integral Limit",
                        "Anti-windup limit for integral accumulation (higher = stronger I but riskier).")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_heldDerivativeFilterAlpha,
                        "Held PID Derivative Filter Alpha",
                        "Derivative filter strength (0=raw, 1=heavy smoothing; 0.7 for responsive with light noise reduction).")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Advanced Tidal Lock Dynamics")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, false)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_enablePIDTidalLockDynamics,
                        "Enable PID Tidal Lock Dynamics",
                        "Enables PID controller for dynamic tidal lock rotation, creating spring-like motion. Disabling "
                        "uses simple angular velocity.")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_massIndependentTidalLock,
                        "Enable Mass Independent Tidal Lock",
                        "When enabled and PID is active, scales torque by object mass for consistent rotation regardless of mass. "
                        "Disable for realistic mass-dependent rotation where heavier objects rotate slower.")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_scaleIndependentTidalLock,
                        "Enable Scale Independent Tidal Lock",
                        "When enabled and PID is active, scales torque by approximate inertia factor (based on grab offset) for consistent "
                        "rotation oscillations regardless of object scale/size.")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_tidalLockProportionalGain,
                        "Tidal Lock PID P Gain",
                        "Proportional gain for tidal lock. Controls rotational stiffness (higher = faster facing).")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_tidalLockIntegralGain,
                        "Tidal Lock PID I Gain",
                        "Integral gain for tidal lock. Corrects persistent rotational errors (usually low or 0).")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_tidalLockDerivativeGain,
                        "Tidal Lock PID D Gain",
                        "Derivative gain for tidal lock. Controls rotational damping (higher = less oscillation).")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_tidalLockIntegralWindupLimit,
                        "Tidal Lock PID Integral Limit",
                        "Anti-windup limit for integral (higher = stronger I).")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_tidalLockDerivativeFilterAlpha,
                        "Tidal Lock PID Derivative Filter Alpha",
                        "Derivative filter (0=raw, 1=heavy smoothing).");
            }
        }

        if (auto bc = azrtti_cast<AZ::BehaviorContext*>(rc))
        {
            bc->EBus<PhysicsGrabNotificationBus>("PhysicsGrabNotificationBus", "PhysicsGrabComponentNotificationBus", "Notifications for Physics Grab Component")
                ->Handler<PhysicsGrabNotificationHandler>();

            // Reflect the enum
            bc->Enum<static_cast<int>(PhysicsGrabStates::idleState)>("PhysicsGrabStates_IdleState")
                ->Enum<static_cast<int>(PhysicsGrabStates::checkState)>("PhysicsGrabStates_CheckState")
                ->Enum<static_cast<int>(PhysicsGrabStates::holdState)>("PhysicsGrabStates_HoldState")
                ->Enum<static_cast<int>(PhysicsGrabStates::rotateState)>("PhysicsGrabStates_RotateState")
                ->Enum<static_cast<int>(PhysicsGrabStates::throwState)>("PhysicsGrabStates_ThrowState");

            bc->EBus<PhysicsGrabComponentRequestBus>("PhysicsGrabComponentRequestBus")
                ->Attribute(AZ::Script::Attributes::Scope, AZ::Script::Attributes::ScopeFlags::Common)
                ->Attribute(AZ::Script::Attributes::Module, "interaction")
                ->Attribute(AZ::Script::Attributes::Category, "Physics Grab")
                ->Event("Get Grabbing EntityId", &PhysicsGrabComponentRequests::GetGrabbingEntityId)
                ->Event("Get Active Camera EntityId", &PhysicsGrabComponentRequests::GetActiveCameraEntityId)
                ->Event("Get Grabbed Object EntityId", &PhysicsGrabComponentRequests::GetGrabbedObjectEntityId)
                ->Event("Get Last Grabbed Object EntityId", &PhysicsGrabComponentRequests::GetLastGrabbedObjectEntityId)
                ->Event("Get Thrown Grabbed Object EntityId", &PhysicsGrabComponentRequests::GetThrownGrabbedObjectEntityId)
                ->Event("Set Thrown Grabbed Object EntityId", &PhysicsGrabComponentRequests::SetThrownGrabbedObjectEntityId)
                ->Event("Set Grabbing Entity", &PhysicsGrabComponentRequests::SetGrabbingEntity)
                ->Event("Get Grabbed Collision Group", &PhysicsGrabComponentRequests::GetGrabbedCollisionGroup)
                ->Event("Set Grabbed Collision Group", &PhysicsGrabComponentRequests::SetGrabbedCollisionGroup)
                ->Event("Get Current Grabbed Collision Layer Name", &PhysicsGrabComponentRequests::GetCurrentGrabbedCollisionLayerName)
                ->Event("Set Current Grabbed Collision Layer By Name", &PhysicsGrabComponentRequests::SetCurrentGrabbedCollisionLayerByName)
                ->Event("Get Current Grabbed Collision Layer", &PhysicsGrabComponentRequests::GetCurrentGrabbedCollisionLayer)
                ->Event("Set Current Grabbed Collision Layer", &PhysicsGrabComponentRequests::SetCurrentGrabbedCollisionLayer)
                ->Event("Get Previous Grabbed Collision Layer Name", &PhysicsGrabComponentRequests::GetPrevGrabbedCollisionLayerName)
                ->Event("Set Previous Grabbed Collision Layer Name By Name", &PhysicsGrabComponentRequests::SetPrevGrabbedCollisionLayerByName)
                ->Event("Get Previous Grabbed Collision Layer", &PhysicsGrabComponentRequests::GetPrevGrabbedCollisionLayer)
                ->Event("Set Previous Grabbed Collision Layer", &PhysicsGrabComponentRequests::SetPrevGrabbedCollisionLayer)
                ->Event("Get Temporary Grabbed Collision Layer Name", &PhysicsGrabComponentRequests::GetTempGrabbedCollisionLayerName)
                ->Event("Set Temporary Grabbed Collision Layer By Name", &PhysicsGrabComponentRequests::SetTempGrabbedCollisionLayerByName)
                ->Event("Get Temporary Grabbed Collision Layer", &PhysicsGrabComponentRequests::GetTempGrabbedCollisionLayer)
                ->Event("Set Temporary Grabbed Collision Layer", &PhysicsGrabComponentRequests::SetTempGrabbedCollisionLayer)
                ->Event("Get State String", &PhysicsGrabComponentRequests::GetStateString)
                ->Event("Get Is In Idle State", &PhysicsGrabComponentRequests::GetIsInIdleState)
                ->Event("Get Is In Check State", &PhysicsGrabComponentRequests::GetIsInCheckState)
                ->Event("Get Is In Held State", &PhysicsGrabComponentRequests::GetIsInHeldState)
                ->Event("Get Is In Rotate State", &PhysicsGrabComponentRequests::GetIsInRotateState)
                ->Event("Get Is In Throw State", &PhysicsGrabComponentRequests::GetIsInThrowState)
                ->Event("Get Object Sphere Cast Hit", &PhysicsGrabComponentRequests::GetObjectSphereCastHit)
                ->Event("Get Stay In Idle State", &PhysicsGrabComponentRequests::GetStayInIdleState)
                ->Event("Set Stay In Idle State", &PhysicsGrabComponentRequests::SetStayInIdleState)
                ->Event("Get Grab Enable Toggle", &PhysicsGrabComponentRequests::GetGrabEnableToggle)
                ->Event("Set Grab Enable Toggle", &PhysicsGrabComponentRequests::SetGrabEnableToggle)
                ->Event("Get Rotate Enable Toggle", &PhysicsGrabComponentRequests::GetRotateEnableToggle)
                ->Event("Set Rotate Enable Toggle", &PhysicsGrabComponentRequests::SetRotateEnableToggle)
                ->Event("Get Grab Key Value", &PhysicsGrabComponentRequests::GetGrabKeyValue)
                ->Event("Set Grab Key Value", &PhysicsGrabComponentRequests::SetGrabKeyValue)
                ->Event("Get Throw Key Value", &PhysicsGrabComponentRequests::GetThrowKeyValue)
                ->Event("Set Throw Key Value", &PhysicsGrabComponentRequests::SetThrowKeyValue)
                ->Event("Get Rotate Key Value", &PhysicsGrabComponentRequests::GetRotateKeyValue)
                ->Event("Set Rotate Key Value", &PhysicsGrabComponentRequests::SetRotateKeyValue)
                ->Event("Get Pitch Key Value", &PhysicsGrabComponentRequests::GetPitchKeyValue)
                ->Event("Set Pitch Key Value", &PhysicsGrabComponentRequests::SetPitchKeyValue)
                ->Event("Get Yaw Key Value", &PhysicsGrabComponentRequests::GetYawKeyValue)
                ->Event("Set Yaw Key Value", &PhysicsGrabComponentRequests::SetYawKeyValue)
                ->Event("Get Roll Key Value", &PhysicsGrabComponentRequests::GetRollKeyValue)
                ->Event("Set Roll Key Value", &PhysicsGrabComponentRequests::SetRollKeyValue)
                ->Event("Get Grab Distance Key Value", &PhysicsGrabComponentRequests::GetGrabbedDistanceKeyValue)
                ->Event("Set Grab Distance Key Value", &PhysicsGrabComponentRequests::SetGrabbedDistanceKeyValue)         
                ->Event("Get Grabbed Object Distance", &PhysicsGrabComponentRequests::GetGrabbedObjectDistance)
                ->Event("Set Grabbed Object Distance", &PhysicsGrabComponentRequests::SetGrabbedObjectDistance)
                ->Event("Get Minimum Grabbed Object Distance", &PhysicsGrabComponentRequests::GetMinGrabbedObjectDistance)
                ->Event("Set Minimum Grabbed Object Distance", &PhysicsGrabComponentRequests::SetMinGrabbedObjectDistance)
                ->Event("Get Maximum Grabbed Object Distance", &PhysicsGrabComponentRequests::GetMaxGrabbedObjectDistance)
                ->Event("Set Maximum Grabbed Object Distance", &PhysicsGrabComponentRequests::SetMaxGrabbedObjectDistance)
                ->Event("Get Grabbed Object Distance Speed", &PhysicsGrabComponentRequests::GetGrabbedObjectDistanceSpeed)
                ->Event("Set Grabbed Object Distance Speed", &PhysicsGrabComponentRequests::SetGrabbedObjectDistanceSpeed)
                ->Event("Get Grab Response", &PhysicsGrabComponentRequests::GetGrabResponse)
                ->Event("Set Grab Response", &PhysicsGrabComponentRequests::SetGrabResponse)
                ->Event("Get Dynamic Object Tidal Lock", &PhysicsGrabComponentRequests::GetDynamicTidalLock)
                ->Event("Set Dynamic Object Tidal Lock", &PhysicsGrabComponentRequests::SetDynamicTidalLock)
                ->Event("Get Kinematic Object Tidal Lock", &PhysicsGrabComponentRequests::GetKinematicTidalLock)
                ->Event("Set Kinematic Object Tidal Lock", &PhysicsGrabComponentRequests::SetKinematicTidalLock)
                ->Event("Get Object Tidal Lock", &PhysicsGrabComponentRequests::GetTidalLock)
                ->Event("Set Object Tidal Lock", &PhysicsGrabComponentRequests::SetTidalLock)
                ->Event("Get Full Tidal Lock For First Person Controller", &PhysicsGrabComponentRequests::GetFullTidalLockForFPC)
                ->Event("Set Full Tidal Lock For First Person Controller", &PhysicsGrabComponentRequests::SetFullTidalLockForFPC)
                ->Event("Get Dynamic Yaw Rotate Scale", &PhysicsGrabComponentRequests::GetDynamicYawRotateScale)
                ->Event("Set Dynamic Yaw Rotate Scale", &PhysicsGrabComponentRequests::SetDynamicYawRotateScale)
                ->Event("Get Dynamic Pitch Rotate Scale", &PhysicsGrabComponentRequests::GetDynamicPitchRotateScale)
                ->Event("Set Dynamic Pitch Rotate Scale", &PhysicsGrabComponentRequests::SetDynamicPitchRotateScale)
                ->Event("Get Dynamic Roll Rotate Scale", &PhysicsGrabComponentRequests::GetDynamicRollRotateScale)
                ->Event("Set Dynamic Roll Rotate Scale", &PhysicsGrabComponentRequests::SetDynamicRollRotateScale)
                ->Event("Get Kinematic Yaw Rotate Scale", &PhysicsGrabComponentRequests::GetKinematicYawRotateScale)
                ->Event("Set Kinematic Yaw Rotate Scale", &PhysicsGrabComponentRequests::SetKinematicYawRotateScale)
                ->Event("Get Kinematic Pitch Rotate Scale", &PhysicsGrabComponentRequests::GetKinematicPitchRotateScale)
                ->Event("Set Kinematic Pitch Rotate Scale", &PhysicsGrabComponentRequests::SetKinematicPitchRotateScale)
                ->Event("Get Kinematic Roll Rotate Scale", &PhysicsGrabComponentRequests::GetKinematicPitchRotateScale)
                ->Event("Set Kinematic Roll Rotate Scale", &PhysicsGrabComponentRequests::SetKinematicPitchRotateScale)
                ->Event("Get Velocity Compensation Damp Rate", &PhysicsGrabComponentRequests::GetVelocityCompDampRate)
                ->Event("Set Velocity Compensation Damp Rate", &PhysicsGrabComponentRequests::SetVelocityCompDampRate)
                ->Event("Get Angular Velocity Damp Rate", &PhysicsGrabComponentRequests::GetAngularVelocityDampRate)
                ->Event("Set Angular Velocity Damp Rate", &PhysicsGrabComponentRequests::SetAngularVelocityDampRate)
                ->Event("Get Velocity Compensation", &PhysicsGrabComponentRequests::GetVelocityCompensation)
                ->Event("Set Velocity Compensation", &PhysicsGrabComponentRequests::SetVelocityCompensation)
                ->Event("Get Smooth Dynamic Rotation", &PhysicsGrabComponentRequests::GetSmoothDynamicRotation)
                ->Event("Set Smooth Dynamic Rotation", &PhysicsGrabComponentRequests::SetSmoothDynamicRotation)
                ->Event("Get Grab Throw Impulse", &PhysicsGrabComponentRequests::GetThrowImpulse)
                ->Event("Set Grab Throw Impulse", &PhysicsGrabComponentRequests::SetThrowImpulse)
                ->Event("Get Grabbed Object Throw State Counter", &PhysicsGrabComponentRequests::GetGrabbedObjectThrowStateCounter)
                ->Event("Set Grabbed Object Throw State Counter", &PhysicsGrabComponentRequests::SetGrabbedObjectThrowStateCounter)
                ->Event("Get Grabbed Object Throw State Max Time", &PhysicsGrabComponentRequests::GetGrabbedObjectThrowStateTime)
                ->Event("Set Grabbed Object Throw State Max Time", &PhysicsGrabComponentRequests::SetGrabbedObjectThrowStateTime)
                ->Event("Get Enable Charge Throw", &PhysicsGrabComponentRequests::GetEnableChargeThrow)
                ->Event("Set Enable Charge Throw", &PhysicsGrabComponentRequests::SetEnableChargeThrow)
                ->Event("Get Enable Charge While Rotating", &PhysicsGrabComponentRequests::GetEnableChargeWhileRotating)
                ->Event("Set Enable Charge While Rotating", &PhysicsGrabComponentRequests::SetEnableChargeWhileRotating)
                ->Event("Get Min Throw Impulse", &PhysicsGrabComponentRequests::GetMinThrowImpulse)
                ->Event("Set Min Throw Impulse", &PhysicsGrabComponentRequests::SetMinThrowImpulse)
                ->Event("Get Max Throw Impulse", &PhysicsGrabComponentRequests::GetMaxThrowImpulse)
                ->Event("Set Max Throw Impulse", &PhysicsGrabComponentRequests::SetMaxThrowImpulse)
                ->Event("Get Current Throw Impulse", &PhysicsGrabComponentRequests::GetCurrentThrowImpulse)
                ->Event("Get Charge Time", &PhysicsGrabComponentRequests::GetChargeTime)
                ->Event("Set Charge Time", &PhysicsGrabComponentRequests::SetChargeTime)
                ->Event("Get Current Charge Time", &PhysicsGrabComponentRequests::GetCurrentChargeTime)
                ->Event("Get Is Charging Throw", &PhysicsGrabComponentRequests::GetIsChargingThrow)
                ->Event("Get Grab Sphere Cast Radius", &PhysicsGrabComponentRequests::GetSphereCastRadius)
                ->Event("Set Grab Sphere Cast Radius", &PhysicsGrabComponentRequests::SetSphereCastRadius)
                ->Event("Get Grab Sphere Cast Distance", &PhysicsGrabComponentRequests::GetSphereCastDistance)
                ->Event("Set Grab Sphere Cast Distance", &PhysicsGrabComponentRequests::SetSphereCastDistance)
                ->Event("Get Grabbed Object Is Kinematic", &PhysicsGrabComponentRequests::GetGrabbedObjectKinematicElseDynamic)
                ->Event("Set Grabbed Object Is Kinematic", &PhysicsGrabComponentRequests::SetGrabbedObjectKinematicElseDynamic)
                ->Event("Get Current Grabbed Object Angular Damping", &PhysicsGrabComponentRequests::GetCurrentGrabbedObjectAngularDamping)
                ->Event("Set Current Grabbed Object Angular Damping", &PhysicsGrabComponentRequests::SetCurrentGrabbedObjectAngularDamping)
                ->Event("Get Previous Grabbed Object Angular Damping", &PhysicsGrabComponentRequests::GetPrevGrabbedObjectAngularDamping)
                ->Event("Set Previous Grabbed Object Angular Damping", &PhysicsGrabComponentRequests::SetPrevGrabbedObjectAngularDamping)
                ->Event("Get Temporary Grabbed Object Angular Damping", &PhysicsGrabComponentRequests::GetTempGrabbedObjectAngularDamping)
                ->Event("Set Temporary Grabbed Object Angular Damping", &PhysicsGrabComponentRequests::SetTempGrabbedObjectAngularDamping)
                ->Event("Get Current Grabbed Object Linear Damping", &PhysicsGrabComponentRequests::GetCurrentGrabbedObjectLinearDamping)
                ->Event("Set Current Grabbed Object Linear Damping", &PhysicsGrabComponentRequests::SetCurrentGrabbedObjectLinearDamping)
                ->Event("Get Previous Grabbed Object Linear Damping", &PhysicsGrabComponentRequests::GetPrevGrabbedObjectLinearDamping)
                ->Event("Set Previous Grabbed Object Linear Damping", &PhysicsGrabComponentRequests::SetPrevGrabbedObjectLinearDamping)
                ->Event("Get Temporary Grabbed Object Linear Damping", &PhysicsGrabComponentRequests::GetTempGrabbedObjectLinearDamping)
                ->Event("Set Temporary Grabbed Object Linear Damping", &PhysicsGrabComponentRequests::SetTempGrabbedObjectLinearDamping)
                ->Event("Get Initial Angular Velocity Zero", &PhysicsGrabComponentRequests::GetInitialAngularVelocityZero)
                ->Event("Set Initial Angular Velocity Zero", &PhysicsGrabComponentRequests::SetInitialAngularVelocityZero)
                ->Event("Force State Transition", &PhysicsGrabComponentRequests::ForceTransition)
                ->Event("Set Locked State Transition", &PhysicsGrabComponentRequests::SetStateLocked)
                ->Event("Get Locked State Transition", &PhysicsGrabComponentRequests::GetStateLocked)
                ->Event("Get Disable Gravity While Holding", &PhysicsGrabComponentRequests::GetDisableGravityWhileHeld)
                ->Event("Set Disable Gravity While Holding", &PhysicsGrabComponentRequests::SetDisableGravityWhileHeld)
                ->Event("Get Offset Grab", &PhysicsGrabComponentRequests::GetOffsetGrab)
                ->Event("Set Offset Grab", &PhysicsGrabComponentRequests::SetOffsetGrab)
                ->Event("Get Gravity Applies To Point Rotation", &PhysicsGrabComponentRequests::GetGravityAppliesToPointRotation)
                ->Event("Set Gravity Applies To Point Rotation", &PhysicsGrabComponentRequests::SetGravityAppliesToPointRotation)
                ->Event("Get Mass Independent Throw", &PhysicsGrabComponentRequests::GetMassIndependentThrow)
                ->Event("Set Mass Independent Throw", &PhysicsGrabComponentRequests::SetMassIndependentThrow)
                ->Event("Get Enable PID Held Dynamics", &PhysicsGrabComponentRequests::GetEnablePIDHeldDynamics)
                ->Event("Set Enable PID Held Dynamics", &PhysicsGrabComponentRequests::SetEnablePIDHeldDymamics)
                ->Event("Get Mass Independent Held PID", &PhysicsGrabComponentRequests::GetMassIndependentHeldPID)
                ->Event("Set Mass Independent Held PID", &PhysicsGrabComponentRequests::SetMassIndependentHeldPID)
                ->Event("Get Held Proportional Gain", &PhysicsGrabComponentRequests::GetHeldProportionalGain)
                ->Event("Set Held Proportional Gain", &PhysicsGrabComponentRequests::SetHeldProportionalGain)
                ->Event("Get Held Integral Gain", &PhysicsGrabComponentRequests::GetHeldIntegralGain)
                ->Event("Set Held Integral Gain", &PhysicsGrabComponentRequests::SetHeldIntegralGain)
                ->Event("Get Held Derivative Gain", &PhysicsGrabComponentRequests::GetHeldDerivativeGain)
                ->Event("Set Held Derivative Gain", &PhysicsGrabComponentRequests::SetHeldDerivativeGain)
                ->Event("Get Held Integral Windup Limit", &PhysicsGrabComponentRequests::GetHeldIntegralWindupLimit)
                ->Event("Set Held Integral Windup Limit", &PhysicsGrabComponentRequests::SetHeldIntegralWindupLimit)
                ->Event("Get Held Derivative Filter Alpha", &PhysicsGrabComponentRequests::GetHeldDerivativeFilterAlpha)
                ->Event("Set Held Derivative Filter Alpha", &PhysicsGrabComponentRequests::SetHeldDerivativeFilterAlpha)
                ->Event("Get Held Derivative Mode", &PhysicsGrabComponentRequests::GetHeldDerivativeMode)
                ->Event("Set Held Derivative Mode", &PhysicsGrabComponentRequests::SetHeldDerivativeMode)
                ->Event("Get Enable PID Tidal Lock Dynamics", &PhysicsGrabComponentRequests::GetEnablePIDTidalLockDynamics)
                ->Event("Set Enable PID Tidal Lock Dynamics", &PhysicsGrabComponentRequests::SetEnablePIDTidalLockDynamics)
                ->Event("Get Mass Independent Tidal Lock", &PhysicsGrabComponentRequests::GetMassIndependentTidalLock)
                ->Event("Set Mass Independent Tidal Lock", &PhysicsGrabComponentRequests::SetMassIndependentTidalLock)
                ->Event("Get Scale Independent Tidal Lock", &PhysicsGrabComponentRequests::GetScaleIndependentTidalLock)
                ->Event("Set Scale Independent Tidal Lock", &PhysicsGrabComponentRequests::SetScaleIndependentTidalLock)
                ->Event("Get Tidal Lock Proportional Gain", &PhysicsGrabComponentRequests::GetTidalLockProportionalGain)
                ->Event("Set Tidal Lock Proportional Gain", &PhysicsGrabComponentRequests::SetTidalLockProportionalGain)
                ->Event("Get Tidal Lock Integral Gain", &PhysicsGrabComponentRequests::GetTidalLockIntegralGain)
                ->Event("Set Tidal Lock Integral Gain", &PhysicsGrabComponentRequests::SetTidalLockIntegralGain)
                ->Event("Get Tidal Lock Derivative Gain", &PhysicsGrabComponentRequests::GetTidalLockDerivativeGain)
                ->Event("Set Tidal Lock Derivative Gain", &PhysicsGrabComponentRequests::SetTidalLockDerivativeGain)
                ->Event("Get Tidal Lock Integral Windup Limit", &PhysicsGrabComponentRequests::GetTidalLockIntegralWindupLimit)
                ->Event("Set Tidal Lock Integral Windup Limit", &PhysicsGrabComponentRequests::SetTidalLockIntegralWindupLimit)
                ->Event("Get Tidal Lock Derivative Filter Alpha", &PhysicsGrabComponentRequests::GetTidalLockDerivativeFilterAlpha)
                ->Event("Set Tidal Lock Derivative Filter Alpha", &PhysicsGrabComponentRequests::SetTidalLockDerivativeFilterAlpha)
                ->Event("Get Tidal Lock Derivative Mode", &PhysicsGrabComponentRequests::GetTidalLockDerivativeMode)
                ->Event("Set Tidal Lock Derivative Mode", &PhysicsGrabComponentRequests::SetTidalLockDerivativeMode)
                ->Event("GetGrabInputKey", &PhysicsGrabComponentRequests::GetGrabInputKey)
                ->Event("SetGrabInputKey", &PhysicsGrabComponentRequests::SetGrabInputKey)
                ->Event("GetThrowInputKey", &PhysicsGrabComponentRequests::GetThrowInputKey)
                ->Event("SetThrowInputKey", &PhysicsGrabComponentRequests::SetThrowInputKey)
                ->Event("GetRotateInputKey", &PhysicsGrabComponentRequests::GetRotateInputKey)
                ->Event("SetRotateInputKey", &PhysicsGrabComponentRequests::SetRotateInputKey)
                ->Event("GetRotatePitchInputKey", &PhysicsGrabComponentRequests::GetRotatePitchInputKey)
                ->Event("SetRotatePitchInputKey", &PhysicsGrabComponentRequests::SetRotatePitchInputKey)
                ->Event("GetRotateYawInputKey", &PhysicsGrabComponentRequests::GetRotateYawInputKey)
                ->Event("SetRotateYawInputKey", &PhysicsGrabComponentRequests::SetRotateYawInputKey)
                ->Event("GetRotateRollInputKey", &PhysicsGrabComponentRequests::GetRotateRollInputKey)
                ->Event("SetRotateRollInputKey", &PhysicsGrabComponentRequests::SetRotateRollInputKey)
                ->Event("GetGrabDistanceInputKey", &PhysicsGrabComponentRequests::GetGrabDistanceInputKey)
                ->Event("SetGrabDistanceInputKey", &PhysicsGrabComponentRequests::SetGrabDistanceInputKey)
                ->Event("GetMeshEntityId", &PhysicsGrabComponentRequests::GetMeshEntityId)
                ->Event("SetMeshEntityId", &PhysicsGrabComponentRequests::SetMeshEntityId)
                ->Event("GetMeshTagName", &PhysicsGrabComponentRequests::GetMeshTagName)
                ->Event("SetMeshTagName", &PhysicsGrabComponentRequests::SetMeshTagName);

            bc->Class<PhysicsGrabComponent>()->RequestBus("PhysicsGrabComponentRequestBus");
        }
    }

    void PhysicsGrabComponent::Activate()
    {
        m_grabEventId = StartingPointInput::InputEventNotificationId(m_strGrab.c_str());
        InputEventNotificationBus::MultiHandler::BusConnect(m_grabEventId);

        m_grabDistanceEventId = StartingPointInput::InputEventNotificationId(m_strGrabDistance.c_str());
        InputEventNotificationBus::MultiHandler::BusConnect(m_grabDistanceEventId);

        m_throwEventId = StartingPointInput::InputEventNotificationId(m_strThrow.c_str());
        InputEventNotificationBus::MultiHandler::BusConnect(m_throwEventId);

        m_rotateEventId = StartingPointInput::InputEventNotificationId(m_strRotate.c_str());
        InputEventNotificationBus::MultiHandler::BusConnect(m_rotateEventId);

        m_rotatePitchEventId = StartingPointInput::InputEventNotificationId(m_strRotatePitch.c_str());
        InputEventNotificationBus::MultiHandler::BusConnect(m_rotatePitchEventId);

        m_rotateYawEventId = StartingPointInput::InputEventNotificationId(m_strRotateYaw.c_str());
        InputEventNotificationBus::MultiHandler::BusConnect(m_rotateYawEventId);

        m_rotateRollEventId = StartingPointInput::InputEventNotificationId(m_strRotateRoll.c_str());
        InputEventNotificationBus::MultiHandler::BusConnect(m_rotateRollEventId);

        AZ::TickBus::Handler::BusConnect();

        Physics::CollisionRequestBus::BroadcastResult(
            m_grabbedCollisionGroup, &Physics::CollisionRequests::GetCollisionGroupById, m_grabbedCollisionGroupId);

        // Connect to physics simulation start event
        Physics::DefaultWorldBus::BroadcastResult(m_attachedSceneHandle, &Physics::DefaultWorldRequests::GetDefaultSceneHandle);
        if (m_attachedSceneHandle == AzPhysics::InvalidSceneHandle)
        {
            AZ_Error("Physics Grab Component", false, "Failed to retrieve default scene.");
            return;
        }

        // Register m_sceneSimulationStartHandler to listen for the OnSceneSimulationStart event, which 
        // is triggered at the start of each physics simulation step
        m_sceneSimulationStartHandler = AzPhysics::SceneEvents::OnSceneSimulationStartHandler(
            [this]([[maybe_unused]] AzPhysics::SceneHandle sceneHandle, float fixedDeltaTime)
            {
                OnSceneSimulationStart(fixedDeltaTime);
            }, aznumeric_cast<int32_t>(AzPhysics::SceneEvents::PhysicsStartFinishSimulationPriority::Physics));

        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        if (sceneInterface != nullptr)
        {
            sceneInterface->RegisterSceneSimulationStartHandler(m_attachedSceneHandle, m_sceneSimulationStartHandler);
        }

        m_sceneSimulationFinishHandler = AzPhysics::SceneEvents::OnSceneSimulationFinishHandler(
            [this](AzPhysics::SceneHandle sceneHandle, float fixedDeltaTime)
            {
                OnSceneSimulationFinish(sceneHandle, fixedDeltaTime);
            }, aznumeric_cast<int32_t>(AzPhysics::SceneEvents::PhysicsStartFinishSimulationPriority::Physics));

        if (sceneInterface != nullptr)
        {
            sceneInterface->RegisterSceneSimulationFinishHandler(m_attachedSceneHandle, m_sceneSimulationFinishHandler);
        }
        // Connect the handler to the request bus
        PhysicsGrabComponentRequestBus::Handler::BusConnect(GetEntityId());

        // Delaying the assignment of Grabbing Entity to OnEntityActivated so the Entity is activated and ready
        AZ::EntityBus::Handler::BusConnect(m_grabbingEntityId);

        // Initialize Held Object PID controller
        m_pidController = PidController<AZ::Vector3>(
            m_heldProportionalGain,
            m_heldIntegralGain,
            m_heldDerivativeGain,
            m_heldIntegralWindupLimit,
            m_heldDerivativeFilterAlpha,
            m_heldDerivativeMode);

        // Initialize Tidal Lock PID controller
        m_tidalLockPidController = PidController<AZ::Vector3>(
            m_tidalLockProportionalGain,
            m_tidalLockIntegralGain,
            m_tidalLockDerivativeGain,
            m_tidalLockIntegralWindupLimit,
            m_tidalLockDerivativeFilterAlpha,
            m_tidalLockDerivativeMode);
    }

    // Called at the beginning of each physics tick
    void PhysicsGrabComponent::OnSceneSimulationStart(float physicsTimestep)
    {
        // Update physics timestep
        m_physicsTimestep = physicsTimestep;

        if (!m_isObjectKinematic && (m_state == PhysicsGrabStates::holdState || m_state == PhysicsGrabStates::rotateState))
        {
            ProcessStates(physicsTimestep, true);
        }

        // Reset time accumulator
        m_physicsTimeAccumulator = 0.0f;
    }

    void PhysicsGrabComponent::OnSceneSimulationFinish(
        [[maybe_unused]] AzPhysics::SceneHandle sceneHandle, [[maybe_unused]] float fixedDeltaTime)
    {
        if (m_lastGrabbedObjectEntityId.IsValid() && !m_isObjectKinematic && m_meshSmoothing &&
            (m_state == PhysicsGrabStates::holdState || m_state == PhysicsGrabStates::rotateState))
        {
            m_prevPhysicsTransform = m_currentPhysicsTransform;
            AZ::TransformBus::EventResult(m_currentPhysicsTransform, m_lastGrabbedObjectEntityId, &AZ::TransformInterface::GetWorldTM);
        }
    }

    void PhysicsGrabComponent::OnEntityActivated([[maybe_unused]] const AZ::EntityId& entityId)
    {
        AZ::EntityBus::Handler::BusDisconnect();

        if (m_grabbingEntityId.IsValid())
        {
            m_grabbingEntityPtr = GetEntityPtr(entityId);
        }
    }

    void PhysicsGrabComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        InputEventNotificationBus::MultiHandler::BusDisconnect();
        PhysicsGrabComponentRequestBus::Handler::BusDisconnect();

        m_attachedSceneHandle = AzPhysics::InvalidSceneHandle;
        m_sceneSimulationStartHandler.Disconnect();
        m_sceneSimulationFinishHandler.Disconnect();
        m_meshEntityPtr = nullptr;
    }

    void PhysicsGrabComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("InputConfigurationService"));
        required.push_back(AZ_CRC_CE("TransformService"));
    }

    void PhysicsGrabComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("GrabService"));
    }

    void PhysicsGrabComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("GrabService"));
        incompatible.push_back(AZ_CRC_CE("InputService"));
    }

    void PhysicsGrabComponent::GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        dependent.push_back(AZ_CRC_CE("FirstPersonControllerService"));
    }

    AZ::Entity* PhysicsGrabComponent::GetActiveCameraEntityPtr() const
    {
        AZ::EntityId activeCameraId;
        Camera::CameraSystemRequestBus::BroadcastResult(activeCameraId, &Camera::CameraSystemRequestBus::Events::GetActiveCamera);

        auto ca = AZ::Interface<AZ::ComponentApplicationRequests>::Get();
        return ca->FindEntity(activeCameraId);
    }

    // Recieve the input event in OnPressed method
    void PhysicsGrabComponent::OnPressed(float value)
    {
        const InputEventNotificationId* inputId = InputEventNotificationBus::GetCurrentBusId();
        if (inputId == nullptr)
        {
            return;
        }

        if (*inputId == m_grabEventId)
        {
            m_grabKeyValue = value;
            // AZ_Printf("Player", "Grab value %f", value);
        }

        if (*inputId == m_grabDistanceEventId)
        {
            m_grabDistanceKeyValue = value;
            // AZ_Printf("Object", "Grab Distance value %f", value);
        }

        if (*inputId == m_throwEventId)
        {
            m_throwKeyValue = value;
            // AZ_Printf("Player", "Throw value %f", value);
        }

        if (*inputId == m_rotateEventId)
        {
            m_rotateKeyValue = value;
            // AZ_Printf("Player", "rotate value %f", value);
        }

        if (*inputId == m_rotatePitchEventId)
        {
            m_pitchKeyValue = value;
            // AZ_Printf("Object", "Grab Object pitch value %f", value);
        }

        if (*inputId == m_rotateYawEventId)
        {
            m_yawKeyValue = value;
            // AZ_Printf("Object", "Grab Object yaw value %f", value);
        }

        if (*inputId == m_rotateRollEventId)
        {
            m_rollKeyValue = value;
            // AZ_Printf("Object", "Grab Object roll value %f", value);
        }
    }

    // Recieve the input event in OnReleased method
    void PhysicsGrabComponent::OnReleased(float value)
    {
        const InputEventNotificationId* inputId = InputEventNotificationBus::GetCurrentBusId();
        if (inputId == nullptr)
        {
            return;
        }
        if (*inputId == m_grabEventId)
        {
            m_grabKeyValue = value;
            // AZ_Printf("Player", "Grab released value %f", value);
        }
        if (*inputId == m_grabDistanceEventId)
        {
            m_grabDistanceKeyValue = value;
            // AZ_Printf("Player", "Grab Distance released value %f", value);
        }
        if (*inputId == m_throwEventId)
        {
            m_throwKeyValue = value;
            // AZ_Printf("Player", "Throw released value %f", value);
        }
        if (*inputId == m_rotateEventId)
        {
            m_rotateKeyValue = value;
            // AZ_Printf("Player", "Throw released value %f", value);
        }

        if (*inputId == m_rotatePitchEventId)
        {
            m_pitchKeyValue = value;
            // AZ_Printf("Object", "Grab Object pitch released value %f", value);
        }

        if (*inputId == m_rotateYawEventId)
        {
            m_yawKeyValue = value;
            // AZ_Printf("Object", "Grab Object yaw released value %f", value);
        }

        if (*inputId == m_rotateRollEventId)
        {
            m_rollKeyValue = value;
            // AZ_Printf("Object", "Grab Object roll released value %f", value);
        }
    }

    void PhysicsGrabComponent::OnHeld(float value)
    {
        const InputEventNotificationId* inputId = InputEventNotificationBus::GetCurrentBusId();
        if (inputId == nullptr)
        {
            return;
        }
        if (*inputId == m_throwEventId)
        {
            m_throwKeyValue = value;
            // AZ_Printf("Player", "Throw held value %f", value);
        }
        
        if (*inputId == m_rotatePitchEventId)
        {
            m_pitchKeyValue = value;
            // AZ_Printf("Object", "Grab Object pitch held value %f", value);
        }
        else if (*inputId == m_rotateYawEventId)
        {
            m_yawKeyValue = value;
            // AZ_Printf("Object", "Grab Object yaw held value %f", value);
        }
        else if (*inputId == m_rotateRollEventId)
        {
            m_rollKeyValue = value;
            // AZ_Printf("Object", "Grab Object roll held value %f", value);
        }
    }

    void PhysicsGrabComponent::ProcessStates(const float& deltaTime, bool isPhysicsUpdate)
    {
        // If physics update, skip full FSM transitions and only process hold/rotate on fixed timestep
        if (isPhysicsUpdate)
        {
            // Only handle hold and rotate for dynamic during physics fixed time steps
            if (m_isObjectKinematic)
            {
                return;
            }
            switch (m_state)
            {
            case PhysicsGrabStates::holdState:
                HoldObjectState(deltaTime, isPhysicsUpdate);
                break;
            case PhysicsGrabStates::rotateState:
                RotateObjectState(deltaTime, isPhysicsUpdate);
                break;
            default:
                return;
            }
            return;
        }

        switch(m_state)
        {
            case PhysicsGrabStates::idleState:
                IdleState();
                break;
            case PhysicsGrabStates::checkState:
                CheckForObjectsState();
                break;
            case PhysicsGrabStates::holdState:
                HoldObjectState(deltaTime);
                break;
            case PhysicsGrabStates::rotateState:
                RotateObjectState(deltaTime);
                break;
            case PhysicsGrabStates::throwState:
                ThrowObjectState(deltaTime);
                break;
            default:
                m_state = PhysicsGrabStates::idleState;
                IdleState();
        }

        m_prevGrabKeyValue = m_grabKeyValue;
        m_prevRotateKeyValue = m_rotateKeyValue;
        m_prevThrowKeyValue = m_throwKeyValue;
    }

    void PhysicsGrabComponent::OnTick(float deltaTime, AZ::ScriptTimePoint)
    {
        ProcessStates(deltaTime);
        if (m_meshSmoothing)
        {
            InterpolateMeshTransform(deltaTime);
        }
    }
    
    // Smoothly update the visual transform of m_meshEntityPtr based on physics transforms
    void PhysicsGrabComponent::InterpolateMeshTransform(float deltaTime)
    {
        if (!m_lastGrabbedObjectEntityId.IsValid() || !m_meshEntityPtr || m_isObjectKinematic)
        {
            return;
        }

        // Update time accumulator
        m_physicsTimeAccumulator += deltaTime;

        // Calculate interpolation factor
        const float alpha = AZ::GetClamp(m_physicsTimeAccumulator / m_physicsTimestep, 0.0f, 1.0f);

        // Interpolate position
        const AZ::Vector3 interpolatedPosition =
            m_prevPhysicsTransform.GetTranslation().Lerp(m_currentPhysicsTransform.GetTranslation(), alpha);

        // Interpolate rotation
        const AZ::Quaternion interpolatedRotation =
            m_prevPhysicsTransform.GetRotation().Slerp(m_currentPhysicsTransform.GetRotation(), alpha);

        // Interpolate uniform scale to preserve parent's scale (prevents reset to 1.0)
        const float interpolatedScale =
            AZ::Lerp(m_prevPhysicsTransform.GetUniformScale(), m_currentPhysicsTransform.GetUniformScale(), alpha);

        // Compose interpolated parent transform with original local mesh TM to preserve offsets, rotation, and scale
        AZ::Transform interpolatedParent = AZ::Transform::CreateFromQuaternionAndTranslation(interpolatedRotation, interpolatedPosition);
        
        // Set the interpolated uniform scale on the parent transform
        interpolatedParent.SetUniformScale(interpolatedScale);
        
        AZ::Transform interpolatedMesh = interpolatedParent * m_originalMeshLocalTM;
        
        // Update mesh entity transform
        AZ::TransformBus::Event(m_meshEntityPtr->GetId(), &AZ::TransformInterface::SetWorldTM, interpolatedMesh);

        // Reset accumulator if it exceeds physics timestep due to accumulated error
        if (m_physicsTimeAccumulator >= m_physicsTimestep)
            m_physicsTimeAccumulator -= m_physicsTimestep;
    }

    void PhysicsGrabComponent::ReleaseMesh()
    {
        if (m_meshEntityPtr)
        {
            AZ::TransformBus::Event(m_meshEntityPtr->GetId(), &AZ::TransformInterface::SetLocalTM, m_originalMeshLocalTM);
            m_meshEntityPtr = nullptr;
        }
    }

    void PhysicsGrabComponent::IdleState()
    {
        if ((m_forceTransition && m_targetState == PhysicsGrabStates::checkState) ||
            (!m_isStateLocked && m_prevGrabKeyValue == 0.f && m_grabKeyValue != 0.f && !m_stayInIdleState))
        {
            m_state = PhysicsGrabStates::checkState;
            m_forceTransition = false;
        }
    }

    void PhysicsGrabComponent::CheckForObjectsState()
    {
        CheckForObjects();
        // Check if sphere cast hits a valid object before transitioning to holdState.
        // Other conditionals allow forced state transition to bypass inputs with m_forceTransition, or 
        // prevent state transition with m_isStateLocked
        if ((m_forceTransition && m_targetState == PhysicsGrabStates::holdState && m_objectSphereCastHit) ||
            (!m_isStateLocked && m_objectSphereCastHit))
        {
            // Check if Grabbed Object is a Dynamic Rigid Body when first interacting with it
            m_isInitialObjectKinematic = GetGrabbedObjectKinematicElseDynamic();
            
            // Store initial collision layer
            m_prevGrabbedCollisionLayer = GetCurrentGrabbedCollisionLayer();
            
            // Set Object Current Layer variable to Temp Layer
            SetCurrentGrabbedCollisionLayer(m_tempGrabbedCollisionLayer);

            // Set Grabbed Object as Kinematic Rigid Body if set to be kinematic while holding
            if (m_kinematicWhileHeld)
            {
                SetGrabbedObjectKinematicElseDynamic(true);
                m_isObjectKinematic = true;
            }
            // Set Grabbed Object as Dynamic Rigid Body if set to be dynamic while holding
            else
            {
                SetGrabbedObjectKinematicElseDynamic(false);
                m_isObjectKinematic = false;
            }
           
            // Store object's original Linear Damping value, and set new value for hold/rotate.
            m_prevObjectLinearDamping = GetCurrentGrabbedObjectLinearDamping();
            SetCurrentGrabbedObjectLinearDamping(m_tempObjectLinearDamping);

            // Store object's original Angular Damping value, and set new value for hold/rotate.
            m_prevObjectAngularDamping = GetCurrentGrabbedObjectAngularDamping();
            SetCurrentGrabbedObjectAngularDamping(m_tempObjectAngularDamping);

            // Store and disable gravity for dynamic objects if enabled
            if (m_disableGravityWhileHeld && !m_isObjectKinematic)
            {
                Physics::RigidBodyRequestBus::EventResult(
                    m_prevGravityEnabled, m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::IsGravityEnabled);
                Physics::RigidBodyRequestBus::Event(m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::SetGravityEnabled, false);
            }

            // Store mass for dynamic objects
            if (!m_isObjectKinematic)
            {
                Physics::RigidBodyRequestBus::EventResult(
                    m_grabbedObjectMass, m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::GetMass);
            }

            // Initialize physics transforms for dynamic objects
            if (!m_isObjectKinematic && m_meshSmoothing)
            {
                AZ::TransformBus::EventResult(m_prevPhysicsTransform, m_lastGrabbedObjectEntityId, &AZ::TransformInterface::GetWorldTM);
                m_currentPhysicsTransform = m_prevPhysicsTransform;
                m_physicsTimeAccumulator = 0.0f;
            }

            // Compute local grab offset from initial hit position
            AZ::Transform objectTM = AZ::Transform::CreateIdentity();
            AZ::TransformBus::EventResult(objectTM, m_lastGrabbedObjectEntityId, &AZ::TransformInterface::GetWorldTM);

            // Get inertia tensor from rigid body (local space) for Tidal Lock scaling
            AZ::Matrix3x3 grabbedObjectInertiaTensor = AZ::Matrix3x3::CreateIdentity();
            Physics::RigidBodyRequestBus::EventResult(
                grabbedObjectInertiaTensor, m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::GetInertiaLocal);

            // Compute average inertia from diagonal elements as scalar approximation
            float averageGrabbedObjectInertia = (grabbedObjectInertiaTensor.GetElement(0, 0) + 
                grabbedObjectInertiaTensor.GetElement(1, 1) +
                grabbedObjectInertiaTensor.GetElement(2, 2)) / 3.0f;

            // If average inertia is valid (non-zero, assuming compute inertia enabled), compute factor (~ s^2)
            if (averageGrabbedObjectInertia > AZ::Constants::FloatEpsilon)
            {
                m_effectiveInertiaFactor = averageGrabbedObjectInertia / m_grabbedObjectMass;
            }
            else
            {
                // If inertia unavailable, default to no scaling
                m_effectiveInertiaFactor = 1.0f;
            }

            // Clamp to minimum to prevent very small values
            if (m_effectiveInertiaFactor < 0.01f)
            {
                m_effectiveInertiaFactor = 0.01f;
            }

            m_localGrabOffset = objectTM.GetInverse().TransformPoint(m_hitPosition);

            // Compute dynamic initial grab distance as projected distance along forward to effective point
            AZ::Vector3 initialEffectivePoint = m_offsetGrab ? m_hitPosition : objectTM.GetTranslation();
            float projectedGrabDistance = (initialEffectivePoint - m_grabbingEntityTransform.GetTranslation()).Dot(m_forwardVector);
            m_grabDistance = AZ::GetClamp(projectedGrabDistance, m_minGrabDistance, m_maxGrabDistance);

            m_meshEntityPtr = nullptr;
            if (m_meshSmoothing && !m_isObjectKinematic)
            {
                // Prioritize m_meshEntityId if valid
                if (m_meshEntityId.IsValid())
                {
                    m_meshEntityPtr = GetEntityPtr(m_meshEntityId);
                }
                // Fallback to tag-based lookup if m_meshEntityId is invalid and m_meshTagName is set
                else if (!m_meshTagName.empty())
                {
                    AZ::Entity* grabbedEntity = GetEntityPtr(m_lastGrabbedObjectEntityId);
                    if (grabbedEntity)
                    {
                        AZStd::vector<AZ::EntityId> children;
                        AZ::TransformBus::EventResult(children, m_lastGrabbedObjectEntityId, &AZ::TransformInterface::GetChildren);
                        AZ::Crc32 meshTag = AZ::Crc32(m_meshTagName.c_str());
                        for (const AZ::EntityId& childId : children)
                        {
                            bool hasTag = false;
                            LmbrCentral::TagComponentRequestBus::EventResult(
                                hasTag, childId, &LmbrCentral::TagComponentRequests::HasTag, meshTag);
                            if (hasTag)
                            {
                                m_meshEntityPtr = GetEntityPtr(childId);
                                break;
                            }
                        }
                    }
                }
            }
            // Capture original local transform of mesh entity for preservation during smoothing
            if (m_meshEntityPtr)
            {
                AZ::TransformBus::EventResult(m_originalMeshLocalTM, m_meshEntityPtr->GetId(), &AZ::TransformInterface::GetLocalTM);
            }
            // Warn if no mesh found but smoothing expected
            else
            {
                AZ_Warning(
                    "PhysicsGrabComponent",
                    false,
                    "Mesh smoothing enabled but no tagged child entity found for %s. Skipping interpolation to avoid physics desync.",
                    GetEntityPtr(m_lastGrabbedObjectEntityId)->GetName().c_str());
            }

            // Reset m_prevGrabbingEntityTranslation and m_currentGrabEntityTranslation to the current 
            // position at the exact moment of transition to holdState, ensuring the first physics velocity is ~zero.
            // Prevents initial object flinging off screen due to large m_grabbingEntityVelocity
            if (m_velocityCompensation)
            {
                // Reset compensation and angular velocity for new grab (ensures gradual start from zero)
                m_currentCompensationVelocity = AZ::Vector3::CreateZero();
                m_currentAngularVelocity = AZ::Vector3::CreateZero();
                if (m_useFPControllerForGrab)
                {
                    m_prevGrabbingEntityTranslation = m_currentGrabEntityTranslation =
                        GetEntityPtr(GetEntityId())->GetTransform()->GetWorldTM().GetTranslation();
                }
                else
                {
                    m_prevGrabbingEntityTranslation = m_currentGrabEntityTranslation =
                        m_grabbingEntityPtr->GetTransform()->GetWorldTM().GetTranslation();
                }
            }

            // Reset PID controller on new grab to clear error history
            if (m_enablePIDHeldDynamics)
            {
                m_pidController.Reset();
            }

            // Reset angular PID and compute initial relative quaternion for tidal lock
            m_tidalLockPidController.Reset();

            AZ::Quaternion grabbingEntityRotationQuat;
            #ifdef FIRST_PERSON_CONTROLLER
            if (m_useFPControllerForGrab)
            {
                if (m_fullTidalLockForFPC)
                {
                    // Use camera rotation for full lock
                    FirstPersonController::FirstPersonControllerComponentRequestBus::EventResult(
                        m_cameraRotationTransform,
                        GetEntityId(),
                        &FirstPersonController::FirstPersonControllerComponentRequests::GetCameraRotationTransform);
                    grabbingEntityRotationQuat = m_cameraRotationTransform->GetWorldRotationQuaternion();
                }
                else
                {
                    grabbingEntityRotationQuat = GetEntity()->GetTransform()->GetWorldRotationQuaternion();
                }
            }
            else
            #endif
            {
                grabbingEntityRotationQuat = m_grabbingEntityPtr->GetTransform()->GetWorldRotationQuaternion();
            }
            AZ::Quaternion grabbedObjectRotationQuat;
            AZ::TransformBus::EventResult(
                grabbedObjectRotationQuat, m_lastGrabbedObjectEntityId, &AZ::TransformInterface::GetWorldRotationQuaternion);
            m_grabbedObjectRelativeQuat = grabbingEntityRotationQuat.GetInverseFull() * grabbedObjectRotationQuat;

            m_state = PhysicsGrabStates::holdState;
            // Broadcast a grab start notification event
            PhysicsGrabNotificationBus::Broadcast(&PhysicsGrabNotificationBus::Events::OnHoldStart);

            #ifdef FIRST_PERSON_CONTROLLER
            if (m_useFPControllerForGrab)
            {
                m_lastEntityRotationQuat = GetEntity()->GetTransform()->GetWorldRotationQuaternion();
            }
            else
            #endif
            {
                m_lastEntityRotationQuat = m_grabbingEntityPtr->GetTransform()->GetWorldRotationQuaternion();
            }

            // Set angular velocity to 0 when first picking up a dynamic rigid body object
            if (!m_kinematicWhileHeld && m_initialAngularVelocityZero)
            {
                SetGrabbedObjectAngularVelocity(AZ::Vector3::CreateZero());
            }
            m_forceTransition = false;
        }
        // Go back to idleState if grab key is not pressed
        // Other conditionals allow forced state transition to bypass inputs with m_forceTransition, or 
        // prevent state transition with m_isStateLocked
        else if (
            (m_forceTransition && m_targetState == PhysicsGrabStates::idleState) ||
            (!m_isStateLocked && !(m_prevGrabKeyValue == 0.f && m_grabKeyValue != 0.f)))
        {
            m_state = PhysicsGrabStates::idleState;
            m_forceTransition = false;
        }
        else
        {
            m_state = PhysicsGrabStates::checkState;
        }
    }

    void PhysicsGrabComponent::HoldObjectState(float deltaTime, bool isPhysicsUpdate)
    {
        if (!m_grabMaintained)
        {
            CheckForObjects();
        }

        if (isPhysicsUpdate)
        {
            // Compensate for potential velocity change from grab entity
            ComputeGrabbingEntityVelocity(deltaTime);

            // Update dynamic objects on physics fixed time step
            HoldObject(deltaTime);
            return;
        }

        // Drop the object and go back to idle state if sphere cast doesn't hit
        // Other conditionals allow forced state transition to bypass inputs with m_forceTransition, or 
        // prevent state transition with m_isStateLocked
        if ((m_forceTransition && m_targetState == PhysicsGrabStates::idleState) || 
            (!m_isStateLocked && !m_objectSphereCastHit))
        {
            ReleaseGrabbedObject(true, false);
            m_state = PhysicsGrabStates::idleState;
            m_forceTransition = false;
            return;
        }

        // Update grab distance every frame (non-physics) for reliable input capture
        UpdateGrabDistance(deltaTime);

        if (m_isObjectKinematic)
        {
            HoldObject(deltaTime);
        }

        // Go back to idle state if grab key is pressed again because we want to stop holding the 
        // object on the second key press. Other conditionals allow forced state transition to 
        // bypass inputs with m_forceTransition, or prevent state transition with m_isStateLocked
        if ((m_forceTransition && m_targetState == PhysicsGrabStates::idleState) ||
            (!m_isStateLocked &&
             ((m_grabEnableToggle && m_prevGrabKeyValue == 0.f && m_grabKeyValue != 0.f) ||
              (!m_grabEnableToggle && m_grabKeyValue == 0.f))))
        {
            ReleaseGrabbedObject(true, false);
            m_state = PhysicsGrabStates::idleState;
            m_forceTransition = false;
        }
        // Enter Rotate State if rotate key is pressed. Other conditionals allow forced state transition 
        // to bypass inputs with m_forceTransition, or prevent state transition with m_isStateLocked
        else if (
            (m_forceTransition && m_targetState == PhysicsGrabStates::rotateState) ||
            (!m_isStateLocked &&
             ((m_rotateEnableToggle && m_prevRotateKeyValue == 0.f && m_rotateKeyValue != 0.f) ||
              (!m_rotateEnableToggle && m_rotateKeyValue != 0.f))))
        {
            m_state = PhysicsGrabStates::rotateState;
            PhysicsGrabNotificationBus::Broadcast(&PhysicsGrabNotificationBus::Events::OnRotateStart);
            m_forceTransition = false;
        }
        // Transition to throwState if Throw key is pressed. Handles throw press, charging, and release.
        // Other conditionals allow forced state transition to bypass inputs with m_forceTransition, or
        // prevent state transition with m_isStateLocked
        else if (!m_isStateLocked && m_prevThrowKeyValue == 0.f && m_throwKeyValue != 0.f && !m_isInitialObjectKinematic)
        {
            // If chargeable throw is disabled, perform immediate throw
            if (!m_enableChargeThrow)
            {
                TransitionToThrow(false);
            }
            else
            {
                // Start charging for throw
                m_isChargingThrow = true;
                m_currentChargeTime = 0.f;
                m_hasNotifiedChargeComplete = false;
            }
        }
        // Accumulate charge time only while the throw key is held (and not in physics update for input responsiveness)
        else if (m_isChargingThrow && m_throwKeyValue != 0.f && !isPhysicsUpdate)
        {
            // Increment the charge timer
            m_currentChargeTime += deltaTime;
            // Check if full charge is reached and notify if not already done
            if (!m_hasNotifiedChargeComplete && m_currentChargeTime >= m_chargeTime)
            {
                PhysicsGrabNotificationBus::Broadcast(&PhysicsGrabNotificationBus::Events::OnChargeComplete);
                m_hasNotifiedChargeComplete = true;
            }
        }
        // Detect throw key release to trigger the charged throw
        else if (!m_isStateLocked && m_prevThrowKeyValue != 0.f && m_throwKeyValue == 0.f && !m_isInitialObjectKinematic)
        {
            // If chargeable throw is enabled and charging was active, perform charged throw
            if (m_enableChargeThrow && m_isChargingThrow)
            {
                TransitionToThrow(true);
            }
        }
        else
        {
            m_state = PhysicsGrabStates::holdState;
        }
    }

    void PhysicsGrabComponent::RotateObjectState(float deltaTime, bool isPhysicsUpdate)
    {
        if (!m_grabMaintained)
        {
            CheckForObjects();
        }

        if (isPhysicsUpdate)
        {
            // Compensate for potential velocity change from grab entity
            ComputeGrabbingEntityVelocity(deltaTime);

            // Update dynamic objects on physics fixed time step
            HoldObject(deltaTime);
            RotateObject(deltaTime);
            return;
        }

        // Drop the object and go back to idle state if sphere cast doesn't hit. Other 
        // conditionals allow forced state transition to bypass inputs with 
        // m_forceTransition, or prevent state transition with m_isStateLocked
        if ((m_forceTransition && m_targetState == PhysicsGrabStates::idleState) || 
            (!m_isStateLocked && !m_objectSphereCastHit))
        {
            // Set Angular Velocity back to zero if sphere cast doesn't hit
            SetGrabbedObjectAngularVelocity(AZ::Vector3::CreateZero());
            ReleaseGrabbedObject(true, true);
            m_state = PhysicsGrabStates::idleState;
            m_forceTransition = false;
            return;
        }

        // Update grab distance every frame (non-physics) for reliable input capture
        UpdateGrabDistance(deltaTime);

        // Accumulate mouse deltas every frame for dynamic (to capture all between physics ticks)
        m_accumPitch += m_pitchKeyValue;
        m_accumYaw += m_yawKeyValue;
        m_accumRoll += m_rollKeyValue;

        if (m_isObjectKinematic)
        {
            HoldObject(deltaTime);
            #ifdef FIRST_PERSON_CONTROLLER
            FreezeCharacterRotation();
            #endif
            RotateObject(deltaTime);
        }
        else
        {
            #ifdef FIRST_PERSON_CONTROLLER
            FreezeCharacterRotation();
            #endif
        }

        // Go back to idle state if grab key is pressed again because we want to stop holding the
        // object on the second key press. Other conditionals allow forced state transition to 
        // bypass inputs with m_forceTransition, or prevent state transition with m_isStateLocked
        if ((m_forceTransition && m_targetState == PhysicsGrabStates::idleState) ||
            (!m_isStateLocked &&
             ((m_grabEnableToggle && m_prevGrabKeyValue == 0.f && m_grabKeyValue != 0.f) ||
              (!m_grabEnableToggle && m_prevGrabKeyValue == 0.f))))
        {
            // Set Angular Velocity back to zero if Grab Key is not pressed
            SetGrabbedObjectAngularVelocity(AZ::Vector3::CreateZero());
            ReleaseGrabbedObject(true, true);
            m_state = PhysicsGrabStates::idleState;
            m_forceTransition = false;
        }
        // Go back to hold state if rotate key is pressed again because we want to stop rotating the 
        // object on the second key press. Other conditionals allow forced state transition to 
        // bypass inputs with m_forceTransition, or prevent state transition with m_isStateLocked
        else if (
            (m_forceTransition && m_targetState == PhysicsGrabStates::holdState) ||
            (!m_isStateLocked &&
             ((m_rotateEnableToggle && m_prevRotateKeyValue == 0.f && m_rotateKeyValue != 0.f) ||
              (!m_rotateEnableToggle && m_rotateKeyValue == 0.f))))
        {
            SetGrabbedObjectAngularVelocity(AZ::Vector3::CreateZero());

            // Recompute relative quaternion after rotation changes (to lock new orientation)
            AZ::Quaternion grabbingEntityRotationQuat;
            #ifdef FIRST_PERSON_CONTROLLER
            if (m_useFPControllerForGrab)
            {
                if (m_fullTidalLockForFPC)
                {
                    // Use camera rotation for full lock
                    FirstPersonController::FirstPersonControllerComponentRequestBus::EventResult(
                        m_cameraRotationTransform,
                        GetEntityId(),
                        &FirstPersonController::FirstPersonControllerComponentRequests::GetCameraRotationTransform);
                    grabbingEntityRotationQuat = m_cameraRotationTransform->GetWorldRotationQuaternion();
                }
                else
                {
                    grabbingEntityRotationQuat = GetEntity()->GetTransform()->GetWorldRotationQuaternion();
                }
            }
            else
            #endif
            {
                grabbingEntityRotationQuat = m_grabbingEntityPtr->GetTransform()->GetWorldRotationQuaternion();
            }

            AZ::Quaternion grabbedObjectRotationQuat;
            AZ::TransformBus::EventResult(
                grabbedObjectRotationQuat, m_lastGrabbedObjectEntityId, &AZ::TransformInterface::GetWorldRotationQuaternion);
            m_grabbedObjectRelativeQuat = grabbingEntityRotationQuat.GetInverseFull() * grabbedObjectRotationQuat;

            m_tidalLockPidController.Reset();

            m_state = PhysicsGrabStates::holdState;
            PhysicsGrabNotificationBus::Broadcast(&PhysicsGrabNotificationBus::Events::OnRotateStop);
            m_forceTransition = false;
        }
        // Transition to throwState if Throw key is pressed. Handles throw press, charging, and release. 
        // Other conditionals allow forced state transition to bypass inputs with m_forceTransition, or 
        // prevent state transition with m_isStateLocked
        else if (!m_isStateLocked && m_prevThrowKeyValue == 0.f && m_throwKeyValue != 0.f && !m_isInitialObjectKinematic)
        {
            // If chargeable throw is disabled or not allowed while rotating, perform immediate throw
            if (!m_enableChargeThrow || !m_enableChargeWhileRotating)
            {
                TransitionToThrow(false);
            }
            else
            {
                // Start charging for throw
                m_isChargingThrow = true;
                m_currentChargeTime = 0.f;
                m_hasNotifiedChargeComplete = false;
            }
        }
        // Accumulate charge time only while the throw key is held (and not in physics update for input responsiveness)
        else if (m_isChargingThrow && m_throwKeyValue != 0.f && !isPhysicsUpdate)
        {
            // Increment the charge timer
            m_currentChargeTime += deltaTime;
            // Check if full charge is reached and notify if not already done
            if (!m_hasNotifiedChargeComplete && m_currentChargeTime >= m_chargeTime)
            {
                PhysicsGrabNotificationBus::Broadcast(&PhysicsGrabNotificationBus::Events::OnChargeComplete);
                m_hasNotifiedChargeComplete = true;
            }
        }
        // Detect throw key release to trigger the charged throw
        else if (!m_isStateLocked && m_prevThrowKeyValue != 0.f && m_throwKeyValue == 0.f && !m_isInitialObjectKinematic)
        {
            // If chargeable throw is enabled and charging was active, perform charged throw
            if (m_enableChargeThrow && m_isChargingThrow)
            {
                TransitionToThrow(true);
            }
        }
        else
        {
            m_state = PhysicsGrabStates::rotateState;
        }
    }

    void PhysicsGrabComponent::ThrowObjectState(const float &deltaTime)
    {
        // ThrowObject() is only executed once. If setting m_throwStateCounter value via ebus, it 
        // is recommended to assign a value equal to m_throwStateMaxTime in order to properly execute ThrowObject()
        if (m_throwStateCounter == m_throwStateMaxTime)
        {
            ThrowObject();
            // Reset entity IDs after throwing to allow grabbing new objects
            m_grabbedObjectEntityId = AZ::EntityId();
            m_lastGrabbedObjectEntityId = AZ::EntityId();
        }

        m_throwStateCounter -= deltaTime;

        // Escape from the throw state if the thrown grabbed object is more than the distance of m_sphereCastDistance away
        if (m_grabReference.GetTranslation().GetDistance(
                GetEntityPtr(m_thrownGrabbedObjectEntityId)->GetTransform()->GetWorldTM().GetTranslation()) > m_sphereCastDistance)
        {
            PhysicsGrabNotificationBus::Broadcast(&PhysicsGrabNotificationBus::Events::OnThrowStop);
            PhysicsGrabNotificationBus::Broadcast(&PhysicsGrabNotificationBus::Events::OnMaxThrowDistance);
            m_state = PhysicsGrabStates::idleState;
        }
        // Escape from the throw state if grabbed object is in throw state longer than m_throwStateMaxTime
        else if (m_throwStateCounter <= 0.f)
        {
            PhysicsGrabNotificationBus::Broadcast(&PhysicsGrabNotificationBus::Events::OnThrowStop);
            PhysicsGrabNotificationBus::Broadcast(&PhysicsGrabNotificationBus::Events::OnThrowStateCounterZero);
            m_state = PhysicsGrabStates::idleState;
        }
        else
        {
            m_state = PhysicsGrabStates::throwState;
        }
    }

    // Perform a spherecast query to check if colliding with a grabbable object, then assign the 
    // first returned hit to m_grabbedObjectEntityId
    void PhysicsGrabComponent::CheckForObjects()
    {
        #ifdef FIRST_PERSON_CONTROLLER
        if (m_useFPControllerForGrab)
        {
            FirstPersonController::FirstPersonControllerComponentRequestBus::EventResult(
                m_cameraRotationTransform,
                GetEntityId(),
                &FirstPersonController::FirstPersonControllerComponentRequests::GetCameraRotationTransform);
            m_grabbingEntityTransform = m_cameraRotationTransform->GetWorldTM();
            m_forwardVector = m_cameraRotationTransform->GetWorldTM().GetBasisY();
        }
        else
        #endif
        {
            // Get forward vector relative to the grabbing entity's transform
            m_forwardVector = m_grabbingEntityPtr->GetTransform()->GetWorldTM().GetBasisY();

            // Get our grabbing entity's world transform
            m_grabbingEntityTransform = m_grabbingEntityPtr->GetTransform()->GetWorldTM();
        }

        // Perform a spherecast query to check if colliding with object
        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();

        AzPhysics::ShapeCastRequest request = AzPhysics::ShapeCastRequestHelpers::CreateSphereCastRequest(
            m_sphereCastRadius,
            m_grabbingEntityTransform,
            m_forwardVector,
            m_sphereCastDistance,
            AzPhysics::SceneQuery::QueryType::Dynamic,
            m_grabbedCollisionGroup,
            nullptr);
        request.m_reportMultipleHits = true;

        AzPhysics::SceneHandle sceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        AzPhysics::SceneQueryHits hits = sceneInterface->QueryScene(sceneHandle, &request);

        // Filter out hits from the grabbing entity and its children
        AZStd::vector<AZ::EntityId> children;
        AZ::TransformBus::EventResult(children, m_grabbingEntityPtr->GetId(), &AZ::TransformInterface::GetChildren);

        auto filterSelfAndChildren = [this, &children](AzPhysics::SceneQueryHit& hit)
        {
            if (hit.m_entityId == m_grabbingEntityPtr->GetId())
                return true;
            for (const AZ::EntityId& childId : children)
            {
                if (hit.m_entityId == childId)
                    return true;
            }
            return false;
        };
        AZStd::erase_if(hits.m_hits, filterSelfAndChildren);

        m_objectSphereCastHit = false;

        // Prioritize hit matching m_lastGrabbedObjectEntityId if grabbing (maintain initial hit position/offset)
        if (m_lastGrabbedObjectEntityId.IsValid())
        {
            for (const AzPhysics::SceneQueryHit& hit : hits.m_hits)
            {
                if (hit.m_entityId == m_lastGrabbedObjectEntityId)
                {
                    m_objectSphereCastHit = true;
                    m_grabbedObjectEntityId = hit.m_entityId;
                    break;
                }
            }
        }
        else if (hits)
        {
            // Take the first hit for initial grab and store hit position
            m_objectSphereCastHit = true;
            m_grabbedObjectEntityId = hits.m_hits.at(0).m_entityId;
            m_lastGrabbedObjectEntityId = m_grabbedObjectEntityId;
            m_hitPosition = hits.m_hits.at(0).m_position;
        }
    }

    // Hold and move object using physics or translation, based on object's 
    // starting Rigid Body type, or if KinematicWhileHeld is enabled
    void PhysicsGrabComponent::HoldObject(float deltaTime)
    {        
        // Use FPC Entity directly for Grab Reference for tighter tracking and avoid camera lerp lag
        #ifdef FIRST_PERSON_CONTROLLER
        if (m_useFPControllerForGrab)
        {
            AZ::Vector3 characterPosition;
            AZ::TransformBus::EventResult(characterPosition, GetEntityId(), &AZ::TransformBus::Events::GetWorldTranslation);
            float eyeHeight = 0.f;
            float cameraLocalZTravelDistance = 0.f;
            FirstPersonController::FirstPersonControllerComponentRequestBus::EventResult(
                m_cameraRotationTransform,
                GetEntityId(),
                &FirstPersonController::FirstPersonControllerComponentRequests::GetCameraRotationTransform);
            m_forwardVector = m_cameraRotationTransform->GetWorldTM().GetBasisY();
            FirstPersonController::FirstPersonControllerComponentRequestBus::EventResult(
                eyeHeight, GetEntityId(), &FirstPersonController::FirstPersonControllerComponentRequests::GetEyeHeight);
            FirstPersonController::FirstPersonControllerComponentRequestBus::EventResult(
                cameraLocalZTravelDistance,
                GetEntityId(),
                &FirstPersonController::FirstPersonControllerComponentRequests::GetCameraLocalZTravelDistance);

            AZ::Transform characterTM;
            AZ::TransformBus::EventResult(characterTM, GetEntityId(), &AZ::TransformInterface::GetWorldTM);
            m_grabReference = characterTM;
            m_grabReference.SetTranslation(characterPosition + AZ::Vector3(0.f, 0.f, eyeHeight + cameraLocalZTravelDistance));
            // Add forward offset to the height-adjusted position
            m_grabReference.SetTranslation(m_grabReference.GetTranslation() + m_forwardVector * m_grabDistance);
        }
        // Use user-specified grab entity for Grab Reference
        else
        #endif
        {
            // Get forward vector relative to the grabbing entity's transform
            m_forwardVector = m_grabbingEntityPtr->GetTransform()->GetWorldTM().GetBasisY();
            // Creates a reference point for the Grabbed Object translation in front of the Grabbing Entity
            m_grabReference = m_grabbingEntityPtr->GetTransform()->GetWorldTM();
            m_grabReference.SetTranslation(
                m_grabbingEntityPtr->GetTransform()->GetWorldTM().GetTranslation() + m_forwardVector * m_grabDistance);
        }

        // Get object transform once (center of mass transform)
        AZ::Transform objectTM = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(objectTM, m_lastGrabbedObjectEntityId, &AZ::TransformInterface::GetWorldTM);
        
        // Center of mass translation
        m_grabbedObjectTranslation = objectTM.GetTranslation();

        // Determine effective point and error based on toggle
        AZ::Vector3 effectivePoint = m_grabbedObjectTranslation;
        AZ::Vector3 positionError = m_grabReference.GetTranslation() - m_grabbedObjectTranslation;
        if (m_offsetGrab)
        {
            effectivePoint = objectTM.TransformPoint(m_localGrabOffset);
            positionError = m_grabReference.GetTranslation() - effectivePoint;
        }

        // Move the object using Translation (Transform) if it is a Kinematic Rigid Body
        if (m_isObjectKinematic)
        {
            // Move object by setting its Translation
            AZ::TransformBus::Event(
                m_lastGrabbedObjectEntityId, &AZ::TransformInterface::SetWorldTranslation, m_grabReference.GetTranslation());
            
            // If object is NOT in rotate state, couple the grabbed entity's rotation to 
            // the controlling entity's local z rotation (causing object to face controlling entity)
            if (m_state != PhysicsGrabStates::rotateState && m_kinematicTidalLock && (m_tidalLock || m_fullTidalLockForFPC))
            {
                TidalLock(deltaTime);
            }
            // Update mesh entity transform if smoothing is disabled
            if (!m_meshSmoothing && m_meshEntityPtr && m_meshEntityPtr != GetEntityPtr(m_lastGrabbedObjectEntityId))
            {
                AZ::TransformBus::Event(
                    m_meshEntityPtr->GetId(),
                    &AZ::TransformInterface::SetWorldTM,
                    GetEntityPtr(m_lastGrabbedObjectEntityId)->GetTransform()->GetWorldTM());
            }
        }

        // Move the object using PhysX if it is a Dynamic Rigid Body
        else
        {
            AZ::Vector3 targetLinearVelocity;
            if (m_enablePIDHeldDynamics)
            {
                // Use PID to compute spring-like velocity adjustment
                targetLinearVelocity = m_pidController.Output(positionError, deltaTime, effectivePoint);
            }
            else
            {
                // Fallback to original simple proportional control
                targetLinearVelocity = positionError * m_grabResponse;
            }

            if (m_enablePIDHeldDynamics)
            {
                AZ::Vector3 linearPidOutput = targetLinearVelocity;

                // Add feed-forward for target velocity only in Velocity mode (ErrorRate handles it natively)
                if (m_velocityCompensation && m_pidController.GetMode() == PidController<AZ::Vector3>::Velocity)
                {
                    float effectiveFactor = 1.0f - exp(-m_velocityCompDampRate * deltaTime);
                    m_currentCompensationVelocity = m_currentCompensationVelocity.Lerp(m_grabbingEntityVelocity, effectiveFactor);
                    linearPidOutput += m_heldDerivativeGain * m_currentCompensationVelocity;
                }

                // Treat PID output as force; optionally scale by mass for mass-independent behavior
                AZ::Vector3 linearForce = m_massIndependentHeldPID ? m_grabbedObjectMass * linearPidOutput : linearPidOutput;

                // Apply as impulse
                AZ::Vector3 linearImpulse = linearForce * deltaTime;
                if (m_offsetGrab && (m_gravityAppliesToPointRotation || m_state != PhysicsGrabStates::rotateState))
                {
                    Physics::RigidBodyRequestBus::Event(
                        m_lastGrabbedObjectEntityId,
                        &Physics::RigidBodyRequests::ApplyLinearImpulseAtWorldPoint,
                        linearImpulse,
                        effectivePoint);
                }
                else
                {
                    Physics::RigidBodyRequestBus::Event(
                        m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::ApplyLinearImpulse, linearImpulse);
                }
            }
            else
            {
                // Compute grabbing entity velocity compensation
                AZ::Vector3 compensation = AZ::Vector3::CreateZero();
                // Add feed-forward for target velocity only in Velocity mode (ErrorRate handles it natively)
                if (m_velocityCompensation)
                {
                    float effectiveFactor = 1.0f - exp(-m_velocityCompDampRate * deltaTime);
                    m_currentCompensationVelocity = m_currentCompensationVelocity.Lerp(m_grabbingEntityVelocity, effectiveFactor);
                    compensation = m_currentCompensationVelocity;
                }

                // Simple velocity-based application
                Physics::RigidBodyRequestBus::Event(
                    m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::SetLinearVelocity, targetLinearVelocity + compensation);
            }

            // If object is NOT in rotate state, couple the grabbed entity's rotation to
            // the controlling entity's local z rotation
            if (m_state != PhysicsGrabStates::rotateState && m_dynamicTidalLock && (m_tidalLock || m_fullTidalLockForFPC))
            {
                TidalLock(deltaTime);
            }

            // Update current physics transform for interpolation
            if (m_meshSmoothing)
            {
                AZ::TransformBus::EventResult(m_currentPhysicsTransform, m_lastGrabbedObjectEntityId, &AZ::TransformInterface::GetWorldTM);
            }
            // Update mesh entity transform if smoothing is disabled
            else if (m_meshEntityPtr && m_meshEntityPtr != GetEntityPtr(m_lastGrabbedObjectEntityId))
            {
                AZ::TransformBus::Event(
                    m_meshEntityPtr->GetId(),
                    &AZ::TransformInterface::SetWorldTM,
                    GetEntityPtr(m_lastGrabbedObjectEntityId)->GetTransform()->GetWorldTM());
            }
        }
    }

    // Rotate object using physics or transforms, based on object's starting 
    // Rigid Body type, or if KinematicWhileHeld is enabled.
    void PhysicsGrabComponent::RotateObject(float deltaTime)
    {
        // Use FPC Entity directly for Grab Reference for tighter tracking and avoid camera lerp lag
        #ifdef FIRST_PERSON_CONTROLLER
        if (m_useFPControllerForGrab)
        {
            FirstPersonController::FirstPersonControllerComponentRequestBus::EventResult(
                m_cameraRotationTransform,
                GetEntityId(),
                &FirstPersonController::FirstPersonControllerComponentRequests::GetCameraRotationTransform);
            m_rightVector = m_cameraRotationTransform->GetWorldTM().GetBasisX();
            m_upVector = m_cameraRotationTransform->GetWorldTM().GetBasisZ();
        }
        else
        #endif
        {
            // Get right vector relative to the grabbing entity's transform
            m_rightVector = m_grabbingEntityPtr->GetTransform()->GetWorldTM().GetBasisX();

            // Get up vector relative to the grabbing entity's transform
            m_upVector = m_grabbingEntityPtr->GetTransform()->GetWorldTM().GetBasisZ();
        }
        // Pitch value depends on whether pitch input key is ignored via SetPitchKeyValue()
        const float pitchValue = m_ignorePitchKeyInputValue ? m_pitchKeyValue : m_pitch;
        // Yaw value depends on whether yaw input key is ignored via SetYawKeyValue()
        const float yawValue = m_ignoreYawKeyInputValue ? m_yawKeyValue : m_yaw;
        // Roll value depends on whether roll input key is ignored via SetRollKeyValue()
        const float rollValue = m_ignoreRollKeyInputValue ? m_rollKeyValue : m_roll;

        // Rotate the object using SetRotation (Transform) if it is a Kinematic Rigid Body
        if (m_isObjectKinematic)
        {
            AZ::Quaternion rotation =
                AZ::Quaternion::CreateFromAxisAngle(m_upVector, yawValue * m_kinematicYawRotateScale * 0.01f) +
                AZ::Quaternion::CreateFromAxisAngle(m_rightVector, pitchValue * m_kinematicPitchRotateScale * 0.01f) +
                AZ::Quaternion::CreateFromAxisAngle(m_forwardVector, rollValue * m_kinematicRollRotateScale * 0.01f);

            AZ::Transform transform = AZ::Transform::CreateIdentity();
            AZ::TransformBus::EventResult(transform, m_lastGrabbedObjectEntityId, &AZ::TransformInterface::GetWorldTM);

            transform.SetRotation((rotation * transform.GetRotation()).GetNormalized());

            AZ::TransformBus::Event(m_lastGrabbedObjectEntityId, &AZ::TransformInterface::SetWorldTM, transform);

            // Update mesh entity transform if smoothing is disabled
            if (!m_meshSmoothing && m_meshEntityPtr && m_meshEntityPtr != GetEntityPtr(m_lastGrabbedObjectEntityId))
            {
                AZ::TransformBus::Event(m_meshEntityPtr->GetId(), &AZ::TransformInterface::SetWorldTM, transform);
            }
        }
        // Rotate the object using SetAngularVelocity (PhysX) if it is a Dynamic Rigid Body
        else
        {
            // Normalize accumulators by deltaTime to get consistent rotation speed regardless of physics timestep
            float pitchSpeed = (deltaTime > 0.0f) ? m_accumPitch / deltaTime : 0.0f;
            float yawSpeed = (deltaTime > 0.0f) ? m_accumYaw / deltaTime : 0.0f;
            float rollSpeed = (deltaTime > 0.0f) ? m_accumRoll / deltaTime : 0.0f;

            AZ::Vector3 targetAngularVelocity = (m_rightVector * pitchSpeed * m_dynamicPitchRotateScale * 0.01) +
                (m_forwardVector * rollSpeed * m_dynamicRollRotateScale * 0.01) +
                (m_upVector * yawSpeed * m_dynamicYawRotateScale) * 0.01;

            // Lerp toward target rotation for gradual damping
            if (m_smoothDynamicRotation)
            {
                float effective_factor = 1.0f - exp(-m_angularVelocityDampRate * deltaTime);
                m_currentAngularVelocity = m_currentAngularVelocity.Lerp(targetAngularVelocity, effective_factor);
            }
            // Set angular velocity directly with no smooth damping
            else
            {
                m_currentAngularVelocity = targetAngularVelocity;
            }

            SetGrabbedObjectAngularVelocity(m_currentAngularVelocity);

            // Reset accumulators after applying in physics branch
            m_accumPitch = 0.0f;
            m_accumYaw = 0.0f;
            m_accumRoll = 0.0f;

            // Update current physics transform for interpolation
            if (m_meshSmoothing)
            {
                AZ::TransformBus::EventResult(m_currentPhysicsTransform, m_lastGrabbedObjectEntityId, &AZ::TransformInterface::GetWorldTM);
            }
            // Update mesh entity transform if smoothing is disabled
            else if (m_meshEntityPtr && m_meshEntityPtr != GetEntityPtr(m_lastGrabbedObjectEntityId))
            {
                AZ::TransformBus::Event(
                    m_meshEntityPtr->GetId(),
                    &AZ::TransformInterface::SetWorldTM,
                    GetEntityPtr(m_lastGrabbedObjectEntityId)->GetTransform()->GetWorldTM());
            }
        }
    }

    // Apply linear impulse to object if it is a Dynamic Rigid Body
    void PhysicsGrabComponent::ThrowObject()
    {
        // Query mass for potential scaling (default to 1 if fails)
        float mass = 1.0f;
        Physics::RigidBodyRequestBus::EventResult(mass, m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::GetMass);

        // Compute base impulse
        AZ::Vector3 base_impulse = m_forwardVector * m_currentThrowImpulse;

        // Optionally scale by mass for mass-independent throw velocity
        AZ::Vector3 impulse = m_massIndependentThrow ? mass * base_impulse : base_impulse;

        // Apply a Linear Impulse to the grabbed object
        Physics::RigidBodyRequestBus::Event(
            m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequestBus::Events::ApplyLinearImpulse, impulse);

        // Trigger an event notification when object enters Throw State
        PhysicsGrabNotificationBus::Broadcast(&PhysicsGrabNotificationBus::Events::OnThrowStart);

        m_thrownGrabbedObjectEntityId = m_lastGrabbedObjectEntityId;

        // Set Object Current Layer variable back to Prev Layer when thrown
        SetCurrentGrabbedCollisionLayer(m_prevGrabbedCollisionLayer);

        // Set Angular Damping back to original value
        SetCurrentGrabbedObjectAngularDamping(m_prevObjectAngularDamping);

        // Restore gravity if it was disabled
        if (m_disableGravityWhileHeld && !m_isObjectKinematic)
        {
            Physics::RigidBodyRequestBus::Event(
                m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::SetGravityEnabled, m_prevGravityEnabled);
        }
    }

    void PhysicsGrabComponent::ReleaseGrabbedObject(bool notifyHoldStop = true, bool notifyRotateStop = false)
    {
        // Set Object Current Layer variable back to initial layer
        SetCurrentGrabbedCollisionLayer(m_prevGrabbedCollisionLayer);

        // Set Object Angular Damping back to original value
        SetCurrentGrabbedObjectAngularDamping(m_prevObjectAngularDamping);

        // Set Object Linear Damping back to original value
        SetCurrentGrabbedObjectLinearDamping(m_prevObjectLinearDamping);

        // Store and disable gravity for dynamic objects
        if (m_disableGravityWhileHeld && !m_isObjectKinematic)
        {
            Physics::RigidBodyRequestBus::Event(
                m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::SetGravityEnabled, m_prevGravityEnabled);
        }

        SetGrabbedObjectKinematicElseDynamic(m_isInitialObjectKinematic);
        m_isObjectKinematic = m_isInitialObjectKinematic;

        m_objectSphereCastHit = false;
        m_grabbedObjectEntityId = AZ::EntityId();
        m_lastGrabbedObjectEntityId = AZ::EntityId();

        // Restore original local TM of mesh entity before nulling pointer
        ReleaseMesh();

        // Reset throw charging on drop
        m_isChargingThrow = false;
        m_currentChargeTime = 0.f;
        m_hasNotifiedChargeComplete = false;

        if (notifyHoldStop)
        {
            PhysicsGrabNotificationBus::Broadcast(&PhysicsGrabNotificationBus::Events::OnHoldStop);
        }
        if (notifyRotateStop)
        {
            PhysicsGrabNotificationBus::Broadcast(&PhysicsGrabNotificationBus::Events::OnRotateStop);
        }
    }

    // Handles throw transition from hold and rotate
    void PhysicsGrabComponent::TransitionToThrow(bool isChargeEnabled)
    {
        // If transitioning from rotate state, reset rotation-specific properties
        if (m_state == PhysicsGrabStates::rotateState)
        {
            SetCurrentGrabbedObjectAngularDamping(m_prevObjectAngularDamping);
            SetGrabbedObjectAngularVelocity(AZ::Vector3::CreateZero());
            PhysicsGrabNotificationBus::Broadcast(&PhysicsGrabNotificationBus::Events::OnRotateStop);
        }

        // Reset to object's original Linear Damping value
        SetCurrentGrabbedObjectLinearDamping(m_prevObjectLinearDamping);

        // Start throw counter
        m_throwStateCounter = m_throwStateMaxTime;

        // Set Kinematic Rigid Body to dynamic if it was held as kinematic
        if (m_isObjectKinematic)
        {
            SetGrabbedObjectKinematicElseDynamic(false);
            m_isObjectKinematic = false;
        }

        // Restore gravity if it was disabled (before throwing)
        if (m_disableGravityWhileHeld && !m_isObjectKinematic)
        {
            Physics::RigidBodyRequestBus::Event(
                m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::SetGravityEnabled, m_prevGravityEnabled);
        }

        // Clear sphere cast hit flag and mesh pointer as object is being released
        m_objectSphereCastHit = false;
        
        // Restore original local TM of mesh entity before nulling pointer
        ReleaseMesh();

        // Determine whether to charge impulse
        if (isChargeEnabled)
        {
            // Calculate charged impulse as a linear interpolation between min and max based on charge fraction
            float chargeFraction = AZ::GetClamp(m_currentChargeTime / m_chargeTime, 0.f, 1.f);
            m_currentThrowImpulse = m_minThrowImpulse + chargeFraction * (m_maxThrowImpulse - m_minThrowImpulse);
        }
        else
        {
            // Use the fixed throw impulse for non-charged throws
            m_currentThrowImpulse = m_throwImpulse;
        }

        // Transition to throw state
        m_state = PhysicsGrabStates::throwState;
        PhysicsGrabNotificationBus::Broadcast(&PhysicsGrabNotificationBus::Events::OnHoldStop);
        PhysicsGrabNotificationBus::Broadcast(&PhysicsGrabNotificationBus::Events::OnThrowStart);

        // Reset charging state after initiating throw
        m_isChargingThrow = false;
        m_currentChargeTime = 0.f;
        m_hasNotifiedChargeComplete = false;
    }

    // Apply tidal lock to grabbed object while grabbing it. This keeps the object facing you in its last rotation while in grabbed state
    void PhysicsGrabComponent::TidalLock(float deltaTime)
    {
        // Initialize local variables for the current entity's rotation quaternion and up vector
        AZ::Quaternion grabbingEntityRotationQuat = AZ::Quaternion::CreateIdentity();

        // Determine the rotation and up vector based on whether First Person Controller is used
        #ifdef FIRST_PERSON_CONTROLLER
        if (m_useFPControllerForGrab)
        {
            if (m_fullTidalLockForFPC)
            {
                grabbingEntityRotationQuat = m_cameraRotationTransform->GetWorldRotationQuaternion();
            }
            else
            {
                grabbingEntityRotationQuat = GetEntity()->GetTransform()->GetWorldRotationQuaternion();
            }
        }
        else
        #endif
        {
            grabbingEntityRotationQuat = m_grabbingEntityPtr->GetTransform()->GetWorldRotationQuaternion();
        }

        // Compute target object rotation based on stored relative
        AZ::Quaternion targetGrabbedObjectRotation = grabbingEntityRotationQuat * m_grabbedObjectRelativeQuat;

        // Get current object rotation
        AZ::Quaternion currentGrabbedObjectRotation = AZ::Quaternion::CreateIdentity();
        AZ::TransformBus::EventResult(
            currentGrabbedObjectRotation, m_lastGrabbedObjectEntityId, &AZ::TransformInterface::GetWorldRotationQuaternion);

        // Compute error quaternion which represents the minimal rotation needed to align the 
        // current orientation to the target. Normalizing ensures numerical stability 
        // and prevents drift in quaternion operations.
        AZ::Quaternion errorQuat = targetGrabbedObjectRotation * currentGrabbedObjectRotation.GetInverseFull();
        errorQuat.Normalize();
        
        // Convert the error quaternion to axis-angle representation for easier use in PID calculations.
        // The axis-angle form provides a vector (errorAxis * errorAngle) that can be scaled by time for angular velocity.
        float errorAngle = 2.0f * acosf(AZ::GetClamp(errorQuat.GetW(), -1.0f, 1.0f));
        if (errorAngle > AZ::Constants::Pi)
            errorAngle = AZ::Constants::TwoPi - errorAngle;

        AZ::Vector3 errorAxis = errorQuat.GetImaginary().GetNormalizedSafe();
        
        // Ensure shortest rotation arc by flipping the quaternion if necessary
        if (errorQuat.GetW() < 0.0f)
            errorAxis = -errorAxis;

        AZ::Vector3 angularError = errorAxis * errorAngle;

        if (m_isObjectKinematic)
        {
            // For kinematic objects, directly apply the delta rotation to the object's transform
            AZ::Transform transform = AZ::Transform::CreateIdentity();
            AZ::TransformBus::EventResult(transform, m_lastGrabbedObjectEntityId, &AZ::TransformInterface::GetWorldTM);
            transform.SetRotation(targetGrabbedObjectRotation);
            AZ::TransformBus::Event(m_lastGrabbedObjectEntityId, &AZ::TransformInterface::SetWorldTM, transform);
        }
        else
        {
            // Compute target angular velocity
            AZ::Vector3 targetAngularVelocity = angularError / deltaTime;

            if (m_enablePIDTidalLockDynamics)
            {
                // Use PID controller to compute torque output based on the angular error.
                AZ::Vector3 angularPidOutput =
                    m_tidalLockPidController.Output(angularError, deltaTime, AZ::Vector3::CreateZero());

                // Initialize angular torque from PID output
                AZ::Vector3 angularTorque = angularPidOutput;

                // Scale torque by mass for mass-independent behavior if enabled
                if (m_massIndependentTidalLock)
                {
                    angularTorque *= m_grabbedObjectMass;
                }

                // Scale torque by effective inertia factor for scale-independent behavior if enabled
                if (m_scaleIndependentTidalLock)
                {
                    angularTorque *= m_effectiveInertiaFactor;
                }

                // Compute angular impulse from torque and delta time
                AZ::Vector3 angularImpulse = angularTorque * deltaTime;

                // Apply angular impulse to the grabbed object
                Physics::RigidBodyRequestBus::Event(
                    m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::ApplyAngularImpulse, angularImpulse);
            }
            else
            {
                // Fallback to simple angular velocity control (mass-independent)
                SetGrabbedObjectAngularVelocity(targetAngularVelocity);
            }
        }
    }

    #ifdef FIRST_PERSON_CONTROLLER
    void PhysicsGrabComponent::FreezeCharacterRotation()
    {
        if (FirstPersonController::FirstPersonControllerComponentRequestBus::HasHandlers() && m_freezeCharacterRotation)
        {
            FirstPersonController::FirstPersonControllerComponentRequestBus::Event(
                GetEntityId(), &FirstPersonController::FirstPersonControllerComponentRequestBus::Events::UpdateCharacterAndCameraYaw, 0.f, false);
            FirstPersonController::FirstPersonControllerComponentRequestBus::Event(
                GetEntityId(), &FirstPersonController::FirstPersonControllerComponentRequestBus::Events::UpdateCameraPitch, 0.f, false);
        }
        else if (m_freezeCharacterRotation)
        {
            AZ_Warning(
                "Physics Grab Component",
                false,
                "No First Person Controller Component handler available to freeze character rotation.")
        }
    }
    #endif

    void PhysicsGrabComponent::UpdateGrabDistance(float deltaTime)
    {
        // Grab distance value depends on whether grab distance input key is ignored via SetGrabbedDistanceKeyValue()
        const float grabDistanceValue = m_ignoreGrabDistanceKeyInputValue ? m_grabDistanceKeyValue : m_combinedGrabDistance;

        // Changes distance between Grabbing Entity and Grabbed object. Minimum and
        // maximum grab distances determined by m_minGrabDistance and m_maxGrabDistance, respectively
        m_grabDistance =
            AZ::GetClamp(m_grabDistance + (grabDistanceValue * m_grabDistanceSpeed * deltaTime), m_minGrabDistance, m_maxGrabDistance);
    }

    void PhysicsGrabComponent::ComputeGrabbingEntityVelocity(float deltaTime)
    {
        if (m_velocityCompensation)
        {
            // Use FPC Entity directly to capture physics timestep translations for interpolation
            if (m_useFPControllerForGrab)
            {
                m_currentGrabEntityTranslation = GetEntityPtr(GetEntityId())->GetTransform()->GetWorldTM().GetTranslation();
            }
            // Use grabbing entity to capture physics timestep translations for interpolation
            else
            {
                m_currentGrabEntityTranslation = m_grabbingEntityPtr->GetTransform()->GetWorldTM().GetTranslation();
            }
            m_grabbingEntityVelocity = (m_currentGrabEntityTranslation - m_prevGrabbingEntityTranslation) / deltaTime;
            
            // Update previous position after velocity calculation
            m_prevGrabbingEntityTranslation = m_currentGrabEntityTranslation;
        }
        else
        {
            m_grabbingEntityVelocity = AZ::Vector3::CreateZero();
        }
    }

    AZ::Entity* PhysicsGrabComponent::GetEntityPtr(AZ::EntityId pointer) const
    {
        auto ca = AZ::Interface<AZ::ComponentApplicationRequests>::Get();
        return ca->FindEntity(pointer);
    }

    // Handles disconnecting, updating, and reconnecting input bindings
    void PhysicsGrabComponent::UpdateInputBinding(
        StartingPointInput::InputEventNotificationId& eventId, AZStd::string& binding, const AZStd::string& newValue)
    {
        if (binding == newValue)
            return;

        // Disconnect from the old binding
        InputEventNotificationBus::MultiHandler::BusDisconnect(eventId);

        // Update the binding string and event ID
        binding = newValue;
        eventId = StartingPointInput::InputEventNotificationId(binding.c_str());

        // Reconnect with the new binding
        InputEventNotificationBus::MultiHandler::BusConnect(eventId);
    }

    // Event Notification methods for use in scripts
    void PhysicsGrabComponent::OnObjectSphereCastHit()
    {
    }
    void PhysicsGrabComponent::OnHoldStart()
    {
    }
    void PhysicsGrabComponent::OnHoldStop()
    {
    }
    void PhysicsGrabComponent::OnRotateStart()
    {
    }
    void PhysicsGrabComponent::OnRotateStop()
    {
    }
    void PhysicsGrabComponent::OnThrowStart()
    {
    }
    void PhysicsGrabComponent::OnThrowStop()
    {
    }
    void PhysicsGrabComponent::OnMaxThrowDistance()
    {
    }
    void PhysicsGrabComponent::OnThrowStateCounterZero()
    {
    }
    void PhysicsGrabComponent::OnChargeComplete()
    {
    }

    // Request Bus getter and setter methods for use in scripts
    AZ::EntityId PhysicsGrabComponent::GetGrabbingEntityId() const
    {
        return m_grabbingEntityPtr->GetId();
    }

    AZ::EntityId PhysicsGrabComponent::GetActiveCameraEntityId() const
    {
        return GetActiveCameraEntityPtr()->GetId();
    }

    AZ::EntityId PhysicsGrabComponent::GetGrabbedObjectEntityId() const
    {
        return m_grabbedObjectEntityId;
    }

    AZ::EntityId PhysicsGrabComponent::GetLastGrabbedObjectEntityId() const
    {
        return m_lastGrabbedObjectEntityId;
    }

    AZ::EntityId PhysicsGrabComponent::GetThrownGrabbedObjectEntityId() const
    {
        return m_thrownGrabbedObjectEntityId;
    }

    void PhysicsGrabComponent::SetThrownGrabbedObjectEntityId(const AZ::EntityId new_thrownGrabbedObjectEntityId)
    {
        m_thrownGrabbedObjectEntityId = new_thrownGrabbedObjectEntityId;
    }

    void PhysicsGrabComponent::SetGrabbingEntity(const AZ::EntityId new_grabbingEntityId)
    {
        m_grabbingEntityPtr = GetEntityPtr(new_grabbingEntityId);
    }

    AZStd::string PhysicsGrabComponent::GetStateString() const
    {
        return m_statesMap.find(m_state)->second;
    }

    bool PhysicsGrabComponent::GetIsInIdleState() const
    {
        return (m_state == PhysicsGrabStates::idleState);
    }

    bool PhysicsGrabComponent::GetIsInCheckState() const
    {
        return (m_state == PhysicsGrabStates::checkState);
    }

    bool PhysicsGrabComponent::GetIsInHeldState() const
    {
        return (m_state == PhysicsGrabStates::holdState);
    }

    bool PhysicsGrabComponent::GetIsInRotateState() const
    {
        return (m_state == PhysicsGrabStates::rotateState);
    }

    bool PhysicsGrabComponent::GetIsInThrowState() const
    {
        return (m_state == PhysicsGrabStates::throwState);
    }

    bool PhysicsGrabComponent::GetObjectSphereCastHit() const
    {
        return m_objectSphereCastHit;
    }

    bool PhysicsGrabComponent::GetStayInIdleState() const
    {
        return m_stayInIdleState;
    }
    
    void PhysicsGrabComponent::SetStayInIdleState(const bool& new_stayInIdleState)
    {
        m_stayInIdleState = new_stayInIdleState;
    }

    bool PhysicsGrabComponent::GetGrabEnableToggle() const
    {
        return m_grabEnableToggle;
    }

    void PhysicsGrabComponent::SetGrabEnableToggle(const bool& new_grabEnableToggle)
    {
        m_grabEnableToggle = new_grabEnableToggle;
    }

    bool PhysicsGrabComponent::GetRotateEnableToggle() const
    {
        return m_rotateEnableToggle;
    }

    void PhysicsGrabComponent::SetRotateEnableToggle(const bool& new_rotateEnableToggle)
    {
        m_rotateEnableToggle = new_rotateEnableToggle;
    }

    float PhysicsGrabComponent::GetGrabKeyValue() const
    {
        return m_grabKeyValue;
    }

    void PhysicsGrabComponent::SetGrabKeyValue(const float& new_grabKeyValue)
    {
        m_grabKeyValue = new_grabKeyValue;
    }

    float PhysicsGrabComponent::GetThrowKeyValue() const
    {
        return m_throwKeyValue;
    }

    void PhysicsGrabComponent::SetThrowKeyValue(const float& new_throwKeyValue)
    {
        m_throwKeyValue = new_throwKeyValue;
    }

    float PhysicsGrabComponent::GetRotateKeyValue() const
    {
        return m_rotateKeyValue;
    }

    void PhysicsGrabComponent::SetRotateKeyValue(const float& new_rotateKeyValue)
    {
        m_rotateKeyValue = new_rotateKeyValue;
    }

    float PhysicsGrabComponent::GetPitchKeyValue() const
    {
         return m_pitchKeyValue;
    }

    void PhysicsGrabComponent::SetPitchKeyValue(const float& new_pitchKeyValue, const bool& new_ignorePitchKeyInputValue)
    {
        if (new_ignorePitchKeyInputValue)
        {
            m_pitchKeyValue = new_pitchKeyValue;
            m_ignorePitchKeyInputValue = true;
        }
        else
        {
            const float newPitch = m_pitchKeyValue;
            m_pitch = new_pitchKeyValue + newPitch;
            m_ignorePitchKeyInputValue = false;
        }
    }

    float PhysicsGrabComponent::GetYawKeyValue() const
    {
        return m_yawKeyValue;
    }

    void PhysicsGrabComponent::SetYawKeyValue(const float& new_yawKeyValue, const bool& new_ignoreYawKeyInputValue)
    {
        if (new_ignoreYawKeyInputValue)
        {
            m_yawKeyValue = new_yawKeyValue;
            m_ignoreYawKeyInputValue = true;
        }
        else
        {
            const float newYaw = m_yawKeyValue;
            m_yaw = new_yawKeyValue + newYaw;
            m_ignoreYawKeyInputValue = false;
        }
    }

    float PhysicsGrabComponent::GetRollKeyValue() const
    {
        return m_rollKeyValue;
    }

    void PhysicsGrabComponent::SetRollKeyValue(const float& new_rollKeyValue, const bool& new_ignoreRollKeyInputValue)
    {
        if (new_ignoreRollKeyInputValue)
        {
            m_rollKeyValue = new_rollKeyValue;
            m_ignoreRollKeyInputValue = true;
        }
        else
        {
            const float newRoll = m_rollKeyValue;
            m_roll = new_rollKeyValue + newRoll;
            m_ignoreRollKeyInputValue = false;
        }
    }

    AZ::EntityId PhysicsGrabComponent::GetMeshEntityId() const
    {
        return m_meshEntityId;
    }

    void PhysicsGrabComponent::SetMeshEntityId(const AZ::EntityId& new_meshEntityId)
    {
        m_meshEntityId = new_meshEntityId;
        m_meshEntityPtr = m_meshEntityId.IsValid() ? GetEntityPtr(m_meshEntityId) : nullptr;
    }

    AZStd::string PhysicsGrabComponent::GetMeshTagName() const
    {
        return m_meshTagName;
    }

    void PhysicsGrabComponent::SetMeshTagName(const AZStd::string& new_meshTagName)
    {
        m_meshTagName = new_meshTagName;
        // Reset m_meshEntityId to invalid to prioritize name-based lookup
        m_meshEntityId = AZ::EntityId();
        m_meshEntityPtr = nullptr;
    }

    float PhysicsGrabComponent::GetGrabbedDistanceKeyValue() const
    {
        return m_grabDistanceKeyValue;
    }

    void PhysicsGrabComponent::SetGrabbedDistanceKeyValue(const float& new_grabDistanceKeyValue, const bool& new_ignoreGrabDistanceKeyInputValue)
    {
        if (new_ignoreGrabDistanceKeyInputValue)
        {
            m_grabDistanceKeyValue = new_grabDistanceKeyValue;
            m_ignoreGrabDistanceKeyInputValue = true;
        }
        else
        {
            const float newGrabDistance = m_grabDistanceKeyValue;
            m_combinedGrabDistance = new_grabDistanceKeyValue + newGrabDistance;
            m_ignoreGrabDistanceKeyInputValue = false;
        }
    }

    float PhysicsGrabComponent::GetGrabbedObjectDistance() const
    {
        return m_grabDistance;
    }

    void PhysicsGrabComponent::SetGrabbedObjectDistance(const float& new_grabDistance)
    {
        m_grabDistance = AZ::GetClamp(new_grabDistance, m_minGrabDistance, m_maxGrabDistance);
    }

    float PhysicsGrabComponent::GetMinGrabbedObjectDistance() const
    {
        return m_minGrabDistance;
    }

    void PhysicsGrabComponent::SetMinGrabbedObjectDistance(const float& new_minGrabDistance)
    {
        m_minGrabDistance = new_minGrabDistance;
    }

    float PhysicsGrabComponent::GetMaxGrabbedObjectDistance() const
    {
        return m_maxGrabDistance;
    }

    void PhysicsGrabComponent::SetMaxGrabbedObjectDistance(const float& new_maxGrabDistance)
    {
        m_maxGrabDistance = new_maxGrabDistance;
    }

    float PhysicsGrabComponent::GetGrabbedObjectDistanceSpeed() const
    {
        return m_grabDistanceSpeed;
    }

    void PhysicsGrabComponent::SetGrabbedObjectDistanceSpeed(const float& new_grabDistanceSpeed)
    {
        m_grabDistanceSpeed = new_grabDistanceSpeed;
    }

    float PhysicsGrabComponent::GetGrabResponse() const
    {
        return m_grabResponse;
    }

    void PhysicsGrabComponent::SetGrabResponse(const float& new_grabResponse)
    {
        m_grabResponse = new_grabResponse;
    }

    bool PhysicsGrabComponent::GetDynamicTidalLock() const
    {
        return m_dynamicTidalLock;
    }

    void PhysicsGrabComponent::SetDynamicTidalLock(const bool& new_dynamicTidalLock)
    {
        m_dynamicTidalLock = new_dynamicTidalLock;
    }

    bool PhysicsGrabComponent::GetKinematicTidalLock() const
    {
        return m_kinematicTidalLock;
    }

    void PhysicsGrabComponent::SetKinematicTidalLock(const bool& new_kinematicTidalLock)
    {
        m_kinematicTidalLock = new_kinematicTidalLock;
    }

    bool PhysicsGrabComponent::GetTidalLock() const
    {
        return m_tidalLock;
    }

    void PhysicsGrabComponent::SetTidalLock(const bool& new_tidalLock)
    {
        m_tidalLock = new_tidalLock;
    }

    bool PhysicsGrabComponent::GetFullTidalLockForFPC() const
    {
        return m_fullTidalLockForFPC;
    }

    void PhysicsGrabComponent::SetFullTidalLockForFPC(const bool& new_fullTidalLockForFPC)
    {
        m_fullTidalLockForFPC = new_fullTidalLockForFPC;
    }

    float PhysicsGrabComponent::GetDynamicYawRotateScale() const
    {
        return m_dynamicYawRotateScale;
    }

    void PhysicsGrabComponent::SetDynamicYawRotateScale(const float& new_dynamicYawRotateScale)
    {
        m_dynamicYawRotateScale = new_dynamicYawRotateScale;
    }

    float PhysicsGrabComponent::GetDynamicPitchRotateScale() const
    {
        return m_dynamicPitchRotateScale;
    }

    void PhysicsGrabComponent::SetDynamicPitchRotateScale(const float& new_dynamicPitchRotateScale)
    {
        m_dynamicPitchRotateScale = new_dynamicPitchRotateScale;
    }

    float PhysicsGrabComponent::GetDynamicRollRotateScale() const
    {
        return m_dynamicRollRotateScale;
    }

    void PhysicsGrabComponent::SetDynamicRollRotateScale(const float& new_dynamicRollRotateScale)
    {
        m_dynamicRollRotateScale = new_dynamicRollRotateScale;
    }

    float PhysicsGrabComponent::GetKinematicYawRotateScale() const
    {
        return m_kinematicYawRotateScale;
    }

    void PhysicsGrabComponent::SetKinematicYawRotateScale(const float& new_kinematicYawRotateScale)
    {
        m_kinematicYawRotateScale = new_kinematicYawRotateScale;
    }

    float PhysicsGrabComponent::GetKinematicPitchRotateScale() const
    {
        return m_kinematicPitchRotateScale;
    }

    void PhysicsGrabComponent::SetKinematicPitchRotateScale(const float& new_kinematicPitchRotateScale)
    {
        m_kinematicPitchRotateScale = new_kinematicPitchRotateScale;
    }

        float PhysicsGrabComponent::GetKinematicRollRotateScale() const
    {
        return m_kinematicRollRotateScale;
    }

    void PhysicsGrabComponent::SetKinematicRollRotateScale(const float& new_kinematicRollRotateScale)
    {
        m_kinematicRollRotateScale = new_kinematicRollRotateScale;
    }

    bool PhysicsGrabComponent::GetVelocityCompensation() const
    {
        return m_velocityCompensation;
    }

    void PhysicsGrabComponent::SetVelocityCompensation(const bool& new_velocityCompensation)
    {
        m_velocityCompensation = new_velocityCompensation;
    }

    float PhysicsGrabComponent::GetVelocityCompDampRate() const
    {
        return m_velocityCompDampRate;
    }

    void PhysicsGrabComponent::SetVelocityCompDampRate(const float& new_velocityCompDampRate)
    {
        m_velocityCompDampRate = new_velocityCompDampRate;
    }

    bool PhysicsGrabComponent::GetSmoothDynamicRotation() const
    {
        return m_smoothDynamicRotation;
    }

    void PhysicsGrabComponent::SetSmoothDynamicRotation(const bool& new_smoothDynamicRotation)
    {
        m_smoothDynamicRotation = new_smoothDynamicRotation;
    }

    float PhysicsGrabComponent::GetAngularVelocityDampRate() const
    {
        return m_angularVelocityDampRate;
    }

    void PhysicsGrabComponent::SetAngularVelocityDampRate(const float& new_angularVelocityDampRate)
    {
        m_angularVelocityDampRate = new_angularVelocityDampRate;
    }

    float PhysicsGrabComponent::GetThrowImpulse() const
    {
        return m_throwImpulse;
    }

    void PhysicsGrabComponent::SetThrowImpulse(const float& new_throwImpulse)
    {
        m_throwImpulse = new_throwImpulse;
    }

    float PhysicsGrabComponent::GetGrabbedObjectThrowStateCounter() const
    {
        return m_throwStateCounter;
    }
    
    void PhysicsGrabComponent::SetGrabbedObjectThrowStateCounter(const float& new_throwStateCounter)
    {
        m_throwStateCounter = new_throwStateCounter;
    }

    float PhysicsGrabComponent::GetGrabbedObjectThrowStateTime() const
    {
        return m_throwStateMaxTime;
    }

    void PhysicsGrabComponent::SetGrabbedObjectThrowStateTime(const float& new_throwStateMaxTime)
    {
        m_throwStateMaxTime = new_throwStateMaxTime;
    }

    bool PhysicsGrabComponent::GetEnableChargeThrow() const
    {
        return m_enableChargeThrow;
    }

    void PhysicsGrabComponent::SetEnableChargeThrow(const bool& new_enableChargeThrow)
    {
        m_enableChargeThrow = new_enableChargeThrow;
    }

    float PhysicsGrabComponent::GetMinThrowImpulse() const
    {
        return m_minThrowImpulse;
    }

    void PhysicsGrabComponent::SetMinThrowImpulse(const float& new_minThrowImpulse)
    {
        m_minThrowImpulse = new_minThrowImpulse;
    }

    float PhysicsGrabComponent::GetMaxThrowImpulse() const
    {
        return m_maxThrowImpulse;
    }

    void PhysicsGrabComponent::SetMaxThrowImpulse(const float& new_maxThrowImpulse)
    {
        m_maxThrowImpulse = new_maxThrowImpulse;
    }

    float PhysicsGrabComponent::GetCurrentThrowImpulse() const
    {
        return m_currentThrowImpulse;
    }

    float PhysicsGrabComponent::GetChargeTime() const
    {
        return m_chargeTime;
    }

    void PhysicsGrabComponent::SetChargeTime(const float& new_chargeTime)
    {
        m_chargeTime = new_chargeTime;
    }

    float PhysicsGrabComponent::GetCurrentChargeTime() const
    {
        return m_currentChargeTime;
    }

    bool PhysicsGrabComponent::GetEnableChargeWhileRotating() const
    {
        return m_enableChargeWhileRotating;
    }

    void PhysicsGrabComponent::SetEnableChargeWhileRotating(const bool& new_enableChargeWhileRotating)
    {
        m_enableChargeWhileRotating = new_enableChargeWhileRotating;
    }

    bool PhysicsGrabComponent::GetIsChargingThrow() const
    {
            return m_isChargingThrow;
    }

    float PhysicsGrabComponent::GetSphereCastRadius() const
    {
        return m_sphereCastRadius;
    }

    void PhysicsGrabComponent::SetSphereCastRadius(const float& new_sphereCastRadius)
    {
        m_sphereCastRadius = new_sphereCastRadius;
    }

    float PhysicsGrabComponent::GetSphereCastDistance() const
    {
        return m_sphereCastDistance;
    }

    void PhysicsGrabComponent::SetSphereCastDistance(const float& new_sphereCastDistance)
    {
        m_sphereCastDistance = new_sphereCastDistance;
    }

    AZStd::string PhysicsGrabComponent::GetGrabbedCollisionGroup() const
    {
        AZStd::string groupName;
        Physics::CollisionRequestBus::BroadcastResult(
            groupName, &Physics::CollisionRequests::GetCollisionGroupName, m_grabbedCollisionGroup);
        return groupName;
    }

    void PhysicsGrabComponent::SetGrabbedCollisionGroup(const AZStd::string& new_grabbedCollisionGroupName)
    {
        bool success = false;
        AzPhysics::CollisionGroup collisionGroup;
        Physics::CollisionRequestBus::BroadcastResult(
            success, &Physics::CollisionRequests::TryGetCollisionGroupByName, new_grabbedCollisionGroupName, collisionGroup);
        if (success)
        {
            m_grabbedCollisionGroup = collisionGroup;
            const AzPhysics::CollisionConfiguration& configuration =
                AZ::Interface<AzPhysics::SystemInterface>::Get()->GetConfiguration()->m_collisionConfig;
            m_grabbedCollisionGroupId = configuration.m_collisionGroups.FindGroupIdByName(new_grabbedCollisionGroupName);
        }
    }

    AZStd::string PhysicsGrabComponent::GetCurrentGrabbedCollisionLayerName() const
    {
        AZStd::string currentGrabbedCollisionLayerName;
        Physics::CollisionFilteringRequestBus::EventResult(
            currentGrabbedCollisionLayerName,
            m_lastGrabbedObjectEntityId,
            &Physics::CollisionFilteringRequestBus::Events::GetCollisionLayerName);
        return currentGrabbedCollisionLayerName;
    }

    void PhysicsGrabComponent::SetCurrentGrabbedCollisionLayerByName(const AZStd::string& new_currentGrabbedCollisionLayerName)
    {
        bool success = false;
        AzPhysics::CollisionLayer grabbedCollisionLayer;
        Physics::CollisionRequestBus::BroadcastResult(
            success, &Physics::CollisionRequests::TryGetCollisionLayerByName, new_currentGrabbedCollisionLayerName, grabbedCollisionLayer);
        if (success)
        {
            m_currentGrabbedCollisionLayerName = new_currentGrabbedCollisionLayerName;
            m_currentGrabbedCollisionLayer = grabbedCollisionLayer;
            Physics::CollisionFilteringRequestBus::Event(
                m_lastGrabbedObjectEntityId,
                &Physics::CollisionFilteringRequestBus::Events::SetCollisionLayer,
                m_currentGrabbedCollisionLayerName,
                AZ::Crc32());
        }
    }

    AzPhysics::CollisionLayer PhysicsGrabComponent::GetCurrentGrabbedCollisionLayer() const
    {
        AZStd::string grabbedCollisionLayerName;
        Physics::CollisionFilteringRequestBus::EventResult(
            grabbedCollisionLayerName, m_lastGrabbedObjectEntityId, &Physics::CollisionFilteringRequestBus::Events::GetCollisionLayerName);
        AzPhysics::CollisionLayer grabbedCollisionLayer;
        Physics::CollisionRequestBus::BroadcastResult(
            grabbedCollisionLayer, &Physics::CollisionRequests::GetCollisionLayerByName, grabbedCollisionLayerName);
        return grabbedCollisionLayer;
    }

    void PhysicsGrabComponent::SetCurrentGrabbedCollisionLayer(const AzPhysics::CollisionLayer& new_currentGrabbedCollisionLayer)
    {
        m_currentGrabbedCollisionLayer = new_currentGrabbedCollisionLayer;
        const AzPhysics::CollisionConfiguration& configuration =
            AZ::Interface<AzPhysics::SystemInterface>::Get()->GetConfiguration()->m_collisionConfig;
        m_currentGrabbedCollisionLayerName = configuration.m_collisionLayers.GetName(m_currentGrabbedCollisionLayer);
        Physics::CollisionFilteringRequestBus::Event(
            m_lastGrabbedObjectEntityId,
            &Physics::CollisionFilteringRequestBus::Events::SetCollisionLayer,
            m_currentGrabbedCollisionLayerName,
            AZ::Crc32());
    }

    AZStd::string PhysicsGrabComponent::GetPrevGrabbedCollisionLayerName() const
    {
        const AzPhysics::CollisionConfiguration& configuration =
            AZ::Interface<AzPhysics::SystemInterface>::Get()->GetConfiguration()->m_collisionConfig;
        return configuration.m_collisionLayers.GetName(m_prevGrabbedCollisionLayer);
    }

    void PhysicsGrabComponent::SetPrevGrabbedCollisionLayerByName(const AZStd::string& new_prevGrabbedCollisionLayerName)
    {
        bool success = false;
        AzPhysics::CollisionLayer prevGrabbedCollisionLayer;
        Physics::CollisionRequestBus::BroadcastResult(
            success, &Physics::CollisionRequests::TryGetCollisionLayerByName, new_prevGrabbedCollisionLayerName, prevGrabbedCollisionLayer);
        if (success)
            m_prevGrabbedCollisionLayer = prevGrabbedCollisionLayer;
    }

    AzPhysics::CollisionLayer PhysicsGrabComponent::GetPrevGrabbedCollisionLayer() const
    {
        return m_prevGrabbedCollisionLayer;
    }

    void PhysicsGrabComponent::SetPrevGrabbedCollisionLayer(const AzPhysics::CollisionLayer& new_prevGrabbedCollisionLayer)
    {
        m_prevGrabbedCollisionLayer = new_prevGrabbedCollisionLayer;
    }

    AZStd::string PhysicsGrabComponent::GetTempGrabbedCollisionLayerName() const
    {
        AZStd::string tempGrabbedCollisionLayerName;
        Physics::CollisionRequestBus::BroadcastResult(
            tempGrabbedCollisionLayerName, &Physics::CollisionRequests::GetCollisionLayerName, m_tempGrabbedCollisionLayer);
        return tempGrabbedCollisionLayerName;
    }

    void PhysicsGrabComponent::SetTempGrabbedCollisionLayerByName(const AZStd::string& new_tempGrabbedCollisionLayerName)
    {
        bool success = false;
        AzPhysics::CollisionLayer tempGrabbedCollisionLayer;
        Physics::CollisionRequestBus::BroadcastResult(
            success, &Physics::CollisionRequests::TryGetCollisionLayerByName, new_tempGrabbedCollisionLayerName, tempGrabbedCollisionLayer);
        if (success)
        {
            m_tempGrabbedCollisionLayer = tempGrabbedCollisionLayer;
        }
    }

    AzPhysics::CollisionLayer PhysicsGrabComponent::GetTempGrabbedCollisionLayer() const
    {
        return m_tempGrabbedCollisionLayer;
    }

    void PhysicsGrabComponent::SetTempGrabbedCollisionLayer(const AzPhysics::CollisionLayer& new_tempGrabbedCollisionLayer)
    {
        m_tempGrabbedCollisionLayer = new_tempGrabbedCollisionLayer;
        const AzPhysics::CollisionConfiguration& configuration =
            AZ::Interface<AzPhysics::SystemInterface>::Get()->GetConfiguration()->m_collisionConfig;
        AZStd::string tempGrabbedCollisionLayerName = configuration.m_collisionLayers.GetName(m_tempGrabbedCollisionLayer);
        Physics::CollisionFilteringRequestBus::Event(
            m_lastGrabbedObjectEntityId,
            &Physics::CollisionFilteringRequestBus::Events::SetCollisionLayer,
            tempGrabbedCollisionLayerName,
            AZ::Crc32());
    }

    bool PhysicsGrabComponent::GetGrabbedObjectKinematicElseDynamic() const
    {
        bool isObjectKinematic = false;
        Physics::RigidBodyRequestBus::EventResult(
            isObjectKinematic, m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequestBus::Events::IsKinematic);

        return isObjectKinematic;
    }

    void PhysicsGrabComponent::SetGrabbedObjectKinematicElseDynamic(const bool& isKinematic)
    {
        Physics::RigidBodyRequestBus::Event(m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::SetKinematic, isKinematic);
    }

    bool PhysicsGrabComponent::GetInitialGrabbedObjectIsKinematic() const
    {
        return m_isInitialObjectKinematic;
    }

    float PhysicsGrabComponent::GetCurrentGrabbedObjectAngularDamping() const
    {
        float currentObjectAngularDamping = 0.f;

        Physics::RigidBodyRequestBus::EventResult(
            currentObjectAngularDamping, m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::GetAngularDamping);

        return currentObjectAngularDamping;
    }

    void PhysicsGrabComponent::SetCurrentGrabbedObjectAngularDamping(const float& new_currentObjectAngularDamping)
    {
        m_currentObjectAngularDamping = new_currentObjectAngularDamping;

        Physics::RigidBodyRequestBus::Event(
            m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::SetAngularDamping, new_currentObjectAngularDamping);
    }

    float PhysicsGrabComponent::GetPrevGrabbedObjectAngularDamping() const
    {
        return m_prevObjectAngularDamping;
    }

    void PhysicsGrabComponent::SetPrevGrabbedObjectAngularDamping(const float& new_prevObjectAngularDamping)
    {
        m_prevObjectAngularDamping = new_prevObjectAngularDamping;
    }

    float PhysicsGrabComponent::GetTempGrabbedObjectAngularDamping() const
    {
        return m_tempObjectAngularDamping;
    }

    void PhysicsGrabComponent::SetTempGrabbedObjectAngularDamping(const float& new_tempObjectAngularDamping)
    {
        m_tempObjectAngularDamping = new_tempObjectAngularDamping;
        Physics::RigidBodyRequestBus::Event(
            m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::SetAngularDamping, m_tempObjectAngularDamping);
    }

    float PhysicsGrabComponent::GetCurrentGrabbedObjectLinearDamping() const
    {
        float currentObjectLinearDamping = 0.f;

        Physics::RigidBodyRequestBus::EventResult(
            currentObjectLinearDamping, m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::GetLinearDamping);

        return currentObjectLinearDamping;
    }

    void PhysicsGrabComponent::SetCurrentGrabbedObjectLinearDamping(const float& new_currentObjectLinearDamping)
    {
        m_currentObjectLinearDamping = new_currentObjectLinearDamping;

        Physics::RigidBodyRequestBus::Event(
            m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::SetLinearDamping, new_currentObjectLinearDamping);
    }

    float PhysicsGrabComponent::GetPrevGrabbedObjectLinearDamping() const
    {
        return m_prevObjectLinearDamping;
    }

    void PhysicsGrabComponent::SetPrevGrabbedObjectLinearDamping(const float& new_prevObjectLinearDamping)
    {
        m_prevObjectLinearDamping = new_prevObjectLinearDamping;
    }

    float PhysicsGrabComponent::GetTempGrabbedObjectLinearDamping() const
    {
        return m_tempObjectLinearDamping;
    }

    void PhysicsGrabComponent::SetTempGrabbedObjectLinearDamping(const float& new_tempObjectLinearDamping)
    {
        m_tempObjectLinearDamping = new_tempObjectLinearDamping;
        Physics::RigidBodyRequestBus::Event(
            m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::SetLinearDamping, m_tempObjectLinearDamping);
    }

    AZ::Vector3 PhysicsGrabComponent::GetGrabbedObjectAngularVelocity() const
    {
        AZ::Vector3 grabbedObjectAngularVelocity = AZ::Vector3::CreateZero();

        Physics::RigidBodyRequestBus::EventResult(
            grabbedObjectAngularVelocity, m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::GetAngularVelocity);
        return grabbedObjectAngularVelocity;
    }

    void PhysicsGrabComponent::SetGrabbedObjectAngularVelocity(const AZ::Vector3& new_grabbedObjectAngularVelocity)
    {
        m_grabbedObjectAngularVelocity = new_grabbedObjectAngularVelocity;

        Physics::RigidBodyRequestBus::Event(
            m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::SetAngularVelocity, m_grabbedObjectAngularVelocity);
    }

    bool PhysicsGrabComponent::GetInitialAngularVelocityZero() const
    {
        return m_initialAngularVelocityZero;
    }

    void PhysicsGrabComponent::SetInitialAngularVelocityZero(const bool& new_initialAngularVelocityZero)
    {
        m_initialAngularVelocityZero = new_initialAngularVelocityZero;
    }
    
    void PhysicsGrabComponent::ForceTransition(const PhysicsGrabStates& targetState)
    {
        m_forceTransition = true;
        m_targetState = targetState;
    }

    void PhysicsGrabComponent::SetStateLocked(const bool& isLocked)
    {
        m_isStateLocked = isLocked;
    }

    bool PhysicsGrabComponent::GetStateLocked() const
    {
        return m_isStateLocked;
    }

    bool PhysicsGrabComponent::GetDisableGravityWhileHeld() const
    {
        return m_disableGravityWhileHeld;
    }

    void PhysicsGrabComponent::SetDisableGravityWhileHeld(const bool& new_disableGravityWhileHeld)
    {
        m_disableGravityWhileHeld = new_disableGravityWhileHeld;
    }

    bool PhysicsGrabComponent::GetOffsetGrab() const
    {
        return m_offsetGrab;
    }

    void PhysicsGrabComponent::SetOffsetGrab(const bool& new_offsetGrab)
    {
        m_offsetGrab = new_offsetGrab;
    }

    bool PhysicsGrabComponent::GetGravityAppliesToPointRotation() const
    {
        return m_gravityAppliesToPointRotation;
    }

    void PhysicsGrabComponent::SetGravityAppliesToPointRotation(const bool& new_gravityAppliesToPointRotation)
    {
        m_gravityAppliesToPointRotation = new_gravityAppliesToPointRotation;
    }

    bool PhysicsGrabComponent::GetMassIndependentThrow() const
    {
        return m_massIndependentThrow;
    }

    void PhysicsGrabComponent::SetMassIndependentThrow(const bool& new_massIndependentThrow)
    {
        m_massIndependentThrow = new_massIndependentThrow;
    }

    bool PhysicsGrabComponent::GetEnablePIDHeldDynamics() const
    {
        return m_enablePIDHeldDynamics;
    }

    void PhysicsGrabComponent::SetEnablePIDHeldDymamics(const bool& new_enablePIDHeldDynamics)
    {
        m_enablePIDHeldDynamics = new_enablePIDHeldDynamics;
    }

    bool PhysicsGrabComponent::GetMassIndependentHeldPID() const
    {
        return m_massIndependentHeldPID;
    }

    void PhysicsGrabComponent::SetMassIndependentHeldPID(const bool& new_massIndependentHeldPID)
    {
        m_massIndependentHeldPID = new_massIndependentHeldPID;
    }

    float PhysicsGrabComponent::GetHeldProportionalGain() const
    {
        return m_heldProportionalGain;
    }

    void PhysicsGrabComponent::SetHeldProportionalGain(const float& new_heldProportionalGain)
    {
        m_heldProportionalGain = new_heldProportionalGain;
        m_pidController.SetProportionalGain(new_heldProportionalGain);
    }

    float PhysicsGrabComponent::GetHeldIntegralGain() const
    {
        return m_heldIntegralGain;
    }

    void PhysicsGrabComponent::SetHeldIntegralGain(const float& new_heldIntegralGain)
    {
        m_heldIntegralGain = new_heldIntegralGain;
        m_pidController.SetIntegralGain(new_heldIntegralGain);
    }

    float PhysicsGrabComponent::GetHeldDerivativeGain() const
    {
        return m_heldDerivativeGain;
    }

    void PhysicsGrabComponent::SetHeldDerivativeGain(const float& new_heldDerivativeGain)
    {
        m_heldDerivativeGain = new_heldDerivativeGain;
        m_pidController.SetDerivativeGain(new_heldDerivativeGain);
    }

    float PhysicsGrabComponent::GetHeldIntegralWindupLimit() const
    {
        return m_heldIntegralWindupLimit;
    }

    void PhysicsGrabComponent::SetHeldIntegralWindupLimit(const float& new_heldIntegralWindupLimit)
    {
        m_heldIntegralWindupLimit = new_heldIntegralWindupLimit;
        m_pidController.SetIntegralWindupLimit(new_heldIntegralWindupLimit);
    }

    float PhysicsGrabComponent::GetHeldDerivativeFilterAlpha() const
    {
        return m_heldDerivativeFilterAlpha;
    }

    void PhysicsGrabComponent::SetHeldDerivativeFilterAlpha(const float& new_heldDerivativeFilterAlpha)
    {
        m_heldDerivativeFilterAlpha = new_heldDerivativeFilterAlpha;
        m_pidController.SetDerivativeFilterAlpha(new_heldDerivativeFilterAlpha);
    }

    PidController<AZ::Vector3>::DerivativeCalculationMode PhysicsGrabComponent::GetHeldDerivativeMode() const
    {
        return m_heldDerivativeMode;
    }

    void PhysicsGrabComponent::SetHeldDerivativeMode(
        const PidController<AZ::Vector3>::DerivativeCalculationMode& new_heldDerivativeMode)
    {
        m_heldDerivativeMode = new_heldDerivativeMode;
        m_pidController.SetDerivativeMode(new_heldDerivativeMode);
        m_pidController.Reset();
    }

    bool PhysicsGrabComponent::GetEnablePIDTidalLockDynamics() const
    {
        return m_enablePIDTidalLockDynamics;
    }

    void PhysicsGrabComponent::SetEnablePIDTidalLockDynamics(const bool& new_enablePIDTidalLockDynamics)
    {
        m_enablePIDTidalLockDynamics = new_enablePIDTidalLockDynamics;
    }

    bool PhysicsGrabComponent::GetMassIndependentTidalLock() const
    {
        return m_massIndependentTidalLock;
    }

    void PhysicsGrabComponent::SetMassIndependentTidalLock(const bool& new_massIndependentTidalLock)
    {
        m_massIndependentTidalLock = new_massIndependentTidalLock;
    }

    bool PhysicsGrabComponent::GetScaleIndependentTidalLock() const
    {
        return m_scaleIndependentTidalLock;
    }

    void PhysicsGrabComponent::SetScaleIndependentTidalLock(const bool& new_scaleIndependentTidalLock)
    {
        m_scaleIndependentTidalLock = new_scaleIndependentTidalLock;
    }

    float PhysicsGrabComponent::GetTidalLockProportionalGain() const
    {
        return m_tidalLockProportionalGain;
    }

    void PhysicsGrabComponent::SetTidalLockProportionalGain(const float& new_tidalLockProportionalGain)
    {
        m_tidalLockProportionalGain = new_tidalLockProportionalGain;
        m_tidalLockPidController.SetProportionalGain(new_tidalLockProportionalGain);
    }

    float PhysicsGrabComponent::GetTidalLockIntegralGain() const
    {
        return m_tidalLockIntegralGain;
    }

    void PhysicsGrabComponent::SetTidalLockIntegralGain(const float& new_tidalLockIntegralGain)
    {
        m_tidalLockIntegralGain = new_tidalLockIntegralGain;
        m_tidalLockPidController.SetIntegralGain(new_tidalLockIntegralGain);
    }

    float PhysicsGrabComponent::GetTidalLockDerivativeGain() const
    {
        return m_tidalLockDerivativeGain;
    }

    void PhysicsGrabComponent::SetTidalLockDerivativeGain(const float& new_tidalLockDerivativeGain)
    {
        m_tidalLockDerivativeGain = new_tidalLockDerivativeGain;
        m_tidalLockPidController.SetDerivativeGain(new_tidalLockDerivativeGain);
    }

    float PhysicsGrabComponent::GetTidalLockIntegralWindupLimit() const
    {
        return m_tidalLockIntegralWindupLimit;
    }

    void PhysicsGrabComponent::SetTidalLockIntegralWindupLimit(const float& new_tidalLockIntegralWindupLimit)
    {
        m_tidalLockIntegralWindupLimit = new_tidalLockIntegralWindupLimit;
        m_tidalLockPidController.SetIntegralWindupLimit(new_tidalLockIntegralWindupLimit);
    }

    float PhysicsGrabComponent::GetTidalLockDerivativeFilterAlpha() const
    {
        return m_tidalLockDerivativeFilterAlpha;
    }

    void PhysicsGrabComponent::SetTidalLockDerivativeFilterAlpha(const float& new_tidalLockDerivativeFilterAlpha)
    {
        m_tidalLockDerivativeFilterAlpha = new_tidalLockDerivativeFilterAlpha;
        m_tidalLockPidController.SetDerivativeFilterAlpha(new_tidalLockDerivativeFilterAlpha);
    }

    PidController<AZ::Vector3>::DerivativeCalculationMode PhysicsGrabComponent::GetTidalLockDerivativeMode() const
    {
        return m_tidalLockDerivativeMode;
    }

    void PhysicsGrabComponent::SetTidalLockDerivativeMode(
        const PidController<AZ::Vector3>::DerivativeCalculationMode& new_tidalLockDerivativeMode)
    {
        m_tidalLockDerivativeMode = new_tidalLockDerivativeMode;
        m_tidalLockPidController.SetDerivativeMode(new_tidalLockDerivativeMode);
        m_tidalLockPidController.Reset();
    }

    AZStd::string PhysicsGrabComponent::GetGrabInputKey() const
    {
        return m_strGrab;
    }
    void PhysicsGrabComponent::SetGrabInputKey(const AZStd::string& keyName)
    {
        UpdateInputBinding(m_grabEventId, m_strGrab, keyName);
    }

    AZStd::string PhysicsGrabComponent::GetThrowInputKey() const
    {
        return m_strThrow;
    }
    void PhysicsGrabComponent::SetThrowInputKey(const AZStd::string& keyName)
    {
        UpdateInputBinding(m_throwEventId, m_strThrow, keyName);
    }

    AZStd::string PhysicsGrabComponent::GetRotateInputKey() const
    {
        return m_strRotate;
    }
    void PhysicsGrabComponent::SetRotateInputKey(const AZStd::string& keyName)
    {
        UpdateInputBinding(m_rotateEventId, m_strRotate, keyName);
    }

    AZStd::string PhysicsGrabComponent::GetRotatePitchInputKey() const
    {
        return m_strRotatePitch;
    }
    void PhysicsGrabComponent::SetRotatePitchInputKey(const AZStd::string& keyName)
    {
        UpdateInputBinding(m_rotatePitchEventId, m_strRotatePitch, keyName);
    }

    AZStd::string PhysicsGrabComponent::GetRotateYawInputKey() const
    {
        return m_strRotateYaw;
    }
    void PhysicsGrabComponent::SetRotateYawInputKey(const AZStd::string& keyName)
    {
        UpdateInputBinding(m_rotateYawEventId, m_strRotateYaw, keyName);
    }

    AZStd::string PhysicsGrabComponent::GetRotateRollInputKey() const
    {
        return m_strRotateRoll;
    }
    void PhysicsGrabComponent::SetRotateRollInputKey(const AZStd::string& keyName)
    {
        UpdateInputBinding(m_rotateRollEventId, m_strRotateRoll, keyName);
    }

    AZStd::string PhysicsGrabComponent::GetGrabDistanceInputKey() const
    {
        return m_strGrabDistance;
    }
    void PhysicsGrabComponent::SetGrabDistanceInputKey(const AZStd::string& keyName)
    {
        UpdateInputBinding(m_grabDistanceEventId, m_strGrabDistance, keyName);
    }
} // namespace PhysicsGrab
