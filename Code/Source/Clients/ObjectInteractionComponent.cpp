#include "ObjectInteractionComponent.h"

#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzFramework/Physics/CollisionBus.h>
#include <AzFramework/Physics/NameConstants.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <AzFramework/Physics/SystemBus.h>
#include <System/PhysXSystem.h>

namespace ObjectInteraction
{
    using namespace StartingPointInput;

    void ObjectInteractionComponent::Reflect(AZ::ReflectContext* rc)
    {
        if (auto sc = azrtti_cast<AZ::SerializeContext*>(rc))
        {
            sc->Class<ObjectInteractionComponent, AZ::Component>()

                // Grab Input Binding Keys
                ->Field("Grab Input Key", &ObjectInteractionComponent::m_strGrab)
                ->Field("Grab Distance Input Key", &ObjectInteractionComponent::m_strGrabDistance)
                ->Field("Throw Input Key", &ObjectInteractionComponent::m_strThrow)
                ->Field("Rotate Enable Input Key", &ObjectInteractionComponent::m_strRotate)
                ->Field("Rotate Pitch Key", &ObjectInteractionComponent::m_strRotatePitch)
                ->Field("Rotate Yaw Key", &ObjectInteractionComponent::m_strRotateYaw)
                ->Field("Rotate Roll Key", &ObjectInteractionComponent::m_strRotateRoll)

                ->Field("GrabbingEntityId", &ObjectInteractionComponent::m_grabbingEntityId)
                ->Field("Mesh Smoothing", &ObjectInteractionComponent::m_meshSmoothing)
                ->Field("Grab Mesh Entity Name", &ObjectInteractionComponent::m_meshEntityName)
                #ifdef FIRST_PERSON_CONTROLLER
                ->Field("Use First Person Controller For Grab", &ObjectInteractionComponent::m_useFPControllerForGrab)
                ->Field("Freeze Character Rotation", &ObjectInteractionComponent::m_freezeCharacterRotation)
                #endif
                ->Field("Grab Enable Toggle", &ObjectInteractionComponent::m_grabEnableToggle)
                ->Field("Maintain Grab", &ObjectInteractionComponent::m_grabMaintained)
                ->Field("Kinematic While Grabbing", &ObjectInteractionComponent::m_kinematicWhileHeld)
                ->Field("Rotate Enable Toggle", &ObjectInteractionComponent::m_rotateEnableToggle)
                ->Field("Disable Gravity", &ObjectInteractionComponent::m_disableGravityWhileHeld)
                ->Field("Tidal Lock Grabbed Object", &ObjectInteractionComponent::m_tidalLock)
                ->Field("Sphere Cast Radius", &ObjectInteractionComponent::m_sphereCastRadius)
                ->Attribute(AZ::Edit::Attributes::Suffix, " " + Physics::NameConstants::GetLengthUnit())
                ->Field("Sphere Cast Distance", &ObjectInteractionComponent::m_sphereCastDistance)
                ->Attribute(AZ::Edit::Attributes::Suffix, " " + Physics::NameConstants::GetLengthUnit())
                ->Field("Default Grab Distance", &ObjectInteractionComponent::m_initialGrabDistance)
                ->Attribute(AZ::Edit::Attributes::Suffix, " " + Physics::NameConstants::GetLengthUnit())
                ->Field("Min Grab Distance", &ObjectInteractionComponent::m_minGrabDistance)
                ->Attribute(AZ::Edit::Attributes::Suffix, " " + Physics::NameConstants::GetLengthUnit())
                ->Field("Max Grab Distance", &ObjectInteractionComponent::m_maxGrabDistance)
                ->Attribute(AZ::Edit::Attributes::Suffix, " " + Physics::NameConstants::GetLengthUnit())
                ->Field("Grab Distance Speed", &ObjectInteractionComponent::m_grabDistanceSpeed)
                ->Attribute(AZ::Edit::Attributes::Suffix, " " + Physics::NameConstants::GetSpeedUnit())
                ->Field("Enable Mass Independent Throw", &ObjectInteractionComponent::m_massIndependentThrow)
                ->Field("Throw Impulse", &ObjectInteractionComponent::m_throwImpulse)
                ->Attribute(AZ::Edit::Attributes::Suffix, AZStd::string::format(" N%ss", Physics::NameConstants::GetInterpunct().c_str()))
                ->Field("Grab Response", &ObjectInteractionComponent::m_grabResponse)
                ->Attribute(
                    AZ::Edit::Attributes::Suffix,
                    AZStd::string::format(
                        " s%s%s",
                        Physics::NameConstants::GetSuperscriptMinus().c_str(),
                        Physics::NameConstants::GetSuperscriptOne().c_str()))
                ->Field("Kinematic Horizontal Rotate Scale", &ObjectInteractionComponent::m_kinematicYawRotateScale)
                ->Attribute(AZ::Edit::Attributes::Suffix, " rad/s")
                ->Field("Kinematic Vertical Rotate Scale", &ObjectInteractionComponent::m_kinematicPitchRotateScale)
                ->Attribute(AZ::Edit::Attributes::Suffix, " rad/s")
                ->Field("Dynamic Horizontal Rotate Scale", &ObjectInteractionComponent::m_dynamicYawRotateScale)
                ->Attribute(AZ::Edit::Attributes::Suffix, " rad/s")
                ->Field("Dynamic Vertical Rotate Scale", &ObjectInteractionComponent::m_dynamicPitchRotateScale)
                ->Attribute(AZ::Edit::Attributes::Suffix, " rad/s")
                ->Field("Angular Damping", &ObjectInteractionComponent::m_tempObjectAngularDamping)
                ->Field("Linear Damping", &ObjectInteractionComponent::m_tempObjectLinearDamping)
                ->Field("Velocity Compensation", &ObjectInteractionComponent::m_velocityCompensation)
                ->Field("Velocity Compensation Damp Rate", &ObjectInteractionComponent::m_velocityCompDampRate)
                ->Field("Smooth Dynamic Rotation", &ObjectInteractionComponent::m_smoothDynamicRotation)
                ->Field("Angular Velocity Damp Rate", &ObjectInteractionComponent::m_angularVelocityDampRate)
                ->Field("Grabbed Object Collision Group", &ObjectInteractionComponent::m_grabbedCollisionGroupId)
                ->Field("Grabbed Object Temporary Collision Layer", &ObjectInteractionComponent::m_tempGrabbedCollisionLayer)

                ->Field("PID Held Dynamics", &ObjectInteractionComponent::m_enablePIDHeldDynamics)
                ->Field("Mass Independent PID", &ObjectInteractionComponent::m_massIndependentPID)
                ->Field("PID P Gain", &ObjectInteractionComponent::m_proportionalGain)
                ->Attribute(AZ::Edit::Attributes::Suffix, " N/m")
                ->Field("PID I Gain", &ObjectInteractionComponent::m_integralGain)
                ->Attribute(AZ::Edit::Attributes::Suffix, " N/(m*s)")
                ->Field("PID D Gain", &ObjectInteractionComponent::m_derivativeGain)
                ->Attribute(AZ::Edit::Attributes::Suffix, " N*s/m")
                ->Field("PID Integral Limit", &ObjectInteractionComponent::m_integralWindupLimit)
                ->Attribute(AZ::Edit::Attributes::Suffix, " N")
                ->Field("PID Deriv Filter Alpha", &ObjectInteractionComponent::m_derivFilterAlpha)

                ->Field("Enable PID Tidal Lock Dynamics", &ObjectInteractionComponent::m_enablePIDTidalLockDynamics)
                ->Field("Mass Independent Tidal Lock", &ObjectInteractionComponent::m_massIndependentTidalLock)
                ->Field("Tidal Lock PID P Gain", &ObjectInteractionComponent::m_tidalLockProportionalGain)
                ->Field("Tidal Lock PID I Gain", &ObjectInteractionComponent::m_tidalLockIntegralGain)
                ->Field("Tidal Lock PID D Gain", &ObjectInteractionComponent::m_tidalLockDerivativeGain)
                ->Field("Tidal Lock PID Integral Limit", &ObjectInteractionComponent::m_tidalLockIntegralWindupLimit)
                ->Field("Tidal Lock PID Deriv Filter Alpha", &ObjectInteractionComponent::m_tidalLockDerivFilterAlpha)
                ->Version(1);

            if (AZ::EditContext* ec = sc->GetEditContext())
            {
                using namespace AZ::Edit;
                ec->Class<ObjectInteractionComponent>("Object Interaction", "[Object Interaction Component]")
                    ->ClassElement(ClassElements::EditorData, "")
                    ->Attribute(Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))

                    ->DataElement(
                        0,
                        &ObjectInteractionComponent::m_grabbingEntityId,
                        "Grab Entity",
                        "Reference entity that interacts with objects. If left blank, Camera entity will be used by default.")
                    ->DataElement(
                        0,
                        &ObjectInteractionComponent::m_meshSmoothing,
                        "Mesh Smoothing",
                        "Enables smooth interpolation of the mesh transform for dynamic objects to reduce stuttering.")
                    ->DataElement(
                        0,
                        &ObjectInteractionComponent::m_meshEntityName,
                        "Mesh Mesh Entity Name",
                        "Name (or partial name) of the child entity to use for mesh interpolation (e.g., 'Grab Mesh').")

                    // Input Binding Keys
                    ->ClassElement(AZ::Edit::ClassElements::Group, "Input Bindings")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(nullptr, &ObjectInteractionComponent::m_strGrab, "Grab Key", "Grab interaction input binding")
                    ->DataElement(nullptr, &ObjectInteractionComponent::m_strGrabDistance, "Grab Distance Key", "Grab distance input binding")
                    ->DataElement(nullptr, &ObjectInteractionComponent::m_strThrow, "Throw Input Key", "Throw object input binding")
                    ->DataElement(nullptr, &ObjectInteractionComponent::m_strRotate, "Rotate Enable Key", "Enable rotate object input binding")
                    ->DataElement(nullptr, &ObjectInteractionComponent::m_strRotatePitch, "Rotate Pitch Key", "Rotate object about X axis input binding")
                    ->DataElement(nullptr, &ObjectInteractionComponent::m_strRotateYaw, "Rotate Yaw Key", "Rotate object about Z axis input binding")
                    ->DataElement(nullptr, &ObjectInteractionComponent::m_strRotateRoll, "Rotate Roll Key", "Rotate object about Y axis input binding")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Toggle Preferences")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    #ifdef FIRST_PERSON_CONTROLLER
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_useFPControllerForGrab,
                        "Use First Person Controller For Grab",
                        "Use First Person Controller player character for grab reference. Enabling this creates tighter tracking by bypassing "
                        "potential camera interpolation lag.")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_freezeCharacterRotation,
                        "Freeze Character Rotation",
                        "Enables character controller rotation while in Rotate State.")
                    #endif
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_grabEnableToggle,
                        "Grab Enable Toggle",
                        "Determines whether pressing Grab Key toggles Grab mode. Disabling this requires the Grab key to be held to "
                        "maintain Grab mode.")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_grabMaintained,
                        "Maintain Grab",
                        "Grabbed Object remains held even if sphere cast no longer intersects it. This prevents the Grabbed Object from "
                        "flying off when quickly changing directions.")
                    ->DataElement(
                        nullptr, &ObjectInteractionComponent::m_kinematicWhileHeld, "Kinematic Grabbed Object", "Sets the grabbed object to kinematic.")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_rotateEnableToggle,
                        "Rotate Enable Toggle",
                        "Determines whether pressing Rotate Key toggles Rotate mode. Disabling this requires the Rotate key to be held to "
                        "maintain Rotate mode.") 
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_disableGravityWhileHeld,
                        "Disable Gravity While Held",
                        "Disables gravity for dynamic objects while being held.")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_tidalLock,
                        "Tidal Lock Grabbed Object",
                        "Determines whether a Grabbed Object is tidal locked while being held. This means that the object will always "
                        "face the Grabbing Entity in it's current relative rotation.")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Scaling Factors")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_massIndependentThrow,
                        "Enable Mass Independent Throw",
                        "When enabled, throw impulse is scaled by object mass for consistent throw velocity regardless of mass "
                        "(mass-independent). Disable for realistic mass-dependent throws where heavier objects fly slower/shorter.")
                    ->DataElement(
                        nullptr, &ObjectInteractionComponent::m_throwImpulse, "Throw Impulse", "Linear Impulse scale applied when throwing grabbed object")
                    ->DataElement(
                        nullptr, &ObjectInteractionComponent::m_grabResponse, "Grab Response", "Linear velocity scale applied when holding grabbed object")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_kinematicYawRotateScale,
                        "Kinematic Horizontal Rotate Scale",
                        "Rotation speed scale for horizontal (yaw) rotation of kinematic objects")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_kinematicPitchRotateScale,
                        "Kinematic Vertical Rotate Scale",
                        "Rotation speed scale for vertical (pitch) rotation of kinematic objects")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_dynamicYawRotateScale,
                        "Dynamic Horizontal Rotate Scale",
                        "Angular velocity scale for horizontal (yaw) rotation of dynamic objects")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_dynamicPitchRotateScale,
                        "Dynamic Vertical Rotate Scale",
                        "Angular velocity scale for vertical (pitch) rotation of dynamic objects")
                    ->DataElement(
                        nullptr, &ObjectInteractionComponent::m_tempObjectAngularDamping, "Angular Damping", "Angular Damping of Grabbed Object while Rotating")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_tempObjectLinearDamping,
                        "Linear Damping", "Linear Damping of Grabbed Object while Grabbing")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_velocityCompensation,
                        "Velocity Compensation",
                        "Determines whether to compensate for velocity changes in grabbing entity. Enabling this will keep grab distance "
                        "the same whether you are walking, sprinting, or standing still.")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_velocityCompDampRate,
                        "Velocity Compensation Damp Rate",
                        "Gradually increase velocity compensation for dynamic object. Velocity compensation must be enabled for this to take effect.")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_smoothDynamicRotation,
                        "Smooth Dynamic Rotation",
                        "Enables smooth rotation for dynamic objects. Angular velocity is dampened and interpolated for smooth rotations.")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_angularVelocityDampRate,
                        "Angular Velocity Damp Rate",
                        "Gradually increase angular velocity for dynamic object. Smooth dynamic rotation must be enabled for this to take effect.")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Sphere Cast Parameters")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_grabbedCollisionGroupId,
                        "Sphere Cast Collision Group",
                        "The collision group which will be used for detecting grabbable objects.")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_tempGrabbedCollisionLayer,
                        "Grabbed Object Temporary Collision Layer",
                        "The temporary collision layer assigned to the grabbed object while it is being grabbed/held.")
                    ->DataElement(nullptr, &ObjectInteractionComponent::m_sphereCastRadius, "Sphere Cast Radius", "Sphere Cast radius used for grabbing objects")
                    ->DataElement(
                        nullptr, &ObjectInteractionComponent::m_sphereCastDistance, "Sphere Cast Distance", "Sphere Cast distance along m_sphereCastDirection")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Grab Distance Parameters")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_initialGrabDistance,
                        "Default Grab Distance",
                        "Distance the grabbed object will default to when letting go of the object")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_minGrabDistance,
                        "Min Grab Distance",
                        "Minimum allowable grab distance. Grabbed object cannot get closer than this distance.")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_maxGrabDistance,
                        "Max Grab Distance",
                        "Maximum allowable grab distance. Grabbed object cannot get further than this distance.")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_grabDistanceSpeed,
                        "Grab Distance Speed",
                        "The speed at which you move the grabbed object closer or away.")
                        
                    ->ClassElement(AZ::Edit::ClassElements::Group, "Advanced Held Dynamics")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_enablePIDHeldDynamics,
                        "Enable PID Held Dynamics",
                        "Enables PID controller for dynamic held objects, creating spring-like (underdamped/overdamped) motion. Disabling "
                        "uses simple linear velocity.")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_massIndependentPID,
                        "Enable Mass Independent PID",
                        "When enabled, PID controller scales forces by object mass for consistent behavior regardless of mass "
                        "(mass-independent). "
                        "Disable for realistic mass-dependent motion where heavier objects feel slower and harder to move.")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_proportionalGain,
                        "PID P Gain",
                        "Proportional gain: Controls stiffness/pull strength (higher = stronger spring).")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_integralGain,
                        "PID I Gain",
                        "Integral gain: Corrects persistent errors (e.g., gravity offset; usually low or 0).")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_derivativeGain,
                        "PID D Gain",
                        "Derivative gain: Controls damping (low = underdamped/oscillatory; high = overdamped/slow).")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_integralWindupLimit,
                        "PID Integral Limit",
                        "Anti-windup limit for integral accumulation (higher = stronger I but riskier).")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_derivFilterAlpha,
                        "PID Deriv Filter Alpha",
                        "Derivative filter strength (0=raw, 1=heavy smoothing; 0.7 for responsive with light noise reduction).")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Tidal Lock Dynamics")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_enablePIDTidalLockDynamics,
                        "Enable PID Tidal Lock Dynamics",
                        "Enables PID controller for dynamic tidal lock rotation, creating spring-like motion. Disabling "
                        "uses simple angular velocity.")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_massIndependentTidalLock,
                        "Enable Mass Independent Tidal Lock",
                        "When enabled and PID is active, scales torque by object mass for consistent rotation regardless of mass. "
                        "Disable for realistic mass-dependent rotation where heavier objects rotate slower.")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_tidalLockProportionalGain,
                        "Tidal Lock PID P Gain",
                        "Proportional gain for tidal lock: Controls rotational stiffness (higher = faster facing).")
                    ->Attribute(AZ::Edit::Attributes::Suffix, " N m / rad")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_tidalLockIntegralGain,
                        "Tidal Lock PID I Gain",
                        "Integral gain: Corrects persistent rotational errors (usually low or 0).")
                    ->Attribute(AZ::Edit::Attributes::Suffix, " N m / (rad s)")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_tidalLockDerivativeGain,
                        "Tidal Lock PID D Gain",
                        "Derivative gain: Controls rotational damping (higher = less oscillation).")
                    ->Attribute(AZ::Edit::Attributes::Suffix, " N m s / rad")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_tidalLockIntegralWindupLimit,
                        "Tidal Lock PID Integral Limit",
                        "Anti-windup limit for integral (higher = stronger I).")
                    ->Attribute(AZ::Edit::Attributes::Suffix, " N m")
                    ->DataElement(
                        nullptr,
                        &ObjectInteractionComponent::m_tidalLockDerivFilterAlpha,
                        "Tidal Lock PID Deriv Filter Alpha",
                        "Derivative filter (0=raw, 1=heavy smoothing).");
            }
        }

        if (auto bc = azrtti_cast<AZ::BehaviorContext*>(rc))
        {
            bc->EBus<ObjectInteractionNotificationBus>("GrabNotificationBus", "GrabComponentNotificationBus", "Notifications for Grab Component")
                ->Handler<ObjectInteractionNotificationHandler>();

            // Reflect the enum
            bc->Enum<static_cast<int>(ObjectInteractionStates::idleState)>("ObjectInteractionStates_IdleState")
                ->Enum<static_cast<int>(ObjectInteractionStates::checkState)>("ObjectInteractionStates_CheckState")
                ->Enum<static_cast<int>(ObjectInteractionStates::holdState)>("ObjectInteractionStates_HoldState")
                ->Enum<static_cast<int>(ObjectInteractionStates::rotateState)>("ObjectInteractionStates_RotateState")
                ->Enum<static_cast<int>(ObjectInteractionStates::throwState)>("ObjectInteractionStates_ThrowState");

            bc->EBus<ObjectInteractionComponentRequestBus>("ObjectInteractionComponentRequestBus")
                ->Attribute(AZ::Script::Attributes::Scope, AZ::Script::Attributes::ScopeFlags::Common)
                ->Attribute(AZ::Script::Attributes::Module, "interaction")
                ->Attribute(AZ::Script::Attributes::Category, "Object Interaction")
                ->Event("Get Grabbing EntityId", &ObjectInteractionComponentRequests::GetGrabbingEntityId)
                ->Event("Get Active Camera EntityId", &ObjectInteractionComponentRequests::GetActiveCameraEntityId)
                ->Event("Get Grabbed Object EntityId", &ObjectInteractionComponentRequests::GetGrabbedObjectEntityId)
                ->Event("Get Last Grabbed Object EntityId", &ObjectInteractionComponentRequests::GetLastGrabbedObjectEntityId)
                ->Event("Get Thrown Grabbed Object EntityId", &ObjectInteractionComponentRequests::GetThrownGrabbedObjectEntityId)
                ->Event("Set Thrown Grabbed Object EntityId", &ObjectInteractionComponentRequests::SetThrownGrabbedObjectEntityId)
                ->Event("Set Grabbing Entity", &ObjectInteractionComponentRequests::SetGrabbingEntity)
                ->Event("Get Grabbed Collision Group", &ObjectInteractionComponentRequests::GetGrabbedCollisionGroup)
                ->Event("Set Grabbed Collision Group", &ObjectInteractionComponentRequests::SetGrabbedCollisionGroup)
                ->Event("Get Current Grabbed Layer Name", &ObjectInteractionComponentRequests::GetCurrentGrabbedCollisionLayerName)
                ->Event("Set Current Grabbed Layer By Name", &ObjectInteractionComponentRequests::SetCurrentGrabbedCollisionLayerByName)
                ->Event("Get Current Grabbed Layer", &ObjectInteractionComponentRequests::GetCurrentGrabbedCollisionLayer)
                ->Event("Set Current Grabbed Layer", &ObjectInteractionComponentRequests::SetCurrentGrabbedCollisionLayer)
                ->Event("Get Previous Grabbed Layer Name", &ObjectInteractionComponentRequests::GetPrevGrabbedCollisionLayerName)
                ->Event("Set Previous Grabbed Layer Name By Name", &ObjectInteractionComponentRequests::SetPrevGrabbedCollisionLayerByName)
                ->Event("Get Previous Grabbed Layer", &ObjectInteractionComponentRequests::GetPrevGrabbedCollisionLayer)
                ->Event("Set Previous Grabbed Layer", &ObjectInteractionComponentRequests::SetPrevGrabbedCollisionLayer)
                ->Event("Get Temporary Grabbed Layer Name", &ObjectInteractionComponentRequests::GetTempGrabbedCollisionLayerName)
                ->Event("Set Temporary Grabbed Layer By Name", &ObjectInteractionComponentRequests::SetTempGrabbedCollisionLayerByName)
                ->Event("Get Temporary Grabbed Layer", &ObjectInteractionComponentRequests::GetTempGrabbedCollisionLayer)
                ->Event("Set Temporary Grabbed Layer", &ObjectInteractionComponentRequests::SetTempGrabbedCollisionLayer)
                ->Event("Get State String", &ObjectInteractionComponentRequests::GetStateString)
                ->Event("Get Is In Idle State", &ObjectInteractionComponentRequests::GetIsInIdleState)
                ->Event("Get Is In Check State", &ObjectInteractionComponentRequests::GetIsInCheckState)
                ->Event("Get Is In Held State", &ObjectInteractionComponentRequests::GetIsInHeldState)
                ->Event("Get Is In Rotate State", &ObjectInteractionComponentRequests::GetIsInRotateState)
                ->Event("Get Is In Throw State", &ObjectInteractionComponentRequests::GetIsInThrowState)
                ->Event("Get Object Sphere Cast Hit", &ObjectInteractionComponentRequests::GetObjectSphereCastHit)
                ->Event("Get Stay In Idle State", &ObjectInteractionComponentRequests::GetStayInIdleState)
                ->Event("Set Stay In Idle State", &ObjectInteractionComponentRequests::SetStayInIdleState)
                ->Event("Get Grab Enable Toggle", &ObjectInteractionComponentRequests::GetGrabEnableToggle)
                ->Event("Set Grab Enable Toggle", &ObjectInteractionComponentRequests::SetGrabEnableToggle)
                ->Event("Get Rotate Enable Toggle", &ObjectInteractionComponentRequests::GetRotateEnableToggle)
                ->Event("Set Rotate Enable Toggle", &ObjectInteractionComponentRequests::SetRotateEnableToggle)
                ->Event("Get Grab Key Value", &ObjectInteractionComponentRequests::GetGrabKeyValue)
                ->Event("Set Grab Key Value", &ObjectInteractionComponentRequests::SetGrabKeyValue)
                ->Event("Get Throw Key Value", &ObjectInteractionComponentRequests::GetThrowKeyValue)
                ->Event("Set Throw Key Value", &ObjectInteractionComponentRequests::SetThrowKeyValue)
                ->Event("Get Rotate Key Value", &ObjectInteractionComponentRequests::GetRotateKeyValue)
                ->Event("Set Rotate Key Value", &ObjectInteractionComponentRequests::SetRotateKeyValue)
                ->Event("Get Pitch Key Value", &ObjectInteractionComponentRequests::GetPitchKeyValue)
                ->Event("Set Pitch Key Value", &ObjectInteractionComponentRequests::SetPitchKeyValue)
                ->Event("Get Yaw Key Value", &ObjectInteractionComponentRequests::GetYawKeyValue)
                ->Event("Set Yaw Key Value", &ObjectInteractionComponentRequests::SetYawKeyValue)
                ->Event("Get Roll Key Value", &ObjectInteractionComponentRequests::GetRollKeyValue)
                ->Event("Set Roll Key Value", &ObjectInteractionComponentRequests::SetRollKeyValue)
                ->Event("Get Grab Distance Key Value", &ObjectInteractionComponentRequests::GetGrabbedDistanceKeyValue)
                ->Event("Set Grab Distance Key Value", &ObjectInteractionComponentRequests::SetGrabbedDistanceKeyValue)         
                ->Event("Get Grabbed Object Distance", &ObjectInteractionComponentRequests::GetGrabbedObjectDistance)
                ->Event("Set Grabbed Object Distance", &ObjectInteractionComponentRequests::SetGrabbedObjectDistance)
                ->Event("Get Minimum Grabbed Object Distance", &ObjectInteractionComponentRequests::GetMinGrabbedObjectDistance)
                ->Event("Set Minimum Grabbed Object Distance", &ObjectInteractionComponentRequests::SetMinGrabbedObjectDistance)
                ->Event("Get Maximum Grabbed Objectt Distance", &ObjectInteractionComponentRequests::GetMaxGrabbedObjectDistance)
                ->Event("Set Maximum Grabbed Object Distance", &ObjectInteractionComponentRequests::SetMaxGrabbedObjectDistance)
                ->Event("Get Initial Grabbed Object Distance", &ObjectInteractionComponentRequests::GetInitialGrabbedObjectDistance)
                ->Event("Set Initial Grabbed Objectt Distance", &ObjectInteractionComponentRequests::SetInitialGrabbedObjectDistance)
                ->Event("Get Grabbed Object Distance Speed", &ObjectInteractionComponentRequests::GetGrabbedObjectDistanceSpeed)
                ->Event("Set Grabbed Object Distance Speed", &ObjectInteractionComponentRequests::SetGrabbedObjectDistanceSpeed)
                ->Event("Get Grab Response", &ObjectInteractionComponentRequests::GetGrabResponse)
                ->Event("Set Grab Response", &ObjectInteractionComponentRequests::SetGrabResponse)
                ->Event("Get Dynamic Object Tidal Lock", &ObjectInteractionComponentRequests::GetDynamicTidalLock)
                ->Event("Set Dynamic Object Tidal Lock", &ObjectInteractionComponentRequests::SetDynamicTidalLock)
                ->Event("Get Kinematic Object Tidal Lock", &ObjectInteractionComponentRequests::GetKinematicTidalLock)
                ->Event("Set Kinematic Object Tidal Lock", &ObjectInteractionComponentRequests::SetKinematicTidalLock)
                ->Event("Get Object Tidal Lock", &ObjectInteractionComponentRequests::GetTidalLock)
                ->Event("Set Object Tidal Lock", &ObjectInteractionComponentRequests::SetTidalLock)
                ->Event("Get Dynamic Horizontal Rotate Scale", &ObjectInteractionComponentRequests::GetDynamicYawRotateScale)
                ->Event("Set Dynamic Horizontal Rotate Scale", &ObjectInteractionComponentRequests::SetDynamicYawRotateScale)
                ->Event("Get Dynamic Vertical Rotate Scale", &ObjectInteractionComponentRequests::GetDynamicPitchRotateScale)
                ->Event("Set Dynamic Vertical Rotate Scale", &ObjectInteractionComponentRequests::SetDynamicPitchRotateScale)
                ->Event("Get Kinematic Horizontal Rotate Scale", &ObjectInteractionComponentRequests::GetKinematicYawRotateScale)
                ->Event("Set Kinematic Horizontal Rotate Scale", &ObjectInteractionComponentRequests::SetKinematicYawRotateScale)
                ->Event("Get Kinematic Vertical Rotate Scale", &ObjectInteractionComponentRequests::GetKinematicPitchRotateScale)
                ->Event("Set Kinematic Vertical Rotate Scale", &ObjectInteractionComponentRequests::SetKinematicPitchRotateScale)
                ->Event("Get Velocity Compensation Damp Rate", &ObjectInteractionComponentRequests::GetVelocityCompDampRate)
                ->Event("Set Velocity Compensation Damp Rate", &ObjectInteractionComponentRequests::SetVelocityCompDampRate)
                ->Event("Get Angular Velocity Damp Rate", &ObjectInteractionComponentRequests::GetAngularVelocityDampRate)
                ->Event("Set Angular Velocity Damp Rate", &ObjectInteractionComponentRequests::SetAngularVelocityDampRate)
                ->Event("Get Velocity Compensation", &ObjectInteractionComponentRequests::GetVelocityCompensation)
                ->Event("Set Velocity Compensation", &ObjectInteractionComponentRequests::SetVelocityCompensation)
                ->Event("Get Smooth Dynamic Rotation", &ObjectInteractionComponentRequests::GetSmoothDynamicRotation)
                ->Event("Set Smooth Dynamic Rotation", &ObjectInteractionComponentRequests::SetSmoothDynamicRotation)
                ->Event("Get Grab Throw Impulse", &ObjectInteractionComponentRequests::GetThrowImpulse)
                ->Event("Set Grab Throw Impulse", &ObjectInteractionComponentRequests::SetThrowImpulse)
                ->Event("Get Grabbed Object Throw State Counter", &ObjectInteractionComponentRequests::GetGrabbedObjectThrowStateCounter)
                ->Event("Set Grabbed Object Throw State Counter", &ObjectInteractionComponentRequests::SetGrabbedObjectThrowStateCounter)
                ->Event("Get Grabbed Object Throw State Max Time", &ObjectInteractionComponentRequests::GetGrabbedObjectThrowStateTime)
                ->Event("Set Grabbed Object Throw State Max Time", &ObjectInteractionComponentRequests::SetGrabbedObjectThrowStateTime)
                ->Event("Get Grab Sphere Cast Radius", &ObjectInteractionComponentRequests::GetSphereCastRadius)
                ->Event("Set Grab Sphere Cast Radius", &ObjectInteractionComponentRequests::SetSphereCastRadius)
                ->Event("Get Grab Sphere Cast Distance", &ObjectInteractionComponentRequests::GetSphereCastDistance)
                ->Event("Set Grab Sphere Cast Distance", &ObjectInteractionComponentRequests::SetSphereCastDistance)
                ->Event("Get Grabbed Object Is Kinematic", &ObjectInteractionComponentRequests::GetGrabbedObjectKinematicElseDynamic)
                ->Event("Set Grabbed Object Is Kinematic", &ObjectInteractionComponentRequests::SetGrabbedObjectKinematicElseDynamic)
                ->Event("Get Current Grabbed Object Angular Damping", &ObjectInteractionComponentRequests::GetCurrentGrabbedObjectAngularDamping)
                ->Event("Set Current Grabbed Object Angular Damping", &ObjectInteractionComponentRequests::SetCurrentGrabbedObjectAngularDamping)
                ->Event("Get Previous Grabbed Object Angular Damping", &ObjectInteractionComponentRequests::GetPrevGrabbedObjectAngularDamping)
                ->Event("Set Previous Grabbed Object Angular Damping", &ObjectInteractionComponentRequests::SetPrevGrabbedObjectAngularDamping)
                ->Event("Get Temporary Grabbed Object Angular Damping", &ObjectInteractionComponentRequests::GetTempGrabbedObjectAngularDamping)
                ->Event("Set Temporary Grabbed Object Angular Damping", &ObjectInteractionComponentRequests::SetTempGrabbedObjectAngularDamping)
                ->Event("Get Current Grabbed Object Linear Damping", &ObjectInteractionComponentRequests::GetCurrentGrabbedObjectLinearDamping)
                ->Event("Set Current Grabbed Object Linear Damping", &ObjectInteractionComponentRequests::SetCurrentGrabbedObjectLinearDamping)
                ->Event("Get Previous Grabbed Object Linear Damping", &ObjectInteractionComponentRequests::GetPrevGrabbedObjectLinearDamping)
                ->Event("Set Previous Grabbed Object Linear Damping", &ObjectInteractionComponentRequests::SetPrevGrabbedObjectLinearDamping)
                ->Event("Get Temporary Grabbed Object Linear Damping", &ObjectInteractionComponentRequests::GetTempGrabbedObjectLinearDamping)
                ->Event("Set Temporary Grabbed Object Linear Damping", &ObjectInteractionComponentRequests::SetTempGrabbedObjectLinearDamping)
                ->Event("Get Initial Angular Velocity Zero", &ObjectInteractionComponentRequests::GetInitialAngularVelocityZero)
                ->Event("Set Initial Angular Velocity Zero", &ObjectInteractionComponentRequests::SetInitialAngularVelocityZero)
                ->Event("Force State Transition", &ObjectInteractionComponentRequests::ForceTransition)
                ->Event("Set Locked State Transition", &ObjectInteractionComponentRequests::SetStateLocked)
                ->Event("Get Locked State Transition", &ObjectInteractionComponentRequests::GetStateLocked)
                ->Event("GetGrabInputKey", &ObjectInteractionComponentRequests::GetGrabInputKey)
                ->Event("SetGrabInputKey", &ObjectInteractionComponentRequests::SetGrabInputKey)
                ->Event("GetThrowInputKey", &ObjectInteractionComponentRequests::GetThrowInputKey)
                ->Event("SetThrowInputKey", &ObjectInteractionComponentRequests::SetThrowInputKey)
                ->Event("GetRotateInputKey", &ObjectInteractionComponentRequests::GetRotateInputKey)
                ->Event("SetRotateInputKey", &ObjectInteractionComponentRequests::SetRotateInputKey)
                ->Event("GetRotatePitchInputKey", &ObjectInteractionComponentRequests::GetRotatePitchInputKey)
                ->Event("SetRotatePitchInputKey", &ObjectInteractionComponentRequests::SetRotatePitchInputKey)
                ->Event("GetRotateYawInputKey", &ObjectInteractionComponentRequests::GetRotateYawInputKey)
                ->Event("SetRotateYawInputKey", &ObjectInteractionComponentRequests::SetRotateYawInputKey)
                ->Event("GetRotateRollInputKey", &ObjectInteractionComponentRequests::GetRotateRollInputKey)
                ->Event("SetRotateRollInputKey", &ObjectInteractionComponentRequests::SetRotateRollInputKey)
                ->Event("GetGrabDistanceInputKey", &ObjectInteractionComponentRequests::GetGrabDistanceInputKey)
                ->Event("SetGrabDistanceInputKey", &ObjectInteractionComponentRequests::SetGrabDistanceInputKey)
                ->Event("GetMeshEntityId", &ObjectInteractionComponentRequests::GetMeshEntityId)
                ->Event("SetMeshEntityId", &ObjectInteractionComponentRequests::SetMeshEntityId)
                ->Event("GetMeshEntityName", &ObjectInteractionComponentRequests::GetMeshEntityName)
                ->Event("SetMeshEntityName", &ObjectInteractionComponentRequests::SetMeshEntityName);

            bc->Class<ObjectInteractionComponent>()->RequestBus("ObjectInteractionComponentRequestBus");
        }
    }

    void ObjectInteractionComponent::Activate()
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
            AZ_Error("Object Interaction Component", false, "Failed to retrieve default scene.");
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
        ObjectInteractionComponentRequestBus::Handler::BusConnect(GetEntityId());

        // Delaying the assignment of Grabbing Entity to OnEntityActivated so the Entity is activated and ready
        AZ::EntityBus::Handler::BusConnect(m_grabbingEntityId);

        // Initialize m_grabDistance to the editor-specified m_initialGrabDistance
        m_grabDistance = m_initialGrabDistance;

        // Initialize PID controller with parameters
        m_pidController = PidController<AZ::Vector3>(
            m_proportionalGain, m_integralGain, m_derivativeGain, m_integralWindupLimit, m_derivFilterAlpha, m_derivativeMode);

        // Initialize angular PID controller with ErrorRate mode for better feedforward
        m_tidalLockPidController = PidController<AZ::Vector3>(
            m_tidalLockProportionalGain,
            m_tidalLockIntegralGain,
            m_tidalLockDerivativeGain,
            m_tidalLockIntegralWindupLimit,
            m_tidalLockDerivFilterAlpha,
            PidController<AZ::Vector3>::ErrorRate);
    }

    // Called at the beginning of each physics tick
    void ObjectInteractionComponent::OnSceneSimulationStart(float physicsTimestep)
    {
        // Update physics timestep
        m_physicsTimestep = physicsTimestep;

        if (!m_isObjectKinematic && (m_state == ObjectInteractionStates::holdState || m_state == ObjectInteractionStates::rotateState))
        {
            ProcessStates(physicsTimestep, true);
        }

        // Reset time accumulator
        m_physicsTimeAccumulator = 0.0f;
    }

    void ObjectInteractionComponent::OnSceneSimulationFinish(
        [[maybe_unused]] AzPhysics::SceneHandle sceneHandle, [[maybe_unused]] float fixedDeltaTime)
    {
        if (m_lastGrabbedObjectEntityId.IsValid() && !m_isObjectKinematic && m_meshSmoothing &&
            (m_state == ObjectInteractionStates::holdState || m_state == ObjectInteractionStates::rotateState))
        {
            m_prevPhysicsTransform = m_currentPhysicsTransform;
            AZ::TransformBus::EventResult(m_currentPhysicsTransform, m_lastGrabbedObjectEntityId, &AZ::TransformInterface::GetWorldTM);
        }
    }

    void ObjectInteractionComponent::OnEntityActivated([[maybe_unused]] const AZ::EntityId& entityId)
    {
        AZ::EntityBus::Handler::BusDisconnect();

        if (m_grabbingEntityId.IsValid())
        {
            m_grabbingEntityPtr = GetEntityPtr(entityId);
        }
    }

    void ObjectInteractionComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        InputEventNotificationBus::MultiHandler::BusDisconnect();
        ObjectInteractionComponentRequestBus::Handler::BusDisconnect();

        m_attachedSceneHandle = AzPhysics::InvalidSceneHandle;
        m_sceneSimulationStartHandler.Disconnect();
        m_sceneSimulationFinishHandler.Disconnect();
        m_meshEntityPtr = nullptr;
    }

    void ObjectInteractionComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("InputConfigurationService"));
        required.push_back(AZ_CRC_CE("TransformService"));
    }

    void ObjectInteractionComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("GrabService"));
    }

    void ObjectInteractionComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("GrabService"));
        incompatible.push_back(AZ_CRC_CE("InputService"));
    }

    void ObjectInteractionComponent::GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        dependent.push_back(AZ_CRC_CE("FirstPersonControllerService"));
    }

    AZ::Entity* ObjectInteractionComponent::GetActiveCameraEntityPtr() const
    {
        AZ::EntityId activeCameraId;
        Camera::CameraSystemRequestBus::BroadcastResult(activeCameraId, &Camera::CameraSystemRequestBus::Events::GetActiveCamera);

        auto ca = AZ::Interface<AZ::ComponentApplicationRequests>::Get();
        return ca->FindEntity(activeCameraId);
    }

    // Recieve the input event in OnPressed method
    void ObjectInteractionComponent::OnPressed(float value)
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
    void ObjectInteractionComponent::OnReleased(float value)
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

    void ObjectInteractionComponent::OnHeld(float value)
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

    void ObjectInteractionComponent::ProcessStates(const float& deltaTime, bool isPhysicsUpdate)
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
            case ObjectInteractionStates::holdState:
                HoldObjectState(deltaTime, isPhysicsUpdate);
                break;
            case ObjectInteractionStates::rotateState:
                RotateObjectState(deltaTime, isPhysicsUpdate);
                break;
            default:
                return;
            }
            return;
        }

        switch(m_state)
        {
            case ObjectInteractionStates::idleState:
                IdleState();
                break;
            case ObjectInteractionStates::checkState:
                CheckForObjectsState();
                break;
            case ObjectInteractionStates::holdState:
                HoldObjectState(deltaTime);
                break;
            case ObjectInteractionStates::rotateState:
                RotateObjectState(deltaTime);
                break;
            case ObjectInteractionStates::throwState:
                ThrowObjectState(deltaTime);
                break;
            default:
                m_state = ObjectInteractionStates::idleState;
                IdleState();
        }

        m_prevGrabKeyValue = m_grabKeyValue;
        m_prevRotateKeyValue = m_rotateKeyValue;
    }

    void ObjectInteractionComponent::OnTick(float deltaTime, AZ::ScriptTimePoint)
    {
        ProcessStates(deltaTime);
        if (m_meshSmoothing)
        {
            InterpolateMeshTransform(deltaTime);
        }
    }
    
    // Smoothly update the visual transform of m_meshEntityPtr based on physics transforms
    void ObjectInteractionComponent::InterpolateMeshTransform(float deltaTime)
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

        // Create interpolated transform
        const AZ::Transform interpolatedTransform =
            AZ::Transform::CreateFromQuaternionAndTranslation(interpolatedRotation, interpolatedPosition);

        // Update mesh entity transform
        AZ::TransformBus::Event(m_meshEntityPtr->GetId(), &AZ::TransformInterface::SetWorldTM, interpolatedTransform);

        // Reset accumulator if it exceeds physics timestep due to accumulated error
        if (m_physicsTimeAccumulator >= m_physicsTimestep)
            m_physicsTimeAccumulator -= m_physicsTimestep;
    }

    void ObjectInteractionComponent::IdleState()
    {
        if ((m_forceTransition && m_targetState == ObjectInteractionStates::checkState) ||
            (!m_isStateLocked && m_prevGrabKeyValue == 0.f && m_grabKeyValue != 0.f && !m_stayInIdleState))
        {
            m_state = ObjectInteractionStates::checkState;
            m_forceTransition = false;
        }
    }

    void ObjectInteractionComponent::CheckForObjectsState()
    {
        CheckForObjects();
        // Check if sphere cast hits a valid object before transitioning to holdState.
        // Other conditionals allow forced state transition to bypass inputs with m_forceTransition, or 
        // prevent state transition with m_isStateLocked
        if ((m_forceTransition && m_targetState == ObjectInteractionStates::holdState && m_objectSphereCastHit) ||
            (!m_isStateLocked && m_objectSphereCastHit))
        {
            // Check if Grabbed Object is a Dynamic Rigid Body when first interacting with it
            m_isInitialObjectKinematic = GetGrabbedObjectKinematicElseDynamic();
            
            // Store initial collision layer
            m_prevGrabbedCollisionLayer = GetCurrentGrabbedCollisionLayer();
            
            // Set Object Current Layer variable to Temp Layer
            SetCurrentGrabbedCollisionLayer(m_tempGrabbedCollisionLayer);

            // Set Grabbed Object as Kinematic Rigid Body if set to be kinematic while held
            if (m_kinematicWhileHeld)
            {
                SetGrabbedObjectKinematicElseDynamic(true);
                m_isObjectKinematic = true;
            }
            // Set Grabbed Object as Dynamic Rigid Body if set to be dynamic while held
            else
            {
                SetGrabbedObjectKinematicElseDynamic(false);
                m_isObjectKinematic = false;
            }
           
            // Store object's original Angular Damping value
            m_prevObjectAngularDamping = GetCurrentGrabbedObjectAngularDamping();

            // Store object's original Linear Damping value, and set new value.
            m_prevObjectLinearDamping = GetCurrentGrabbedObjectLinearDamping();
            SetCurrentGrabbedObjectLinearDamping(m_tempObjectLinearDamping);

            // Store and disable gravity for dynamic objects if enabled
            if (m_disableGravityWhileHeld && !m_isObjectKinematic)
            {
                Physics::RigidBodyRequestBus::EventResult(
                    m_prevGravityEnabled, m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::IsGravityEnabled);
                Physics::RigidBodyRequestBus::Event(m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::SetGravityEnabled, false);
            }

            // Initialize physics transforms for dynamic objects
            if (!m_isObjectKinematic && m_meshSmoothing)
            {
                AZ::TransformBus::EventResult(m_prevPhysicsTransform, m_lastGrabbedObjectEntityId, &AZ::TransformInterface::GetWorldTM);
                m_currentPhysicsTransform = m_prevPhysicsTransform;
                m_physicsTimeAccumulator = 0.0f;
            }

            // Find child entity with name containing m_meshEntityName
            m_meshEntityPtr = nullptr;
            if (m_meshSmoothing && !m_isObjectKinematic)
            {
                // Prioritize m_meshEntityId if valid
                if (m_meshEntityId.IsValid())
                {
                    m_meshEntityPtr = GetEntityPtr(m_meshEntityId);
                }
                // Fallback to name-based lookup if m_meshEntityId is invalid and m_meshEntityName is set
                else if (!m_meshEntityName.empty())
                {
                    AZ::Entity* grabbedEntity = GetEntityPtr(m_lastGrabbedObjectEntityId);
                    if (grabbedEntity)
                    {
                        AZStd::vector<AZ::EntityId> children;
                        AZ::TransformBus::EventResult(children, m_lastGrabbedObjectEntityId, &AZ::TransformInterface::GetChildren);
                        for (const AZ::EntityId& childId : children)
                        {
                            AZ::Entity* childEntity = GetEntityPtr(childId);
                            if (childEntity && childEntity->GetName().find(m_meshEntityName) != AZStd::string::npos)
                            {
                                m_meshEntityPtr = childEntity;
                                break;
                            }
                        }
                    }
                }
            }

            // Fallback to grabbed object if no matching child found or smoothing disabled
            if (!m_meshEntityPtr)
            {
                m_meshEntityPtr = GetEntityPtr(m_lastGrabbedObjectEntityId);
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
            if (m_useFPControllerForGrab)
            {
                grabbingEntityRotationQuat = GetEntity()->GetTransform()->GetWorldRotationQuaternion();
            }
            else
            {
                grabbingEntityRotationQuat = m_grabbingEntityPtr->GetTransform()->GetWorldRotationQuaternion();
            }
            AZ::Quaternion grabbedObjectRotationQuat;
            AZ::TransformBus::EventResult(
                grabbedObjectRotationQuat, m_lastGrabbedObjectEntityId, &AZ::TransformInterface::GetWorldRotationQuaternion);
            m_grabbedObjectRelativeQuat = grabbingEntityRotationQuat.GetInverseFull() * grabbedObjectRotationQuat;

            m_state = ObjectInteractionStates::holdState;
            // Broadcast a grab start notification event
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnHoldStart);

            #ifdef FIRST_PERSON_CONTROLLER
            if (m_useFPControllerForGrab)
            {
                m_lastEntityRotationQuat = GetEntity()->GetTransform()->GetWorldRotationQuaternion();
            }
            #endif
            else
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
            (m_forceTransition && m_targetState == ObjectInteractionStates::idleState) ||
            (!m_isStateLocked && !(m_prevGrabKeyValue == 0.f && m_grabKeyValue != 0.f)))
        {
            m_state = ObjectInteractionStates::idleState;
            m_forceTransition = false;
        }
        else
        {
            m_state = ObjectInteractionStates::checkState;
        }
    }

    void ObjectInteractionComponent::HoldObjectState(float deltaTime, bool isPhysicsUpdate)
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
        if ((m_forceTransition && m_targetState == ObjectInteractionStates::idleState) || 
            (!m_isStateLocked && !m_objectSphereCastHit))
        {
            // Reset current grabbed distance to m_initialGrabDistance if sphere cast doesn't hit
            m_grabDistance = m_initialGrabDistance;

            // Set Object Current Layer variable back to initial layer if sphere cast doesn't hit
            SetCurrentGrabbedCollisionLayer(m_prevGrabbedCollisionLayer);

            // Set Angular Damping back to original value if sphere cast doesn't hit
            SetCurrentGrabbedObjectAngularDamping(m_prevObjectAngularDamping);

            // Restore gravity if it was disabled
            if (m_disableGravityWhileHeld && !m_isObjectKinematic)
            {
                Physics::RigidBodyRequestBus::Event(
                    m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::SetGravityEnabled, m_prevGravityEnabled);
            }

            // Set Grabbed Object back to Dynamic Rigid Body if previously dynamic
            if (!m_isInitialObjectKinematic)
            {
                SetGrabbedObjectKinematicElseDynamic(false);
                m_isObjectKinematic = false;
            }
            // Set Grabbed Object back to Kinematic Rigid Body if previously kinematic
            else if (m_isInitialObjectKinematic)
            {
                SetGrabbedObjectKinematicElseDynamic(true);
                m_isObjectKinematic = true;
            }

            // Reset to object's original Linear Damping value
            SetCurrentGrabbedObjectLinearDamping(m_prevObjectLinearDamping);

            m_objectSphereCastHit = false;
            m_grabbedObjectEntityId = AZ::EntityId();
            m_lastGrabbedObjectEntityId = AZ::EntityId();
            m_meshEntityPtr = nullptr;

            m_state = ObjectInteractionStates::idleState;
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnHoldStop);
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
        if ((m_forceTransition && m_targetState == ObjectInteractionStates::idleState) ||
            (!m_isStateLocked &&
             ((m_grabEnableToggle && m_prevGrabKeyValue == 0.f && m_grabKeyValue != 0.f) ||
              (!m_grabEnableToggle && m_grabKeyValue == 0.f))))
        {
            // Reset current grabbed distance to m_initialGrabDistance if grab key is not pressed
            m_grabDistance = m_initialGrabDistance;

            // Set Object Current Layer variable back to initial layer if Grab Key is not pressed
            SetCurrentGrabbedCollisionLayer(m_prevGrabbedCollisionLayer);

            // Set Angular Damping back to original value if Grab Key is not pressed
            SetCurrentGrabbedObjectAngularDamping(m_prevObjectAngularDamping);

            // Restore gravity if it was disabled
            if (m_disableGravityWhileHeld && !m_isObjectKinematic)
            {
                Physics::RigidBodyRequestBus::Event(
                    m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::SetGravityEnabled, m_prevGravityEnabled);
            }

            // Set Grabbed Object back to Dynamic Rigid Body if previously dynamic
            if (!m_isInitialObjectKinematic)
            {
                SetGrabbedObjectKinematicElseDynamic(false);
                m_isObjectKinematic = false;
            }
            // Set Grabbed Object back to Kinematic Rigid Body if previously kinematic
            else if (m_isInitialObjectKinematic)
            {
                SetGrabbedObjectKinematicElseDynamic(true);
                m_isObjectKinematic = true;
            }

            // Reset to object's original Linear Damping value
            SetCurrentGrabbedObjectLinearDamping(m_prevObjectLinearDamping);

            m_objectSphereCastHit = false;
            m_grabbedObjectEntityId = AZ::EntityId();
            m_lastGrabbedObjectEntityId = AZ::EntityId();
            m_meshEntityPtr = nullptr;

            m_state = ObjectInteractionStates::idleState;
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnHoldStop);
            m_forceTransition = false;
        }
        // Enter Rotate State if rotate key is pressed. Other conditionals allow forced state transition 
        // to bypass inputs with m_forceTransition, or prevent state transition with m_isStateLocked
        else if (
            (m_forceTransition && m_targetState == ObjectInteractionStates::rotateState) ||
            (!m_isStateLocked &&
             ((m_rotateEnableToggle && m_prevRotateKeyValue == 0.f && m_rotateKeyValue != 0.f) ||
              (!m_rotateEnableToggle && m_rotateKeyValue != 0.f))))
        {
            // Store object's original Angular Damping value before rotating
            m_prevObjectAngularDamping = GetCurrentGrabbedObjectAngularDamping();
            // Set new Angular Damping before rotating object
            SetCurrentGrabbedObjectAngularDamping(m_tempObjectAngularDamping);

            m_state = ObjectInteractionStates::rotateState;
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnRotateStart);
            m_forceTransition = false;
        }
        // Enter throw state if throw key is pressed. Other conditionals allow forced state transition to 
        // bypass inputs with m_forceTransition, or prevent state transition with m_isStateLocked
        else if (
            (m_forceTransition && m_targetState == ObjectInteractionStates::throwState && !m_isInitialObjectKinematic) ||
            (!m_isStateLocked && m_throwKeyValue != 0.f && !m_isInitialObjectKinematic))
        {
            // Start throw counter
            m_throwStateCounter = m_throwStateMaxTime;

            // Set Kinematic Rigid Body to dynamic if it was held as kinematic
            if (m_isObjectKinematic)
            {
                SetGrabbedObjectKinematicElseDynamic(false);
                m_isObjectKinematic = false;
            }

            // Reset to object's original Linear Damping value
            SetCurrentGrabbedObjectLinearDamping(m_prevObjectLinearDamping);

            m_objectSphereCastHit = false;
            m_meshEntityPtr = nullptr;

            m_state = ObjectInteractionStates::throwState;
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnHoldStop);
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnThrowStart);
            m_forceTransition = false;
        }
        else
        {
            m_state = ObjectInteractionStates::holdState;
        }
    }

    void ObjectInteractionComponent::RotateObjectState(float deltaTime, bool isPhysicsUpdate)
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
        if ((m_forceTransition && m_targetState == ObjectInteractionStates::idleState) || 
            (!m_isStateLocked && !m_objectSphereCastHit))
        {
            // Set Angular Damping back to original value if sphere cast doesn't hit
            SetCurrentGrabbedObjectAngularDamping(m_prevObjectAngularDamping);
            
            // Set Linear Damping back to zero if sphere cast doesn't hit
            SetCurrentGrabbedObjectLinearDamping(m_prevObjectLinearDamping);
            
            // Set Angular Velocity back to zero if sphere cast doesn't hit
            SetGrabbedObjectAngularVelocity(AZ::Vector3::CreateZero());

            m_grabDistance = m_initialGrabDistance;
            SetCurrentGrabbedCollisionLayer(m_prevGrabbedCollisionLayer);

            // Restore gravity if it was disabled
            if (m_disableGravityWhileHeld && !m_isObjectKinematic)
            {
                Physics::RigidBodyRequestBus::Event(
                    m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::SetGravityEnabled, m_prevGravityEnabled);
            }

            // Set Grabbed Object back to Dynamic Rigid Body if previously dynamic
            if (!m_isInitialObjectKinematic)
            {
                SetGrabbedObjectKinematicElseDynamic(false);
                m_isObjectKinematic = false;
            }
            // Set Grabbed Object back to Kinematic Rigid Body if previously kinematic
            else if (m_isInitialObjectKinematic)
            {
                SetGrabbedObjectKinematicElseDynamic(true);
                m_isObjectKinematic = true;
            }

            m_objectSphereCastHit = false;
            m_grabbedObjectEntityId = AZ::EntityId();
            m_lastGrabbedObjectEntityId = AZ::EntityId();
            m_meshEntityPtr = nullptr;

            m_state = ObjectInteractionStates::idleState;
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnRotateStop);
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnHoldStop);
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
        if ((m_forceTransition && m_targetState == ObjectInteractionStates::idleState) ||
            (!m_isStateLocked &&
             ((m_grabEnableToggle && m_prevGrabKeyValue == 0.f && m_grabKeyValue != 0.f) ||
              (!m_grabEnableToggle && m_prevGrabKeyValue == 0.f))))
        {
            // Set Angular Damping back to original value if Grab Key is not pressed
            SetCurrentGrabbedObjectAngularDamping(m_prevObjectAngularDamping);

            // Set Linear Damping back to zero if sphere cast doesn't hit
            SetCurrentGrabbedObjectLinearDamping(m_prevObjectLinearDamping);

            // Set Angular Velocity back to zero if Grab Key is not pressed
            SetGrabbedObjectAngularVelocity(AZ::Vector3::CreateZero());

            // Reset current grabbed distance to m_initialGrabDistance if grab key is not pressed
            m_grabDistance = m_initialGrabDistance;

            // Set Object Current Layer variable back to initial layer if Grab Key is not pressed
            SetCurrentGrabbedCollisionLayer(m_prevGrabbedCollisionLayer);

            // Restore gravity if it was disabled
            if (m_disableGravityWhileHeld && !m_isObjectKinematic)
            {
                Physics::RigidBodyRequestBus::Event(
                    m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::SetGravityEnabled, m_prevGravityEnabled);
            }

            // Set Grabbed Object back to Dynamic Rigid Body if previously dynamic
            if (!m_isInitialObjectKinematic)
            {
                SetGrabbedObjectKinematicElseDynamic(false);
                m_isObjectKinematic = false;
            }
            // Set Grabbed Object back to Kinematic Rigid Body if previously kinematic
            else if (m_isInitialObjectKinematic)
            {
                SetGrabbedObjectKinematicElseDynamic(true);
                m_isObjectKinematic = true;
            }

            m_objectSphereCastHit = false;
            m_grabbedObjectEntityId = AZ::EntityId();
            m_lastGrabbedObjectEntityId = AZ::EntityId();
            m_meshEntityPtr = nullptr;

            m_state = ObjectInteractionStates::idleState;
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnRotateStop);
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnHoldStop);
            m_forceTransition = false;
        }
        // Go back to hold state if rotate key is pressed again because we want to stop rotating the 
        // object on the second key press. Other conditionals allow forced state transition to 
        // bypass inputs with m_forceTransition, or prevent state transition with m_isStateLocked
        else if (
            (m_forceTransition && m_targetState == ObjectInteractionStates::holdState) ||
            (!m_isStateLocked &&
             ((m_rotateEnableToggle && m_prevRotateKeyValue == 0.f && m_rotateKeyValue != 0.f) ||
              (!m_rotateEnableToggle && m_rotateKeyValue == 0.f))))
        {
            SetCurrentGrabbedObjectAngularDamping(m_prevObjectAngularDamping);
            SetGrabbedObjectAngularVelocity(AZ::Vector3::CreateZero());

            // Recompute relative quaternion after rotation changes (to lock new orientation)
            AZ::Quaternion grabbingEntityRotationQuat;
            if (m_useFPControllerForGrab)
            {
                grabbingEntityRotationQuat = GetEntity()->GetTransform()->GetWorldRotationQuaternion();
            }
            else
            {
                grabbingEntityRotationQuat = m_grabbingEntityPtr->GetTransform()->GetWorldRotationQuaternion();
            }

            AZ::Quaternion grabbedObjectRotationQuat;
            AZ::TransformBus::EventResult(
                grabbedObjectRotationQuat, m_lastGrabbedObjectEntityId, &AZ::TransformInterface::GetWorldRotationQuaternion);
            m_grabbedObjectRelativeQuat = grabbingEntityRotationQuat.GetInverseFull() * grabbedObjectRotationQuat;

            m_tidalLockPidController.Reset();

            m_state = ObjectInteractionStates::holdState;
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnRotateStop);
            m_forceTransition = false;
        }
        // Transition to throwState if Throw key is pressed. Other conditionals allow forced state transition to 
        // bypass inputs with m_forceTransition, or prevent state transition with m_isStateLocked
        else if (
            (m_forceTransition && m_targetState == ObjectInteractionStates::throwState && !m_isInitialObjectKinematic) ||
            (!m_isStateLocked && m_throwKeyValue != 0.f && !m_isInitialObjectKinematic))
        {
            // Set Angular Damping back to original value
            SetCurrentGrabbedObjectAngularDamping(m_prevObjectAngularDamping);

            // Set Linear Damping back to zero if sphere cast doesn't hit
            SetCurrentGrabbedObjectLinearDamping(m_prevObjectLinearDamping);

            // Set Angular Velocity to zero when no longer rotating
            SetGrabbedObjectAngularVelocity(AZ::Vector3::CreateZero());

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

            m_objectSphereCastHit = false;
            m_meshEntityPtr = nullptr;

            m_state = ObjectInteractionStates::throwState;
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnRotateStop);
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnHoldStop);
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnThrowStart);
            m_forceTransition = false;
        }
        else
        {
            m_state = ObjectInteractionStates::rotateState;
        }
    }

    void ObjectInteractionComponent::ThrowObjectState(const float &deltaTime)
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
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnThrowStop);
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnMaxThrowDistance);
            m_state = ObjectInteractionStates::idleState;
        }
        // Escape from the throw state if grabbed object is in throw state longer than m_throwStateMaxTime
        else if (m_throwStateCounter <= 0.f)
        {
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnThrowStop);
            ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnThrowStateCounterZero);
            m_state = ObjectInteractionStates::idleState;
        }
        else
        {
            m_state = ObjectInteractionStates::throwState;
        }
    }

    // Perform a spherecast query to check if colliding with a grabbable object, then assign the 
    // first returned hit to m_grabbedObjectEntityId
    void ObjectInteractionComponent::CheckForObjects()
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
        #endif
        else
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
            AzPhysics::SceneQuery::QueryType::StaticAndDynamic,
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

        // Prioritize hit matching m_lastGrabbedObjectEntityId if grabbing
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
            // Take the first hit for initial grab
            m_objectSphereCastHit = true;
            m_grabbedObjectEntityId = hits.m_hits.at(0).m_entityId;
            m_lastGrabbedObjectEntityId = m_grabbedObjectEntityId;
        }
    }

    // Hold and move object using physics or translation, based on object's 
    // starting Rigid Body type, or if KinematicWhileHeld is enabled
    void ObjectInteractionComponent::HoldObject(float deltaTime)
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
            m_grabReference.SetTranslation(
                m_grabbingEntityPtr->GetTransform()->GetWorldTM().GetTranslation() + m_forwardVector * m_grabDistance);
        }
        #endif
        // Use user-specified grab entity for Grab Reference
        else
        {
            // Get forward vector relative to the grabbing entity's transform
            m_forwardVector = m_grabbingEntityPtr->GetTransform()->GetWorldTM().GetBasisY();
            // Creates a reference point for the Grabbed Object translation in front of the Grabbing Entity
            m_grabReference = m_grabbingEntityPtr->GetTransform()->GetWorldTM();
            m_grabReference.SetTranslation(
                m_grabbingEntityPtr->GetTransform()->GetWorldTM().GetTranslation() + m_forwardVector * m_grabDistance);
        }

        m_grabbedObjectTranslation = GetEntityPtr(m_lastGrabbedObjectEntityId)->GetTransform()->GetWorldTM().GetTranslation();

        // Move the object using Translation (Transform) if it is a Kinematic Rigid Body
        if (m_isObjectKinematic)
        {
            // Move object by setting its Translation
            AZ::TransformBus::Event(
                m_lastGrabbedObjectEntityId, &AZ::TransformInterface::SetWorldTranslation, m_grabReference.GetTranslation());
            
            // If object is NOT in rotate state, couple the grabbed entity's rotation to 
            // the controlling entity's local z rotation (causing object to face controlling entity)
            if (m_state != ObjectInteractionStates::rotateState && m_tidalLock && m_kinematicTidalLock)
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

        // Move the object using SetLinearVelocity (PhysX) if it is a Dynamic Rigid Body
        else
        {
            // Compute error once
            AZ::Vector3 error = m_grabReference.GetTranslation() - m_grabbedObjectTranslation;

            AZ::Vector3 targetVector;
            if (m_enablePIDHeldDynamics)
            {
                // Use PID to compute spring-like velocity adjustment
                targetVector = m_pidController.Output(error, deltaTime, m_grabbedObjectTranslation);
            }
            else
            {
                // Fallback to original simple proportional control
                targetVector = error * m_grabResponse;
            }

            // Query mass for potential scaling (default to 1 if fails)
            float mass = 1.0f;
            Physics::RigidBodyRequestBus::EventResult(mass, m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::GetMass);

            if (m_enablePIDHeldDynamics)
            {
                AZ::Vector3 pid_out = targetVector;

                // Add feed-forward for target velocity only in Velocity mode (ErrorRate handles it natively)
                if (m_velocityCompensation && m_pidController.GetMode() == PidController<AZ::Vector3>::Velocity)
                {
                    float effective_factor = 1.0f - exp(-m_velocityCompDampRate * deltaTime);
                    m_currentCompensationVelocity = m_currentCompensationVelocity.Lerp(m_grabbingEntityVelocity, effective_factor);
                    pid_out += m_derivativeGain * m_currentCompensationVelocity;
                }

                // Treat PID output as force; optionally scale by mass for mass-independent behavior
                AZ::Vector3 force = m_massIndependentPID ? mass * pid_out : pid_out;

                // Apply as impulse
                AZ::Vector3 total_impulse = force * deltaTime;
                Physics::RigidBodyRequestBus::Event(
                    m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::ApplyLinearImpulse, total_impulse);
            }
            else
            {
                // Compute grabbing entity velocity compensation
                AZ::Vector3 compensation = AZ::Vector3::CreateZero();
                if (m_velocityCompensation)
                {
                    float effective_factor = 1.0f - exp(-m_velocityCompDampRate * deltaTime);
                    m_currentCompensationVelocity = m_currentCompensationVelocity.Lerp(m_grabbingEntityVelocity, effective_factor);
                    compensation = m_currentCompensationVelocity;
                }

                // Original velocity-based application
                Physics::RigidBodyRequestBus::Event(
                    m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::SetLinearVelocity, targetVector + compensation);
            }

            // If object is NOT in rotate state, couple the grabbed entity's rotation to
            // the controlling entity's local z rotation
            if (m_state != ObjectInteractionStates::rotateState && m_tidalLock && m_dynamicTidalLock)
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
    void ObjectInteractionComponent::RotateObject(float deltaTime)
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
        #endif
        else
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
    void ObjectInteractionComponent::ThrowObject()
    {
        // Query mass for potential scaling (default to 1 if fails)
        float mass = 1.0f;
        Physics::RigidBodyRequestBus::EventResult(mass, m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::GetMass);

        // Compute base impulse
        AZ::Vector3 base_impulse = m_forwardVector * m_throwImpulse;

        // Optionally scale by mass for mass-independent throw velocity
        AZ::Vector3 impulse = m_massIndependentThrow ? mass * base_impulse : base_impulse;

        // Apply a Linear Impulse to the grabbed object
        Physics::RigidBodyRequestBus::Event(
            m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequestBus::Events::ApplyLinearImpulse, impulse);

        // Trigger an event notification when object enters Throw State
        ObjectInteractionNotificationBus::Broadcast(&ObjectInteractionNotificationBus::Events::OnThrowStart);

        m_thrownGrabbedObjectEntityId = m_lastGrabbedObjectEntityId;

        // Reset current grabbed distance to m_initialGrabDistance if grab key is not pressed
        m_grabDistance = m_initialGrabDistance;

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

    // Apply tidal lock to grabbed object while grabbing it. This keeps the object facing you in its last rotation while in grabbed state
    void ObjectInteractionComponent::TidalLock(float deltaTime)
    {
        // Initialize local variables for the current entity's rotation quaternion and up vector
        AZ::Quaternion grabbingEntityRotationQuat = AZ::Quaternion::CreateIdentity();
        AZ::Vector3 grabbingEntityUpVector = AZ::Vector3::CreateZero();

        // Determine the rotation and up vector based on whether First Person Controller is used
        #ifdef FIRST_PERSON_CONTROLLER
        if (m_useFPControllerForGrab)
        {
            grabbingEntityRotationQuat = GetEntity()->GetTransform()->GetWorldRotationQuaternion();
            grabbingEntityUpVector = grabbingEntityRotationQuat.TransformVector(AZ::Vector3::CreateAxisZ());
        }
        #endif
        else
        {
            grabbingEntityRotationQuat = m_grabbingEntityPtr->GetTransform()->GetWorldRotationQuaternion();
            grabbingEntityUpVector = grabbingEntityRotationQuat.TransformVector(AZ::Vector3::CreateAxisZ());
        }

        // Compute target object rotation based on stored relative
        AZ::Quaternion targetGrabbedObjectRotation = grabbingEntityRotationQuat * m_grabbedObjectRelativeQuat;

        // Get current object rotation
        AZ::Quaternion currentGrabbedObjectRotation = AZ::Quaternion::CreateIdentity();
        AZ::TransformBus::EventResult(
            currentGrabbedObjectRotation, m_lastGrabbedObjectEntityId, &AZ::TransformInterface::GetWorldRotationQuaternion);

        // Compute error quaternion (target * current_inv)
        AZ::Quaternion errorQuat = targetGrabbedObjectRotation * currentGrabbedObjectRotation.GetInverseFull();
        errorQuat.Normalize();

        // Convert to axis-angle (ensure shortest arc)
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
            // For dynamic objects
            AZ::Vector3 targetAngularVelocity = angularError / deltaTime;

            if (m_enablePIDTidalLockDynamics)
            {
                AZ::Vector3 angularPidOutput =
                    m_tidalLockPidController.Output(angularError, deltaTime, AZ::Vector3::CreateZero());

                AZ::Vector3 angularImpulse = angularPidOutput * deltaTime;

                if (m_massIndependentTidalLock)
                {
                    float mass = 1.0f;
                    Physics::RigidBodyRequestBus::EventResult(mass, m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::GetMass);
                    angularImpulse *= mass;
                }

                Physics::RigidBodyRequestBus::Event(
                    m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::ApplyAngularImpulse, angularImpulse);
            }
            else
            {
                // Simple mode: Directly set angular velocity (always mass-independent)
                SetGrabbedObjectAngularVelocity(targetAngularVelocity);
            }
        }
    }

    #ifdef FIRST_PERSON_CONTROLLER
    void ObjectInteractionComponent::FreezeCharacterRotation()
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
                "Object Interaction Component",
                false,
                "No First Person Controller Component handler available to freeze character rotation.")
        }
    }
    #endif

    void ObjectInteractionComponent::UpdateGrabDistance(float deltaTime)
    {
        // Grab distance value depends on whether grab distance input key is ignored via SetGrabbedDistanceKeyValue()
        const float grabDistanceValue = m_ignoreGrabDistanceKeyInputValue ? m_grabDistanceKeyValue : m_combinedGrabDistance;

        // Changes distance between Grabbing Entity and Grabbed object. Minimum and
        // maximum grab distances determined by m_minGrabDistance and m_maxGrabDistance, respectively
        m_grabDistance =
            AZ::GetClamp(m_grabDistance + (grabDistanceValue * m_grabDistanceSpeed * deltaTime), m_minGrabDistance, m_maxGrabDistance);
    }

    void ObjectInteractionComponent::ComputeGrabbingEntityVelocity(float deltaTime)
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

    AZ::Entity* ObjectInteractionComponent::GetEntityPtr(AZ::EntityId pointer) const
    {
        auto ca = AZ::Interface<AZ::ComponentApplicationRequests>::Get();
        return ca->FindEntity(pointer);
    }

    // Handles disconnecting, updating, and reconnecting input bindings
    void ObjectInteractionComponent::UpdateInputBinding(
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
    void ObjectInteractionComponent::OnObjectSphereCastHit()
    {
    }
    void ObjectInteractionComponent::OnHoldStart()
    {
    }
    void ObjectInteractionComponent::OnHoldStop()
    {
    }
    void ObjectInteractionComponent::OnRotateStart()
    {
    }
    void ObjectInteractionComponent::OnRotateStop()
    {
    }
    void ObjectInteractionComponent::OnThrowStart()
    {
    }
    void ObjectInteractionComponent::OnThrowStop()
    {
    }
    void ObjectInteractionComponent::OnMaxThrowDistance()
    {
    }
    void ObjectInteractionComponent::OnThrowStateCounterZero()
    {
    }

    // Request Bus getter and setter methods for use in scripts
    AZ::EntityId ObjectInteractionComponent::GetGrabbingEntityId() const
    {
        return m_grabbingEntityPtr->GetId();
    }

    AZ::EntityId ObjectInteractionComponent::GetActiveCameraEntityId() const
    {
        return GetActiveCameraEntityPtr()->GetId();
    }

    AZ::EntityId ObjectInteractionComponent::GetGrabbedObjectEntityId() const
    {
        return m_grabbedObjectEntityId;
    }

    AZ::EntityId ObjectInteractionComponent::GetLastGrabbedObjectEntityId() const
    {
        return m_lastGrabbedObjectEntityId;
    }

    AZ::EntityId ObjectInteractionComponent::GetThrownGrabbedObjectEntityId() const
    {
        return m_thrownGrabbedObjectEntityId;
    }

    void ObjectInteractionComponent::SetThrownGrabbedObjectEntityId(const AZ::EntityId new_thrownGrabbedObjectEntityId)
    {
        m_thrownGrabbedObjectEntityId = new_thrownGrabbedObjectEntityId;
    }

    void ObjectInteractionComponent::SetGrabbingEntity(const AZ::EntityId new_grabbingEntityId)
    {
        m_grabbingEntityPtr = GetEntityPtr(new_grabbingEntityId);
    }

    AZStd::string ObjectInteractionComponent::GetStateString() const
    {
        return m_statesMap.find(m_state)->second;
    }

    bool ObjectInteractionComponent::GetIsInIdleState() const
    {
        return (m_state == ObjectInteractionStates::idleState);
    }

    bool ObjectInteractionComponent::GetIsInCheckState() const
    {
        return (m_state == ObjectInteractionStates::checkState);
    }

    bool ObjectInteractionComponent::GetIsInHeldState() const
    {
        return (m_state == ObjectInteractionStates::holdState);
    }

    bool ObjectInteractionComponent::GetIsInRotateState() const
    {
        return (m_state == ObjectInteractionStates::rotateState);
    }

    bool ObjectInteractionComponent::GetIsInThrowState() const
    {
        return (m_state == ObjectInteractionStates::throwState);
    }

    bool ObjectInteractionComponent::GetObjectSphereCastHit() const
    {
        return m_objectSphereCastHit;
    }

    bool ObjectInteractionComponent::GetStayInIdleState() const
    {
        return m_stayInIdleState;
    }
    
    void ObjectInteractionComponent::SetStayInIdleState(const bool& new_stayInIdleState)
    {
        m_stayInIdleState = new_stayInIdleState;
    }

    bool ObjectInteractionComponent::GetGrabEnableToggle() const
    {
        return m_grabEnableToggle;
    }

    void ObjectInteractionComponent::SetGrabEnableToggle(const bool& new_grabEnableToggle)
    {
        m_grabEnableToggle = new_grabEnableToggle;
    }

    bool ObjectInteractionComponent::GetRotateEnableToggle() const
    {
        return m_rotateEnableToggle;
    }

    void ObjectInteractionComponent::SetRotateEnableToggle(const bool& new_rotateEnableToggle)
    {
        m_rotateEnableToggle = new_rotateEnableToggle;
    }

    float ObjectInteractionComponent::GetGrabKeyValue() const
    {
        return m_grabKeyValue;
    }

    void ObjectInteractionComponent::SetGrabKeyValue(const float& new_grabKeyValue)
    {
        m_grabKeyValue = new_grabKeyValue;
    }

    float ObjectInteractionComponent::GetThrowKeyValue() const
    {
        return m_throwKeyValue;
    }

    void ObjectInteractionComponent::SetThrowKeyValue(const float& new_throwKeyValue)
    {
        m_throwKeyValue = new_throwKeyValue;
    }

    float ObjectInteractionComponent::GetRotateKeyValue() const
    {
        return m_rotateKeyValue;
    }

    void ObjectInteractionComponent::SetRotateKeyValue(const float& new_rotateKeyValue)
    {
        m_rotateKeyValue = new_rotateKeyValue;
    }

    float ObjectInteractionComponent::GetPitchKeyValue() const
    {
         return m_pitchKeyValue;
    }

    void ObjectInteractionComponent::SetPitchKeyValue(const float& new_pitchKeyValue, const bool& new_ignorePitchKeyInputValue)
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

    float ObjectInteractionComponent::GetYawKeyValue() const
    {
        return m_yawKeyValue;
    }

    void ObjectInteractionComponent::SetYawKeyValue(const float& new_yawKeyValue, const bool& new_ignoreYawKeyInputValue)
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

    float ObjectInteractionComponent::GetRollKeyValue() const
    {
        return m_rollKeyValue;
    }

    void ObjectInteractionComponent::SetRollKeyValue(const float& new_rollKeyValue, const bool& new_ignoreRollKeyInputValue)
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

    AZ::EntityId ObjectInteractionComponent::GetMeshEntityId() const
    {
        return m_meshEntityId;
    }

    void ObjectInteractionComponent::SetMeshEntityId(const AZ::EntityId& new_meshEntityId)
    {
        m_meshEntityId = new_meshEntityId;
        m_meshEntityPtr = m_meshEntityId.IsValid() ? GetEntityPtr(m_meshEntityId) : nullptr;
    }

    AZStd::string ObjectInteractionComponent::GetMeshEntityName() const
    {
        return m_meshEntityName;
    }

    void ObjectInteractionComponent::SetMeshEntityName(const AZStd::string& new_meshEntityName)
    {
        m_meshEntityName = new_meshEntityName;
        // Reset m_meshEntityId to invalid to prioritize name-based lookup
        m_meshEntityId = AZ::EntityId();
        m_meshEntityPtr = nullptr;
    }

    float ObjectInteractionComponent::GetGrabbedDistanceKeyValue() const
    {
        return m_grabDistanceKeyValue;
    }

    void ObjectInteractionComponent::SetGrabbedDistanceKeyValue(const float& new_grabDistanceKeyValue, const bool& new_ignoreGrabDistanceKeyInputValue)
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

    float ObjectInteractionComponent::GetGrabbedObjectDistance() const
    {
        return m_grabDistance;
    }

    void ObjectInteractionComponent::SetGrabbedObjectDistance(const float& new_grabDistance)
    {
        m_grabDistance = AZ::GetClamp(new_grabDistance, m_minGrabDistance, m_maxGrabDistance);
    }

    float ObjectInteractionComponent::GetMinGrabbedObjectDistance() const
    {
        return m_minGrabDistance;
    }

    void ObjectInteractionComponent::SetMinGrabbedObjectDistance(const float& new_minGrabDistance)
    {
        m_minGrabDistance = new_minGrabDistance;
    }

    float ObjectInteractionComponent::GetMaxGrabbedObjectDistance() const
    {
        return m_maxGrabDistance;
    }

    void ObjectInteractionComponent::SetMaxGrabbedObjectDistance(const float& new_maxGrabDistance)
    {
        m_maxGrabDistance = new_maxGrabDistance;
    }

    float ObjectInteractionComponent::GetInitialGrabbedObjectDistance() const
    {
        return m_initialGrabDistance;
    }

    void ObjectInteractionComponent::SetInitialGrabbedObjectDistance(const float& new_initialGrabDistance)
    {
        m_initialGrabDistance = new_initialGrabDistance;
    }

    float ObjectInteractionComponent::GetGrabbedObjectDistanceSpeed() const
    {
        return m_grabDistanceSpeed;
    }

    void ObjectInteractionComponent::SetGrabbedObjectDistanceSpeed(const float& new_grabDistanceSpeed)
    {
        m_grabDistanceSpeed = new_grabDistanceSpeed;
    }

    float ObjectInteractionComponent::GetGrabResponse() const
    {
        return m_grabResponse;
    }

    void ObjectInteractionComponent::SetGrabResponse(const float& new_grabResponse)
    {
        m_grabResponse = new_grabResponse;
    }

    bool ObjectInteractionComponent::GetDynamicTidalLock() const
    {
        return m_dynamicTidalLock;
    }

    void ObjectInteractionComponent::SetDynamicTidalLock(const bool& new_dynamicTidalLock)
    {
        m_dynamicTidalLock = new_dynamicTidalLock;
    }

    bool ObjectInteractionComponent::GetKinematicTidalLock() const
    {
        return m_kinematicTidalLock;
    }

    void ObjectInteractionComponent::SetKinematicTidalLock(const bool& new_kinematicTidalLock)
    {
        m_kinematicTidalLock = new_kinematicTidalLock;
    }

    bool ObjectInteractionComponent::GetTidalLock() const
    {
        return m_tidalLock;
    }

    void ObjectInteractionComponent::SetTidalLock(const bool& new_tidalLock)
    {
        m_tidalLock = new_tidalLock;
    }

    float ObjectInteractionComponent::GetDynamicYawRotateScale() const
    {
        return m_dynamicYawRotateScale;
    }

    void ObjectInteractionComponent::SetDynamicYawRotateScale(const float& new_dynamicYawRotateScale)
    {
        m_dynamicYawRotateScale = new_dynamicYawRotateScale;
    }

    float ObjectInteractionComponent::GetDynamicPitchRotateScale() const
    {
        return m_dynamicPitchRotateScale;
    }

    void ObjectInteractionComponent::SetDynamicPitchRotateScale(const float& new_dynamicPitchRotateScale)
    {
        m_dynamicPitchRotateScale = new_dynamicPitchRotateScale;
    }

    float ObjectInteractionComponent::GetKinematicYawRotateScale() const
    {
        return m_kinematicYawRotateScale;
    }

    void ObjectInteractionComponent::SetKinematicYawRotateScale(const float& new_kinematicYawRotateScale)
    {
        m_kinematicYawRotateScale = new_kinematicYawRotateScale;
    }

    float ObjectInteractionComponent::GetKinematicPitchRotateScale() const
    {
        return m_kinematicPitchRotateScale;
    }

    void ObjectInteractionComponent::SetKinematicPitchRotateScale(const float& new_kinematicPitchRotateScale)
    {
        m_kinematicPitchRotateScale = new_kinematicPitchRotateScale;
    }

    bool ObjectInteractionComponent::GetVelocityCompensation() const
    {
        return m_velocityCompensation;
    }

    void ObjectInteractionComponent::SetVelocityCompensation(const bool& new_velocityCompensation)
    {
        m_velocityCompensation = new_velocityCompensation;
    }

    float ObjectInteractionComponent::GetVelocityCompDampRate() const
    {
        return m_velocityCompDampRate;
    }

    void ObjectInteractionComponent::SetVelocityCompDampRate(const float& new_velocityCompDampRate)
    {
        m_velocityCompDampRate = new_velocityCompDampRate;
    }

    bool ObjectInteractionComponent::GetSmoothDynamicRotation() const
    {
        return m_smoothDynamicRotation;
    }

    void ObjectInteractionComponent::SetSmoothDynamicRotation(const bool& new_smoothDynamicRotation)
    {
        m_smoothDynamicRotation = new_smoothDynamicRotation;
    }

    float ObjectInteractionComponent::GetAngularVelocityDampRate() const
    {
        return m_angularVelocityDampRate;
    }

    void ObjectInteractionComponent::SetAngularVelocityDampRate(const float& new_angularVelocityDampRate)
    {
        m_angularVelocityDampRate = new_angularVelocityDampRate;
    }

    float ObjectInteractionComponent::GetThrowImpulse() const
    {
        return m_throwImpulse;
    }

    void ObjectInteractionComponent::SetThrowImpulse(const float& new_throwImpulse)
    {
        m_throwImpulse = new_throwImpulse;
    }

    float ObjectInteractionComponent::GetGrabbedObjectThrowStateCounter() const
    {
        return m_throwStateCounter;
    }
    
    void ObjectInteractionComponent::SetGrabbedObjectThrowStateCounter(const float& new_throwStateCounter)
    {
        m_throwStateCounter = new_throwStateCounter;
    }

    float ObjectInteractionComponent::GetGrabbedObjectThrowStateTime() const
    {
        return m_throwStateMaxTime;
    }

    void ObjectInteractionComponent::SetGrabbedObjectThrowStateTime(const float& new_throwStateMaxTime)
    {
        m_throwStateMaxTime = new_throwStateMaxTime;
    }

    float ObjectInteractionComponent::GetSphereCastRadius() const
    {
        return m_sphereCastRadius;
    }

    void ObjectInteractionComponent::SetSphereCastRadius(const float& new_sphereCastRadius)
    {
        m_sphereCastRadius = new_sphereCastRadius;
    }

    float ObjectInteractionComponent::GetSphereCastDistance() const
    {
        return m_sphereCastDistance;
    }

    void ObjectInteractionComponent::SetSphereCastDistance(const float& new_sphereCastDistance)
    {
        m_sphereCastDistance = new_sphereCastDistance;
    }

    AZStd::string ObjectInteractionComponent::GetGrabbedCollisionGroup() const
    {
        AZStd::string groupName;
        Physics::CollisionRequestBus::BroadcastResult(
            groupName, &Physics::CollisionRequests::GetCollisionGroupName, m_grabbedCollisionGroup);
        return groupName;
    }

    void ObjectInteractionComponent::SetGrabbedCollisionGroup(const AZStd::string& new_grabbedCollisionGroupName)
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

    AZStd::string ObjectInteractionComponent::GetCurrentGrabbedCollisionLayerName() const
    {
        AZStd::string currentGrabbedCollisionLayerName;
        Physics::CollisionFilteringRequestBus::EventResult(
            currentGrabbedCollisionLayerName,
            m_lastGrabbedObjectEntityId,
            &Physics::CollisionFilteringRequestBus::Events::GetCollisionLayerName);
        return currentGrabbedCollisionLayerName;
    }

    void ObjectInteractionComponent::SetCurrentGrabbedCollisionLayerByName(const AZStd::string& new_currentGrabbedCollisionLayerName)
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

    AzPhysics::CollisionLayer ObjectInteractionComponent::GetCurrentGrabbedCollisionLayer() const
    {
        AZStd::string grabbedCollisionLayerName;
        Physics::CollisionFilteringRequestBus::EventResult(
            grabbedCollisionLayerName, m_lastGrabbedObjectEntityId, &Physics::CollisionFilteringRequestBus::Events::GetCollisionLayerName);
        AzPhysics::CollisionLayer grabbedCollisionLayer;
        Physics::CollisionRequestBus::BroadcastResult(
            grabbedCollisionLayer, &Physics::CollisionRequests::GetCollisionLayerByName, grabbedCollisionLayerName);
        return grabbedCollisionLayer;
    }

    void ObjectInteractionComponent::SetCurrentGrabbedCollisionLayer(const AzPhysics::CollisionLayer& new_currentGrabbedCollisionLayer)
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

    AZStd::string ObjectInteractionComponent::GetPrevGrabbedCollisionLayerName() const
    {
        const AzPhysics::CollisionConfiguration& configuration =
            AZ::Interface<AzPhysics::SystemInterface>::Get()->GetConfiguration()->m_collisionConfig;
        return configuration.m_collisionLayers.GetName(m_prevGrabbedCollisionLayer);
    }

    void ObjectInteractionComponent::SetPrevGrabbedCollisionLayerByName(const AZStd::string& new_prevGrabbedCollisionLayerName)
    {
        bool success = false;
        AzPhysics::CollisionLayer prevGrabbedCollisionLayer;
        Physics::CollisionRequestBus::BroadcastResult(
            success, &Physics::CollisionRequests::TryGetCollisionLayerByName, new_prevGrabbedCollisionLayerName, prevGrabbedCollisionLayer);
        if (success)
            m_prevGrabbedCollisionLayer = prevGrabbedCollisionLayer;
    }

    AzPhysics::CollisionLayer ObjectInteractionComponent::GetPrevGrabbedCollisionLayer() const
    {
        return m_prevGrabbedCollisionLayer;
    }

    void ObjectInteractionComponent::SetPrevGrabbedCollisionLayer(const AzPhysics::CollisionLayer& new_prevGrabbedCollisionLayer)
    {
        m_prevGrabbedCollisionLayer = new_prevGrabbedCollisionLayer;
    }

    AZStd::string ObjectInteractionComponent::GetTempGrabbedCollisionLayerName() const
    {
        return m_tempGrabbedCollisionLayerName;
    }

    void ObjectInteractionComponent::SetTempGrabbedCollisionLayerByName(const AZStd::string& new_tempGrabbedCollisionLayerName)
    {
        bool success = false;
        AzPhysics::CollisionLayer tempGrabbedCollisionLayer;
        Physics::CollisionRequestBus::BroadcastResult(
            success, &Physics::CollisionRequests::TryGetCollisionLayerByName, new_tempGrabbedCollisionLayerName, tempGrabbedCollisionLayer);
        if (success)
        {
            m_tempGrabbedCollisionLayerName = new_tempGrabbedCollisionLayerName;
            m_tempGrabbedCollisionLayer = tempGrabbedCollisionLayer;
        }
    }

    AzPhysics::CollisionLayer ObjectInteractionComponent::GetTempGrabbedCollisionLayer() const
    {
        return m_tempGrabbedCollisionLayer;
    }

    void ObjectInteractionComponent::SetTempGrabbedCollisionLayer(const AzPhysics::CollisionLayer& new_tempGrabbedCollisionLayer)
    {
        m_tempGrabbedCollisionLayer = new_tempGrabbedCollisionLayer;
        const AzPhysics::CollisionConfiguration& configuration =
            AZ::Interface<AzPhysics::SystemInterface>::Get()->GetConfiguration()->m_collisionConfig;
        m_tempGrabbedCollisionLayerName = configuration.m_collisionLayers.GetName(m_tempGrabbedCollisionLayer);
    }

    bool ObjectInteractionComponent::GetGrabbedObjectKinematicElseDynamic() const
    {
        bool isObjectKinematic = false;
        Physics::RigidBodyRequestBus::EventResult(
            isObjectKinematic, m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequestBus::Events::IsKinematic);

        return isObjectKinematic;
    }

    void ObjectInteractionComponent::SetGrabbedObjectKinematicElseDynamic(const bool& isKinematic)
    {
        Physics::RigidBodyRequestBus::Event(m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::SetKinematic, isKinematic);
    }

    bool ObjectInteractionComponent::GetInitialGrabbedObjectIsKinematic() const
    {
        return m_isInitialObjectKinematic;
    }

    float ObjectInteractionComponent::GetCurrentGrabbedObjectAngularDamping() const
    {
        float currentObjectAngularDamping = 0.f;

        Physics::RigidBodyRequestBus::EventResult(
            currentObjectAngularDamping, m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::GetAngularDamping);

        return currentObjectAngularDamping;
    }

    void ObjectInteractionComponent::SetCurrentGrabbedObjectAngularDamping(const float& new_currentObjectAngularDamping)
    {
        m_currentObjectAngularDamping = new_currentObjectAngularDamping;

        Physics::RigidBodyRequestBus::Event(
            m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::SetAngularDamping, new_currentObjectAngularDamping);
    }

    float ObjectInteractionComponent::GetPrevGrabbedObjectAngularDamping() const
    {
        return m_prevObjectAngularDamping;
    }

    void ObjectInteractionComponent::SetPrevGrabbedObjectAngularDamping(const float& new_prevObjectAngularDamping)
    {
        m_prevObjectAngularDamping = new_prevObjectAngularDamping;
    }

    float ObjectInteractionComponent::GetTempGrabbedObjectAngularDamping() const
    {
        return m_tempObjectAngularDamping;
    }

    void ObjectInteractionComponent::SetTempGrabbedObjectAngularDamping(const float& new_tempObjectAngularDamping)
    {
        m_tempObjectAngularDamping = new_tempObjectAngularDamping;
        Physics::RigidBodyRequestBus::Event(
            m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::SetAngularDamping, m_tempObjectAngularDamping);
    }

    float ObjectInteractionComponent::GetCurrentGrabbedObjectLinearDamping() const
    {
        float currentObjectLinearDamping = 0.f;

        Physics::RigidBodyRequestBus::EventResult(
            currentObjectLinearDamping, m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::GetLinearDamping);

        return currentObjectLinearDamping;
    }

    void ObjectInteractionComponent::SetCurrentGrabbedObjectLinearDamping(const float& new_currentObjectLinearDamping)
    {
        m_currentObjectLinearDamping = new_currentObjectLinearDamping;

        Physics::RigidBodyRequestBus::Event(
            m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::SetLinearDamping, new_currentObjectLinearDamping);
    }

    float ObjectInteractionComponent::GetPrevGrabbedObjectLinearDamping() const
    {
        return m_prevObjectLinearDamping;
    }

    void ObjectInteractionComponent::SetPrevGrabbedObjectLinearDamping(const float& new_prevObjectLinearDamping)
    {
        m_prevObjectLinearDamping = new_prevObjectLinearDamping;
    }

    float ObjectInteractionComponent::GetTempGrabbedObjectLinearDamping() const
    {
        return m_tempObjectLinearDamping;
    }

    void ObjectInteractionComponent::SetTempGrabbedObjectLinearDamping(const float& new_tempObjectLinearDamping)
    {
        m_tempObjectLinearDamping = new_tempObjectLinearDamping;
        Physics::RigidBodyRequestBus::Event(
            m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::SetLinearDamping, m_tempObjectLinearDamping);
    }

    AZ::Vector3 ObjectInteractionComponent::GetGrabbedObjectAngularVelocity() const
    {
        AZ::Vector3 grabbedObjectAngularVelocity = AZ::Vector3::CreateZero();

        Physics::RigidBodyRequestBus::EventResult(
            grabbedObjectAngularVelocity, m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::GetAngularVelocity);
        return grabbedObjectAngularVelocity;
    }

    void ObjectInteractionComponent::SetGrabbedObjectAngularVelocity(const AZ::Vector3& new_grabbedObjectAngularVelocity)
    {
        m_grabbedObjectAngularVelocity = new_grabbedObjectAngularVelocity;

        Physics::RigidBodyRequestBus::Event(
            m_lastGrabbedObjectEntityId, &Physics::RigidBodyRequests::SetAngularVelocity, m_grabbedObjectAngularVelocity);
    }

    bool ObjectInteractionComponent::GetInitialAngularVelocityZero() const
    {
        return m_initialAngularVelocityZero;
    }

    void ObjectInteractionComponent::SetInitialAngularVelocityZero(const bool& new_initialAngularVelocityZero)
    {
        m_initialAngularVelocityZero = new_initialAngularVelocityZero;
    }
    
    void ObjectInteractionComponent::ForceTransition(const ObjectInteractionStates& targetState)
    {
        m_forceTransition = true;
        m_targetState = targetState;
    }

    void ObjectInteractionComponent::SetStateLocked(const bool& isLocked)
    {
        m_isStateLocked = isLocked;
    }

    bool ObjectInteractionComponent::GetStateLocked() const
    {
        return m_isStateLocked;
    }

    AZStd::string ObjectInteractionComponent::GetGrabInputKey() const
    {
        return m_strGrab;
    }
    void ObjectInteractionComponent::SetGrabInputKey(const AZStd::string& keyName)
    {
        UpdateInputBinding(m_grabEventId, m_strGrab, keyName);
    }

    AZStd::string ObjectInteractionComponent::GetThrowInputKey() const
    {
        return m_strThrow;
    }
    void ObjectInteractionComponent::SetThrowInputKey(const AZStd::string& keyName)
    {
        UpdateInputBinding(m_throwEventId, m_strThrow, keyName);
    }

    AZStd::string ObjectInteractionComponent::GetRotateInputKey() const
    {
        return m_strRotate;
    }
    void ObjectInteractionComponent::SetRotateInputKey(const AZStd::string& keyName)
    {
        UpdateInputBinding(m_rotateEventId, m_strRotate, keyName);
    }

    AZStd::string ObjectInteractionComponent::GetRotatePitchInputKey() const
    {
        return m_strRotatePitch;
    }
    void ObjectInteractionComponent::SetRotatePitchInputKey(const AZStd::string& keyName)
    {
        UpdateInputBinding(m_rotatePitchEventId, m_strRotatePitch, keyName);
    }

    AZStd::string ObjectInteractionComponent::GetRotateYawInputKey() const
    {
        return m_strRotateYaw;
    }
    void ObjectInteractionComponent::SetRotateYawInputKey(const AZStd::string& keyName)
    {
        UpdateInputBinding(m_rotateYawEventId, m_strRotateYaw, keyName);
    }

    AZStd::string ObjectInteractionComponent::GetRotateRollInputKey() const
    {
        return m_strRotateRoll;
    }
    void ObjectInteractionComponent::SetRotateRollInputKey(const AZStd::string& keyName)
    {
        UpdateInputBinding(m_rotateRollEventId, m_strRotateRoll, keyName);
    }

    AZStd::string ObjectInteractionComponent::GetGrabDistanceInputKey() const
    {
        return m_strGrabDistance;
    }
    void ObjectInteractionComponent::SetGrabDistanceInputKey(const AZStd::string& keyName)
    {
        UpdateInputBinding(m_grabDistanceEventId, m_strGrabDistance, keyName);
    }
} // namespace ObjectInteraction
