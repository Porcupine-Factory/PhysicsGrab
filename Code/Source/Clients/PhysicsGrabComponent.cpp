#include <Clients/PhysicsGrabComponent.h>

#ifdef NETWORKPHYSICSGRAB
#include <Multiplayer/NetworkPhysicsGrabComponent.h>
#endif

#include <Atom/RPI.Public/ViewportContext.h>
#include <Atom/RPI.Public/ViewportContextBus.h>

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
                ->Field("Detect In Idle", &PhysicsGrabComponent::m_detectInIdle)

                ->Field("Grab Entity", &PhysicsGrabComponent::m_grabbingEntityId)
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
                ->Field("Max Drop Distance", &PhysicsGrabComponent::m_maxDropDistance)
                ->Attribute(AZ::Edit::Attributes::Suffix, " " + Physics::NameConstants::GetLengthUnit())
                ->Field("Grab Distance Speed", &PhysicsGrabComponent::m_grabDistanceSpeed)
                ->Attribute(AZ::Edit::Attributes::Suffix, " " + Physics::NameConstants::GetSpeedUnit())

                ->Field("Throw Impulse", &PhysicsGrabComponent::m_throwImpulse)
                ->Attribute(AZ::Edit::Attributes::Suffix, AZStd::string::format(" N%ss", Physics::NameConstants::GetInterpunct().c_str()))
                ->Field("Enable Mass Independent Throw", &PhysicsGrabComponent::m_massIndependentThrow)
                ->Field("Throw State Max Time", &PhysicsGrabComponent::m_throwStateMaxTime)
                ->Attribute(AZ::Edit::Attributes::Suffix, " s")
                ->Field("Charge Throw", &PhysicsGrabComponent::m_enableChargeThrow)
                ->Field("Charge While Rotating", &PhysicsGrabComponent::m_enableChargeWhileRotating)
                ->Field("Min Throw Impulse", &PhysicsGrabComponent::m_minThrowImpulse)
                ->Attribute(AZ::Edit::Attributes::Suffix, AZStd::string::format(" N%ss", Physics::NameConstants::GetInterpunct().c_str()))
                ->Field("Max Throw Impulse", &PhysicsGrabComponent::m_maxThrowImpulse)
                ->Attribute(AZ::Edit::Attributes::Suffix, AZStd::string::format(" N%ss", Physics::NameConstants::GetInterpunct().c_str()))
                ->Field("Charge Time", &PhysicsGrabComponent::m_chargeTime)
                ->Attribute(AZ::Edit::Attributes::Suffix, " s")

#ifdef FIRST_PERSON_CONTROLLER
                ->Field("Freeze Character Rotation", &PhysicsGrabComponent::m_freezeCharacterRotation)
#endif
                ->Field("Dynamic Horizontal Rotate Scale", &PhysicsGrabComponent::m_dynamicYawRotateScale)
                ->Field("Dynamic Vertical Rotate Scale", &PhysicsGrabComponent::m_dynamicPitchRotateScale)
                ->Field("Dynamic Roll Rotate Scale", &PhysicsGrabComponent::m_dynamicRollRotateScale)
                ->Field("Kinematic Horizontal Rotate Scale", &PhysicsGrabComponent::m_kinematicYawRotateScale)
                ->Field("Kinematic Vertical Rotate Scale", &PhysicsGrabComponent::m_kinematicPitchRotateScale)
                ->Field("Kinematic Roll Rotate Scale", &PhysicsGrabComponent::m_kinematicRollRotateScale)
                ->Field("Rotate Enable Toggle", &PhysicsGrabComponent::m_rotateEnableToggle)
                ->Field("Gravity Applies To Point Rotation", &PhysicsGrabComponent::m_gravityAppliesToPointRotation)
                ->Field("Smooth Dynamic Rotation", &PhysicsGrabComponent::m_smoothDynamicRotation)
                ->Field("Angular Velocity Damp Rate", &PhysicsGrabComponent::m_angularVelocityDampRate)

                ->Field("PID Held Dynamics", &PhysicsGrabComponent::m_enablePIDHeldDynamics)
                ->Field("Mass Independent PID", &PhysicsGrabComponent::m_massIndependentHeldPID)
                ->Field("PID P Gain", &PhysicsGrabComponent::m_heldProportionalGain)
                ->Attribute(AZ::Edit::Attributes::Suffix, " N/m")
                ->Field("PID I Gain", &PhysicsGrabComponent::m_heldIntegralGain)
                ->Attribute(
                    AZ::Edit::Attributes::Suffix, AZStd::string::format(" N/(m%ss)", Physics::NameConstants::GetInterpunct().c_str()))
                ->Field("PID D Gain", &PhysicsGrabComponent::m_heldDerivativeGain)
                ->Attribute(AZ::Edit::Attributes::Suffix, AZStd::string::format(" N%ss/m", Physics::NameConstants::GetInterpunct().c_str()))
                ->Field("PID Integral Limit", &PhysicsGrabComponent::m_heldIntegralWindupLimit)
                ->Attribute(AZ::Edit::Attributes::Suffix, " N")
                ->Field("PID Deriv Filter Alpha", &PhysicsGrabComponent::m_heldDerivativeFilterAlpha)

                ->Field("Enable PID Tidal Lock Dynamics", &PhysicsGrabComponent::m_enablePIDTidalLockDynamics)
                ->Field("Mass Independent Tidal Lock", &PhysicsGrabComponent::m_massIndependentTidalLock)
                ->Field("Scale Independent Tidal Lock", &PhysicsGrabComponent::m_scaleIndependentTidalLock)
                ->Field("Tidal Lock PID P Gain", &PhysicsGrabComponent::m_tidalLockProportionalGain)
                ->Attribute(
                    AZ::Edit::Attributes::Suffix, AZStd::string::format(" N%sm/rad", Physics::NameConstants::GetInterpunct().c_str()))
                ->Field("Tidal Lock PID I Gain", &PhysicsGrabComponent::m_tidalLockIntegralGain)
                ->Attribute(
                    AZ::Edit::Attributes::Suffix,
                    AZStd::string::format(
                        " N%sm/(rad%ss)", Physics::NameConstants::GetInterpunct().c_str(), Physics::NameConstants::GetInterpunct().c_str()))
                ->Field("Tidal Lock PID D Gain", &PhysicsGrabComponent::m_tidalLockDerivativeGain)
                ->Attribute(
                    AZ::Edit::Attributes::Suffix,
                    AZStd::string::format(
                        " N%sm%ss/rad", Physics::NameConstants::GetInterpunct().c_str(), Physics::NameConstants::GetInterpunct().c_str()))
                ->Field("Tidal Lock PID Integral Limit", &PhysicsGrabComponent::m_tidalLockIntegralWindupLimit)
                ->Attribute(AZ::Edit::Attributes::Suffix, AZStd::string::format(" N%sm", Physics::NameConstants::GetInterpunct().c_str()))
                ->Field("Tidal Lock PID Deriv Filter Alpha", &PhysicsGrabComponent::m_tidalLockDerivativeFilterAlpha)
                ->Version(1);

            if (AZ::EditContext* ec = sc->GetEditContext())
            {
                using namespace AZ::Edit;
                ec->Class<PhysicsGrabComponent>("Physics Grab", "Enables grabbing, holding, rotating, and throwing physics objects")
                    ->ClassElement(ClassElements::EditorData, "")
                    ->Attribute(Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "Physics Grab")

                    // Input Binding Keys
                    ->ClassElement(AZ::Edit::ClassElements::Group, "Input Bindings")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, false)
                    ->DataElement(nullptr, &PhysicsGrabComponent::m_strGrab, "Grab Key", "Grab interaction input binding")
                    ->DataElement(nullptr, &PhysicsGrabComponent::m_strGrabDistance, "Grab Distance Key", "Grab distance input binding")
                    ->DataElement(nullptr, &PhysicsGrabComponent::m_strThrow, "Throw Input Key", "Throw object input binding")
                    ->DataElement(nullptr, &PhysicsGrabComponent::m_strRotate, "Rotate Enable Key", "Enable rotate object input binding")
                    ->DataElement(
                        nullptr, &PhysicsGrabComponent::m_strRotatePitch, "Rotate Pitch Key", "Rotate object about X axis input binding")
                    ->DataElement(
                        nullptr, &PhysicsGrabComponent::m_strRotateYaw, "Rotate Yaw Key", "Rotate object about Z axis input binding")
                    ->DataElement(
                        nullptr, &PhysicsGrabComponent::m_strRotateRoll, "Rotate Roll Key", "Rotate object about Y axis input binding")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Detection Parameters")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, false)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_grabbedCollisionGroupId,
                        "Sphere Cast Collision Group",
                        "Collision group for detecting grabbable objects via sphere cast. Only objects in this group can be grabbed.")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_tempGrabbedCollisionLayer,
                        "Temp Grabbed Object Collision Layer",
                        "Temporary collision layer for held objects (e.g., to prevent jumping on held objects). Restored on release.")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_sphereCastRadius,
                        "Sphere Cast Radius",
                        "Radius of detection sphere for grabbing (higher = larger grab area).")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_sphereCastDistance,
                        "Sphere Cast Distance",
                        "Maximum reach for grabbing (higher = farther grabs).")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_detectInIdle,
                        "Detect In Idle",
                        "If enabled, performs sphere cast in idle state to detect potential grabbable objects without grabbing.")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Hold Parameters")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, false)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_grabbingEntityId,
                        "Grab Entity",
                        "Entity performing grabs (e.g., player camera). Defaults to active camera if blank. Determines grab "
                        "origin/direction.")
#ifdef FIRST_PERSON_CONTROLLER
                    ->Attribute(AZ::Edit::Attributes::ReadOnly, &PhysicsGrabComponent::GetUseFPControllerForGrab)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_useFPControllerForGrab,
                        "Use First Person Controller For Grab",
                        "Use First Person Controller player character for grab reference. Enabling this creates tighter tracking by "
                        "bypassing "
                        "potential camera interpolation lag.")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::AttributesAndValues)
#endif
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_meshSmoothing,
                        "Mesh Smoothing",
                        "Smooths visual mesh for dynamic objects to reduce physics jitter (enable recommended if physics timesteps are "
                        "less than "
                        "render framerate.)")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::AttributesAndValues)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_meshTagName,
                        "Mesh Tag",
                        "Tag for child entity used in mesh smoothing (e.g., 'GrabMesh'). Allows visual separation from physics body.")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &PhysicsGrabComponent::GetMeshSmoothing)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_grabResponse,
                        "Grab Response",
                        "Velocity scale for pulling held objects (higher = quicker snap to position, more rigid feel). Used when PID "
                        "disabled.")
                    ->Attribute(AZ::Edit::Attributes::ReadOnly, &PhysicsGrabComponent::GetEnablePIDHeldDynamics)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_grabEnableToggle,
                        "Grab Enable Toggle",
                        "Toggles grab on press (enable for click-to-hold; disable to require holding key).")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_grabMaintained,
                        "Maintain Grab",
                        "Keeps holding object even if sphere cast misses (enable to prevent drops during fast turns).")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::AttributesAndValues)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_kinematicWhileHeld,
                        "Kinematic While Held",
                        "Makes held object kinematic (no physics simulation; enable for stable holding, disable for dynamic interactions "
                        "and collisions).")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::AttributesAndValues)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_disableGravityWhileHeld,
                        "Disable Gravity While Held",
                        "Turns off gravity for dynamic held objects (enable for weightless feel; disable for natural sag under gravity).")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_offsetGrab,
                        "Offset Grab",
                        "Applies force at hit point for dynamic objects (enable for realistic tilt/rotation; disable "
                        "to use center-of-mass).")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::AttributesAndValues)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_tidalLock,
                        "Tidal Lock",
                        "Locks object rotation to face grab entity (enable for consistent orientation; disable for free spin during hold).")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::AttributesAndValues)
#ifdef FIRST_PERSON_CONTROLLER
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_fullTidalLockForFPC,
                        "Full Tidal Lock For First Person Controller",
                        "Uses full camera rotation for tidal lock in FPC (enable for head-tracking lock; disable for body yaw only).")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &PhysicsGrabComponent::GetTidalLockAndUseFPController)
#endif
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_velocityCompensation,
                        "Velocity Compensation",
                        "Adjusts object distance for velocity changes in grabbing entity. Enabling this will keep grab distance "
                        "the same whether you are walking, sprinting, or standing still.")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::AttributesAndValues)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_velocityCompDampRate,
                        "Velocity Compensation Damp Rate",
                        "Smoothing rate for velocity adjustment (higher = quicker adaptation to speed changes, less lag).")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &PhysicsGrabComponent::GetVelocityCompensation)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_tempObjectLinearDamping,
                        "Linear Damping",
                        "Damps linear motion while held (higher = quicker stop, less drift; low = more momentum preservation).")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_tempObjectAngularDamping,
                        "Angular Damping",
                        "Damps angular motion while held/rotated (higher = less wobble/spin; low = more natural rotation decay).")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Distance Parameters")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, false)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_minGrabDistance,
                        "Min Grab Distance",
                        "Closest allowable hold distance (higher = prevents clipping into entity; too high limits close inspection).")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_maxGrabDistance,
                        "Max Grab Distance",
                        "Farthest allowable hold distance (higher = more reach; too high may feel unstable at distance).")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::AttributesAndValues)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_maxDropDistance,
                        "Max Drop Distance",
                        "Absolute distance threshold between grabber and object; drops if exceeded (independent of Sphere Cast Distance). "
                        "Cannot be less than Max Grab Distance.")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &PhysicsGrabComponent::GetGrabMaintained)
                    ->Attribute(AZ::Edit::Attributes::Min, &PhysicsGrabComponent::m_maxGrabDistance)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_grabDistanceSpeed,
                        "Grab Distance Speed",
                        "Speed of distance adjustment (higher = quicker push/pull; balance for smooth control without jerk).")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Rotate Parameters")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, false)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_dynamicYawRotateScale,
                        "Dynamic Horizontal Rotate Scale",
                        "Yaw (horizontal) angular velocity scale for dynamic objects (higher = faster turn).")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_dynamicPitchRotateScale,
                        "Dynamic Vertical Rotate Scale",
                        "Pitch (vertical) angular velocity scale for dynamic objects (higher = faster turn).")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_dynamicRollRotateScale,
                        "Dynamic Roll Rotate Scale",
                        "Roll angular velocity scale for dynamic objects (higher = faster turn).")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_kinematicYawRotateScale,
                        "Kinematic Horizontal Rotate Scale",
                        "Yaw (horizontal) rotation speed scale for kinematic objects (higher = faster turn).")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &PhysicsGrabComponent::GetKinematicWhileHeld)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_kinematicPitchRotateScale,
                        "Kinematic Vertical Rotate Scale",
                        "Pitch (vertical) rotation speed scale for kinematic objects (higher = faster turn).")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &PhysicsGrabComponent::GetKinematicWhileHeld)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_kinematicRollRotateScale,
                        "Kinematic Roll Rotate Scale",
                        "Roll rotation speed scale for kinematic objects (higher = faster turn).")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &PhysicsGrabComponent::GetKinematicWhileHeld)
#ifdef FIRST_PERSON_CONTROLLER
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_freezeCharacterRotation,
                        "Freeze Character Rotation",
                        "Locks FPC rotation during rotate mode (enable to focus on object; disable for simultaneous movement).")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &PhysicsGrabComponent::GetUseFPControllerForGrab)
#endif
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_rotateEnableToggle,
                        "Rotate Enable Toggle",
                        "Toggles rotate mode on press (enable for click-to-rotate; disable to require holding for mode).")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_smoothDynamicRotation,
                        "Smooth Dynamic Rotation",
                        "Gradually applies angular velocity for dynamic objects in rotate mode (enable for fluid turns; disable for "
                        "instant response).")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::AttributesAndValues)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_angularVelocityDampRate,
                        "Angular Velocity Damp Rate",
                        "Smoothing rate for rotation buildup (higher = faster ramp-up, more responsive; low = gradual acceleration).")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &PhysicsGrabComponent::GetSmoothDynamicRotation)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_gravityAppliesToPointRotation,
                        "Gravity Applies To Point Rotation",
                        "Allows gravity at offset point during rotation (enable for natural droop; disable for uniform behavior). "
                        "Requires Offset Grab and PID Tidal Lock.")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &PhysicsGrabComponent::GetOffsetGrab)

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Throw Parameters")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, false)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_throwImpulse,
                        "Throw Impulse",
                        "Base throw strength for non-charged throws (higher = farther/faster throws; scales velocity if mass-independent).")
                    ->Attribute(AZ::Edit::Attributes::ReadOnly, &PhysicsGrabComponent::GetEnableChargeThrow)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_massIndependentThrow,
                        "Enable Mass Independent Throw",
                        "Scales throw strength by mass for consistent velocity (enable for uniform throws; "
                        "disable for heavier objects to throw shorter/slower). ")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_throwStateMaxTime,
                        "Throw State Max Time",
                        "Cooldown after throw before next grab (higher = longer wait, prevents instant re-grab).")
                    ->Attribute(AZ::Edit::Attributes::Suffix, " s")
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_enableChargeThrow,
                        "Enable Chargeable Throw",
                        "Allows holding throw key to build power (enable for variable strength; disable for fixed impulse throws).")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::AttributesAndValues)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_enableChargeWhileRotating,
                        "Enable Charge While Rotating",
                        "Permits charging during rotation mode.")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &PhysicsGrabComponent::GetEnableChargeThrow)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_minThrowImpulse,
                        "Min Throw Impulse",
                        "Lowest strength for instant throws (higher = stronger quick throws; sets charge start point).")
                    ->Attribute(
                        AZ::Edit::Attributes::Suffix, AZStd::string::format(" N%ss", Physics::NameConstants::GetInterpunct().c_str()))
                    ->Attribute(AZ::Edit::Attributes::Visibility, &PhysicsGrabComponent::GetEnableChargeThrow)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_maxThrowImpulse,
                        "Max Throw Impulse",
                        "Highest strength after full charge (higher = more powerful max throws; sets charge ceiling).")
                    ->Attribute(
                        AZ::Edit::Attributes::Suffix, AZStd::string::format(" N%ss", Physics::NameConstants::GetInterpunct().c_str()))
                    ->Attribute(AZ::Edit::Attributes::Visibility, &PhysicsGrabComponent::GetEnableChargeThrow)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_chargeTime,
                        "Charge Time",
                        "Duration to reach max impulse (higher = slower charge buildup, more control over strength).")
                    ->Attribute(AZ::Edit::Attributes::Suffix, " s")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &PhysicsGrabComponent::GetEnableChargeThrow)

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Advanced Hold Dynamics")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, false)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_enablePIDHeldDynamics,
                        "PID Held Dynamics",
                        "Enables PID controller for dynamic held objects, creating spring-like (underdamped/overdamped) motion. Disabling "
                        "uses simple linear velocity.")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::AttributesAndValues)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_massIndependentHeldPID,
                        "Mass Independent PID",
                        "Scales PID by mass for uniform behavior (enable for consistent feel across masses; "
                        "disable for heavier objects to move slower).")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &PhysicsGrabComponent::GetEnablePIDHeldDynamics)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_heldProportionalGain,
                        "Held PID P Gain",
                        "Proportional gain for holding objects. Controls stiffness/pull strength (higher = stronger spring, "
                        "quicker return but more oscillations).")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &PhysicsGrabComponent::GetEnablePIDHeldDynamics)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_heldIntegralGain,
                        "Held PID I Gain",
                        "Integral gain for holding. Reduces steady errors like gravity (higher = better offset correction, "
                        "but risk of windup/overshoot; often low/zero).")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &PhysicsGrabComponent::GetEnablePIDHeldDynamics)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_heldDerivativeGain,
                        "Held PID D Gain",
                        "Derivative gain for holding. Adds damping (higher = smoother motion, less oscillation; too high = sluggish "
                        "response).")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &PhysicsGrabComponent::GetEnablePIDHeldDynamics)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_heldIntegralWindupLimit,
                        "Held PID Integral Limit",
                        "Caps integral buildup to prevent overshoot (higher = more correction allowance; prevents excessive windup).")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &PhysicsGrabComponent::GetEnablePIDHeldDynamics)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_heldDerivativeFilterAlpha,
                        "Held PID Derivative Filter Alpha",
                        "Filters derivative noise (higher = more smoothing, less jitter but delayed response).")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &PhysicsGrabComponent::GetEnablePIDHeldDynamics)

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Advanced Tidal Lock Dynamics")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, false)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_enablePIDTidalLockDynamics,
                        "PID Tidal Lock Dynamics",
                        "Uses PID for spring-like tidal lock (enable for tunable rotation; disable for direct angular velocity).")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::AttributesAndValues)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_massIndependentTidalLock,
                        "Mass Independent Tidal Lock",
                        "Scales PID by mass for uniform rotation (enable for consistent feel; disable for heavier objects to rotate "
                        "slower).")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &PhysicsGrabComponent::GetEnablePIDTidalLockDynamics)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_scaleIndependentTidalLock,
                        "Scale Independent Tidal Lock",
                        "Scales PID by size-derived inertia (enable for consistent oscillations across scales; disable for larger objects "
                        "to feel slower).")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &PhysicsGrabComponent::GetEnablePIDTidalLockDynamics)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_tidalLockProportionalGain,
                        "Tidal Lock PID P Gain",
                        "Proportional gain for tidal lock. Controls rotational stiffness (higher = quicker alignment, more oscillations).")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &PhysicsGrabComponent::GetEnablePIDTidalLockDynamics)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_tidalLockIntegralGain,
                        "Tidal Lock PID I Gain",
                        "Integral gain for tidal lock. Fixes persistent rotation errors (higher = better correction, but risk of "
                        "overshoot; often low/zero).")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &PhysicsGrabComponent::GetEnablePIDTidalLockDynamics)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_tidalLockDerivativeGain,
                        "Tidal Lock PID D Gain",
                        "Derivative gain for tidal lock. Adds rotational damping (higher = less wobble, smoother stop; too high = slow "
                        "response).")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &PhysicsGrabComponent::GetEnablePIDTidalLockDynamics)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_tidalLockIntegralWindupLimit,
                        "Tidal Lock PID Integral Limit",
                        "Limits integral buildup for stability (higher = more correction allowance; prevents excessive windup).")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &PhysicsGrabComponent::GetEnablePIDTidalLockDynamics)
                    ->DataElement(
                        nullptr,
                        &PhysicsGrabComponent::m_tidalLockDerivativeFilterAlpha,
                        "Tidal Lock PID Derivative Filter Alpha",
                        "Filters derivative noise in tidal lock (higher = smoother but delayed).")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &PhysicsGrabComponent::GetEnablePIDTidalLockDynamics);
            }
        }

        if (auto bc = azrtti_cast<AZ::BehaviorContext*>(rc))
        {
            bc->EBus<PhysicsGrabNotificationBus>(
                  "PhysicsGrabNotificationBus", "PhysicsGrabComponentNotificationBus", "Notifications for Physics Grab Component")
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
                ->Event("Get Detected Object EntityId", &PhysicsGrabComponentRequests::GetDetectedObjectEntityId)
                ->Event("Set Detected Object EntityId", &PhysicsGrabComponentRequests::SetDetectedObjectEntityId)
                ->Event("Get Grabbed Object EntityId", &PhysicsGrabComponentRequests::GetGrabbedObjectEntityId)
                ->Event("Set Grabbed Object EntityId", &PhysicsGrabComponentRequests::SetGrabbedObjectEntityId)
                ->Event("Get Thrown Grabbed Object EntityId", &PhysicsGrabComponentRequests::GetThrownGrabbedObjectEntityId)
                ->Event("Set Thrown Grabbed Object EntityId", &PhysicsGrabComponentRequests::SetThrownGrabbedObjectEntityId)
                ->Event("Set Grabbing Entity", &PhysicsGrabComponentRequests::SetGrabbingEntity)
                ->Event("Get Grabbed Collision Group Name", &PhysicsGrabComponentRequests::GetGrabbedCollisionGroupName)
                ->Event("Set Grabbed Collision Group By Name", &PhysicsGrabComponentRequests::SetGrabbedCollisionGroupByName)
                ->Event("Get Grabbed Collision Group", &PhysicsGrabComponentRequests::GetGrabbedCollisionGroup)
                ->Event("Set Grabbed Collision Group", &PhysicsGrabComponentRequests::SetGrabbedCollisionGroup)
                ->Event("Get Collision Layer Is In Grabbed Group", &PhysicsGrabComponentRequests::GetCollisionLayerIsInGrabbedGroup)
                ->Event(
                    "Get Collision Layer Name Is In Grabbed Group", &PhysicsGrabComponentRequests::GetCollisionLayerNameIsInGrabbedGroup)
                ->Event("Get Current Grabbed Collision Layer Name", &PhysicsGrabComponentRequests::GetCurrentGrabbedCollisionLayerName)
                ->Event("Set Current Grabbed Collision Layer By Name", &PhysicsGrabComponentRequests::SetCurrentGrabbedCollisionLayerByName)
                ->Event("Get Current Grabbed Collision Layer", &PhysicsGrabComponentRequests::GetCurrentGrabbedCollisionLayer)
                ->Event("Set Current Grabbed Collision Layer", &PhysicsGrabComponentRequests::SetCurrentGrabbedCollisionLayer)
                ->Event("Get Previous Grabbed Collision Layer Name", &PhysicsGrabComponentRequests::GetPrevGrabbedCollisionLayerName)
                ->Event(
                    "Set Previous Grabbed Collision Layer Name By Name", &PhysicsGrabComponentRequests::SetPrevGrabbedCollisionLayerByName)
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
                ->Event("Get Hold Key To Check Until Hit", &PhysicsGrabComponentRequests::GetHoldKeyToCheckUntilHit)
                ->Event("Set Hold Key To Check Until Hit", &PhysicsGrabComponentRequests::SetHoldKeyToCheckUntilHit)
                ->Event("Get Grab Enable Toggle", &PhysicsGrabComponentRequests::GetGrabEnableToggle)
                ->Event("Set Grab Enable Toggle", &PhysicsGrabComponentRequests::SetGrabEnableToggle)
                ->Event("Get Rotate Enable Toggle", &PhysicsGrabComponentRequests::GetRotateEnableToggle)
                ->Event("Set Rotate Enable Toggle", &PhysicsGrabComponentRequests::SetRotateEnableToggle)
                ->Event("Get Grab Maintained", &PhysicsGrabComponentRequests::GetGrabMaintained)
                ->Event("Set Grab Maintained", &PhysicsGrabComponentRequests::SetGrabMaintained)
                ->Event("Get Kinematic While Held", &PhysicsGrabComponentRequests::GetKinematicWhileHeld)
                ->Event("Set Kinematic While Held", &PhysicsGrabComponentRequests::SetKinematicWhileHeld)
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
                ->Event("Get Max Drop Distance", &PhysicsGrabComponentRequests::GetMaxDropDistance)
                ->Event("Set Max Drop Distance", &PhysicsGrabComponentRequests::SetMaxDropDistance)
                ->Event("Get Enable Max Drop Distance", &PhysicsGrabComponentRequests::GetEnableMaxDropDistance)
                ->Event("Set Enable Max Drop Distance", &PhysicsGrabComponentRequests::SetEnableMaxDropDistance)
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
                ->Event("Get Use First Person Controller For Grab", &PhysicsGrabComponentRequests::GetUseFPControllerForGrab)
                ->Event("Set Use First Person Controller For Grab", &PhysicsGrabComponentRequests::SetUseFPControllerForGrab)
                ->Event("Get Tidal Lock And Use FPC Controller For Grab", &PhysicsGrabComponentRequests::GetTidalLockAndUseFPController)
                ->Event("Get Mesh Smoothing", &PhysicsGrabComponentRequests::GetMeshSmoothing)
                ->Event("Set Mesh Smoothing", &PhysicsGrabComponentRequests::SetMeshSmoothing)
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
                ->Event("Get Kinematic Roll Rotate Scale", &PhysicsGrabComponentRequests::GetKinematicRollRotateScale)
                ->Event("Set Kinematic Roll Rotate Scale", &PhysicsGrabComponentRequests::SetKinematicRollRotateScale)
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
                ->Event("Force Grab", &PhysicsGrabComponentRequests::ForceGrab)
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
                ->Event("Get Is Object Grabbable", &PhysicsGrabComponentRequests::GetIsObjectGrabbable)
                ->Event("Get Detect In Idle", &PhysicsGrabComponentRequests::GetDetectInIdle)
                ->Event("Set Detect In Idle", &PhysicsGrabComponentRequests::SetDetectInIdle)
                ->Event("Get Enable PID Held Dynamics", &PhysicsGrabComponentRequests::GetEnablePIDHeldDynamics)
                ->Event("Set Enable PID Held Dynamics", &PhysicsGrabComponentRequests::SetEnablePIDHeldDynamics)
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
                ->Event("SetMeshTagName", &PhysicsGrabComponentRequests::SetMeshTagName)
                ->Event("Get Held Last Proportional", &PhysicsGrabComponentRequests::GetHeldLastProportional)
                ->Event("Get Held Last Integral", &PhysicsGrabComponentRequests::GetHeldLastIntegral)
                ->Event("Get Held Last Derivative", &PhysicsGrabComponentRequests::GetHeldLastDerivative)
                ->Event("Get Tidal Lock Last Proportional", &PhysicsGrabComponentRequests::GetTidalLockLastProportional)
                ->Event("Get Tidal Lock Last Integral", &PhysicsGrabComponentRequests::GetTidalLockLastIntegral)
                ->Event("Get Tidal Lock Last Derivative", &PhysicsGrabComponentRequests::GetTidalLockLastDerivative)
                ->Event("Get Target Translation", &PhysicsGrabComponentRequests::GetTargetTranslation)
                ->Event("Get Target Rotation", &PhysicsGrabComponentRequests::GetTargetRotation)
                ->Event("Get Is Autonomous Client", &PhysicsGrabComponentRequests::GetIsAutonomousClient)
                ->Event("Get Is Server", &PhysicsGrabComponentRequests::GetIsServer)
                ->Event("Get Is Host", &PhysicsGrabComponentRequests::GetIsHost)
                ->Event("Get Locally Enable NetworkFPC", &PhysicsGrabComponentRequests::GetLocallyEnableNetworkPhysicsGrabComponent)
                ->Event("Set Locally Enable NetworkFPC", &PhysicsGrabComponentRequests::SetLocallyEnableNetworkPhysicsGrabComponent)
                ->Event(
                    "Network Physics Grab Component Enabled Ignore Inputs",
                    &PhysicsGrabComponentRequests::NetworkPhysicsGrabComponentEnabledIgnoreInputs)
                ->Event("Is Autonomous So Connect", &PhysicsGrabComponentRequests::IsAutonomousSoConnect)
                ->Event("Not Autonomous So Disconnect", &PhysicsGrabComponentRequests::NotAutonomousSoDisconnect);

            bc->Class<PhysicsGrabComponent>()->RequestBus("PhysicsGrabComponentRequestBus");
        }
    }

    void PhysicsGrabComponent::Activate()
    {
        AssignConnectInputEvents();

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
            },
            aznumeric_cast<int32_t>(AzPhysics::SceneEvents::PhysicsStartFinishSimulationPriority::Physics));

        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        if (sceneInterface != nullptr)
        {
            sceneInterface->RegisterSceneSimulationStartHandler(m_attachedSceneHandle, m_sceneSimulationStartHandler);
        }

        m_sceneSimulationFinishHandler = AzPhysics::SceneEvents::OnSceneSimulationFinishHandler(
            [this](AzPhysics::SceneHandle sceneHandle, float fixedDeltaTime)
            {
                OnSceneSimulationFinish(sceneHandle, fixedDeltaTime);
            },
            aznumeric_cast<int32_t>(AzPhysics::SceneEvents::PhysicsStartFinishSimulationPriority::Physics));

        if (sceneInterface != nullptr)
        {
            sceneInterface->RegisterSceneSimulationFinishHandler(m_attachedSceneHandle, m_sceneSimulationFinishHandler);
        }
        // Connect the handler to the request bus
        PhysicsGrabComponentRequestBus::Handler::BusConnect(GetEntityId());
#ifdef NETWORKPHYSICSGRAB
        NetworkPhysicsGrabComponentNotificationBus::Handler::BusConnect(GetEntityId());
#endif

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

        // Fallback to active camera if Grabbing Entity not specified in editor
        if (!m_grabbingEntityId.IsValid())
        {
            m_grabbingEntityPtr = GetActiveCameraEntityPtr();
            if (m_grabbingEntityPtr == nullptr)
            {
                m_needsCameraFallback = true;
                Camera::CameraNotificationBus::Handler::BusConnect();
            }
        }
        else
        {
            // Connect EntityBus for valid ID (pointer set in OnEntityActivated)
            AZ::EntityBus::Handler::BusConnect(m_grabbingEntityId);
        }

        // Get access to the NetworkPhysicsGrab object and its member
#ifdef NETWORKPHYSICSGRAB
        const AZ::Entity* entity = GetEntity();
        m_networkPhysicsGrabObject = entity->FindComponent<NetworkPhysicsGrabComponent>();
#endif

        // Determine if the NetworkPhysicsGrab is enabled
        if (m_networkPhysicsGrabObject != nullptr)
        {
            InputEventNotificationBus::MultiHandler::BusDisconnect();
        }
    }

    // Called at the beginning of each physics tick
    void PhysicsGrabComponent::OnSceneSimulationStart(float physicsTimestep)
    {
        // Update physics timestep
        m_physicsTimestep = physicsTimestep;

        if (!m_isObjectKinematic && (m_state == PhysicsGrabStates::holdState || m_state == PhysicsGrabStates::rotateState))
        {
            ProcessStates((physicsTimestep + m_prevTimestep) / 2.f, 1);
        }

        // Reset time accumulator
        m_physicsTimeAccumulator = 0.0f;

        // Track the current physics timestep to average with the next one
        m_prevTimestep = physicsTimestep;
    }

    void PhysicsGrabComponent::OnSceneSimulationFinish(
        [[maybe_unused]] AzPhysics::SceneHandle sceneHandle, [[maybe_unused]] float fixedDeltaTime)
    {
        if (m_grabbedObjectEntityId.IsValid() && !m_isObjectKinematic && m_meshSmoothing &&
            (m_state == PhysicsGrabStates::holdState || m_state == PhysicsGrabStates::rotateState))
        {
            m_prevPhysicsTransform = m_currentPhysicsTransform;
            AZ::TransformBus::EventResult(m_currentPhysicsTransform, m_grabbedObjectEntityId, &AZ::TransformInterface::GetWorldTM);
        }
    }

    void PhysicsGrabComponent::OnEntityActivated([[maybe_unused]] const AZ::EntityId& entityId)
    {
        AZ::EntityBus::Handler::BusDisconnect();

        if (m_grabbingEntityId.IsValid() && entityId == m_grabbingEntityId)
        {
            m_grabbingEntityPtr = GetEntityPtr(entityId);
            if (m_grabbingEntityPtr == nullptr)
            {
                AZ_Warning(
                    "PhysicsGrabComponent", false, "Failed to retrieve grabbing entity for ID %s.", m_grabbingEntityId.ToString().c_str());
            }
        }

        if (m_networkPhysicsGrabObject != nullptr
#ifdef NETWORKPHYSICSGRAB
            || Multiplayer::NetEntityId() != static_cast<Multiplayer::NetEntityId>(-1)
#endif
        )
        {
            // Check whether the game is being ran in the O3DE editor
            AZ::ApplicationTypeQuery applicationType;
            if (auto componentApplicationRequests = AZ::Interface<AZ::ComponentApplicationRequests>::Get();
                componentApplicationRequests != nullptr)
            {
                componentApplicationRequests->QueryApplicationType(applicationType);
            }

            // If not running in the editor and the timestep is less than or equal to 1/(refresh rate) then disable mesh smoothing
            if (!applicationType.IsEditor())
            {
                AzFramework::NativeWindowHandle windowHandle = nullptr;
                windowHandle = AZ::RPI::ViewportContextRequests::Get()->GetDefaultViewportContext()->GetWindowHandle();
                if (windowHandle)
                {
                    float refreshRate = 60.f;
                    AzFramework::WindowRequestBus::EventResult(
                        refreshRate, windowHandle, &AzFramework::WindowRequestBus::Events::GetDisplayRefreshRate);

                    const AzPhysics::SystemConfiguration* config = AZ::Interface<AzPhysics::SystemInterface>::Get()->GetConfiguration();

                    // Disable mesh smoothing if the physics timestep is less than or equal to the refresh time
                    if (config->m_fixedTimestep <= 1.f / refreshRate)
                        m_meshSmoothing = false;
                }
            }
        }
    }

    void PhysicsGrabComponent::OnActiveViewChanged(const AZ::EntityId& activeEntityId)
    {
        if (m_needsCameraFallback)
        {
            // Fallback to active camera if Grabbing Entity not specified in editor
            m_grabbingEntityPtr = GetEntityPtr(activeEntityId);
            if (m_grabbingEntityPtr != nullptr)
            {
                m_grabbingEntityId = activeEntityId;
                Camera::CameraNotificationBus::Handler::BusDisconnect();
                m_needsCameraFallback = false;
                // AZ_Printf("PhysicsGrabComponent",
                // "Assigned active camera %s as grabbing entity.", m_grabbingEntityId.ToString().c_str());
            }
            else
            {
                AZ_Warning("PhysicsGrabComponent", false, "Active camera %s invalid.", activeEntityId.ToString().c_str());
            }
        }
    }

    void PhysicsGrabComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        InputEventNotificationBus::MultiHandler::BusDisconnect();
        PhysicsGrabComponentRequestBus::Handler::BusDisconnect();
#ifdef NETWORKPHYSICSGRAB
        NetworkPhysicsGrabComponentNotificationBus::Handler::BusDisconnect();
#endif

        m_attachedSceneHandle = AzPhysics::InvalidSceneHandle;
        m_sceneSimulationStartHandler.Disconnect();
        m_sceneSimulationFinishHandler.Disconnect();
        m_meshEntityPtr = nullptr;

        // Disconnect camera bus if flagged (from m_grabbingEntityPtr fallback handling)
        if (m_needsCameraFallback)
        {
            Camera::CameraNotificationBus::Handler::BusDisconnect();
            m_needsCameraFallback = false;
        }
        AZ::EntityBus::Handler::BusDisconnect();
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

    void PhysicsGrabComponent::AssignConnectInputEvents()
    {
        // Disconnect prior to connecting since this may be a reassignment
        InputEventNotificationBus::MultiHandler::BusDisconnect();

        if (m_controlMap.size() != (sizeof(m_inputNames) / sizeof(AZStd::string*)))
        {
            AZ_Error("Physics Grab Component", false, "Number of input IDs not equal to number of input names!");
        }
        else
        {
            for (auto& it_event : m_controlMap)
            {
                *(it_event.first) = StartingPointInput::InputEventNotificationId(
                    (m_inputNames[std::distance(m_controlMap.begin(), m_controlMap.find(it_event.first))])->c_str());
                if (!m_networkPhysicsGrabComponentEnabled)
                    InputEventNotificationBus::MultiHandler::BusConnect(*(it_event.first));
            }
        }
    }

    void PhysicsGrabComponent::OnPressed(float value)
    {
        const InputEventNotificationId* inputId = InputEventNotificationBus::GetCurrentBusId();
        if (inputId == nullptr)
            return;

        for (auto& it_event : m_controlMap)
        {
            if (*inputId == *(it_event.first))
            {
                *(it_event.second) = value;
                // print the local user ID and the action name CRC
                // AZ_Printf("Pressed", it_event.first->ToString().c_str());
            }
        }
    }

    void PhysicsGrabComponent::OnReleased(float value)
    {
        const InputEventNotificationId* inputId = InputEventNotificationBus::GetCurrentBusId();
        if (inputId == nullptr)
            return;

        for (auto& it_event : m_controlMap)
        {
            if (*inputId == *(it_event.first))
            {
                *(it_event.second) = value;
                // print the local user ID and the action name CRC
                // AZ_Printf("Released", it_event.first->ToString().c_str());
            }
        }
    }

    void PhysicsGrabComponent::OnHeld(float value)
    {
        const InputEventNotificationId* inputId = InputEventNotificationBus::GetCurrentBusId();
        if (inputId == nullptr)
            return;

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

    void PhysicsGrabComponent::ProcessStates(const float& deltaTime, const AZ::u8& tickTimestepNetwork)
    {
        // If physics update, skip full FSM transitions and only process hold/rotate on fixed timestep
        if (tickTimestepNetwork == 1 && (!m_networkPhysicsGrabComponentEnabled || m_isServer || m_isHost))
        {
            // Only handle hold and rotate for dynamic during physics fixed time steps
            if (m_isObjectKinematic)
            {
                return;
            }
            switch (m_state)
            {
            case PhysicsGrabStates::holdState:
                HoldObjectState(deltaTime, tickTimestepNetwork);
                break;
            case PhysicsGrabStates::rotateState:
                RotateObjectState(deltaTime, tickTimestepNetwork);
                break;
            default:
                return;
            }
            return;
        }

        switch (m_state)
        {
        case PhysicsGrabStates::idleState:
            IdleState();
            break;
        case PhysicsGrabStates::checkState:
            CheckForObjectsState();
            break;
        case PhysicsGrabStates::holdState:
            HoldObjectState(deltaTime, tickTimestepNetwork);
            break;
        case PhysicsGrabStates::rotateState:
            RotateObjectState(deltaTime, tickTimestepNetwork);
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
        ProcessStates((deltaTime + m_prevDeltaTime) / 2.f, 0);
        if (m_meshSmoothing)
        {
            InterpolateMeshTransform((deltaTime + m_prevDeltaTime) / 2.f);
        }

        // Track the current deltaTime to average with the next one
        m_prevDeltaTime = deltaTime;
    }

    void PhysicsGrabComponent::OnNetworkTickStart(const float& deltaTime, const bool& server, const AZ::EntityId& entity)
    {
        if (!m_isAutonomousClient && !m_isServer && !m_isHost)
        {
            NotAutonomousSoDisconnect();
            return;
        }
        if (entity != GetEntityId())
            return;
        if (!((m_isHost && server) || (m_isServer && !server)))
        {
            PhysicsGrabNotificationBus::Broadcast(
                &PhysicsGrabNotificationBus::Events::OnNetworkPhysicsGrabTickStart, deltaTime * m_physicsTimestepScaleFactor);
#ifdef NETWORKPHYSICSGRAB
            if (!m_networkPhysicsGrabComponentEnabled)
                NetworkPhysicsGrabComponentRequestBus::BroadcastResult(
                    m_networkPhysicsGrabComponentEnabled, &NetworkPhysicsGrabComponentRequestBus::Events::GetEnabled);
#endif
            ProcessStates(((deltaTime + m_prevNetworkPhysicsGrabDeltaTime) / 2.f), 2);
        }
    }

    void PhysicsGrabComponent::OnNetworkTickFinish(const float& deltaTime, const bool& server, const AZ::EntityId& entity)
    {
        if ((!m_isAutonomousClient && !m_isServer && !m_isHost) || (entity != GetEntityId()))
            return;
        if (!((m_isHost && server) || (m_isServer && !server)))
            PhysicsGrabNotificationBus::Broadcast(
                &PhysicsGrabNotificationBus::Events::OnNetworkPhysicsGrabTickFinish, deltaTime * m_physicsTimestepScaleFactor);
        m_prevNetworkPhysicsGrabDeltaTime = deltaTime;
    }

    // Smoothly update the visual transform of m_meshEntityPtr based on physics transforms
    void PhysicsGrabComponent::InterpolateMeshTransform(float deltaTime)
    {
        if (!m_grabbedObjectEntityId.IsValid() || !m_meshEntityPtr || m_isObjectKinematic)
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
        // Perform spherecast in idle if detection enabled, updating detection state without state change
        // Allows user to check if entity can be grabbed before grabbing
        if (m_detectInIdle)
        {
            CheckForObjects(true);
        }
        else
        {
            m_detectedObjectEntityId = AZ::EntityId();
            m_objectSphereCastHit = false;
        }

        if ((m_forceTransition && m_targetState == PhysicsGrabStates::checkState) ||
            (!m_isStateLocked && m_prevGrabKeyValue == 0.f && m_grabKeyValue != 0.f && !m_stayInIdleState))
        {
            m_state = PhysicsGrabStates::checkState;
            if (m_continueToHoldState)
                m_continueToHoldState = false;
            else
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
                // Don't enable local physics simulation on autonomous clients with networked objects
                // NetworkRigidBodyComponent keeps them kinematic for replication
                if (!(m_networkPhysicsGrabComponentEnabled && m_isAutonomousClient))
                {
                    SetGrabbedObjectKinematicElseDynamic(false);
                }
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
                    m_prevGravityEnabled, m_grabbedObjectEntityId, &Physics::RigidBodyRequests::IsGravityEnabled);
                Physics::RigidBodyRequestBus::Event(m_grabbedObjectEntityId, &Physics::RigidBodyRequests::SetGravityEnabled, false);
            }

            // Store mass for dynamic objects
            if (!m_isObjectKinematic)
            {
                Physics::RigidBodyRequestBus::EventResult(
                    m_grabbedObjectMass, m_grabbedObjectEntityId, &Physics::RigidBodyRequests::GetMass);
            }

            // Initialize physics transforms for dynamic objects
            if (!m_isObjectKinematic && m_meshSmoothing)
            {
                AZ::TransformBus::EventResult(m_prevPhysicsTransform, m_grabbedObjectEntityId, &AZ::TransformInterface::GetWorldTM);
                m_currentPhysicsTransform = m_prevPhysicsTransform;
                m_physicsTimeAccumulator = 0.0f;
            }

            // Compute local grab offset from initial hit position
            AZ::Transform objectTM = AZ::Transform::CreateIdentity();
            AZ::TransformBus::EventResult(objectTM, m_grabbedObjectEntityId, &AZ::TransformInterface::GetWorldTM);

            // Get inertia tensor from rigid body (local space) for Tidal Lock scaling
            AZ::Matrix3x3 grabbedObjectInertiaTensor = AZ::Matrix3x3::CreateIdentity();
            Physics::RigidBodyRequestBus::EventResult(
                grabbedObjectInertiaTensor, m_grabbedObjectEntityId, &Physics::RigidBodyRequests::GetInertiaLocal);

            // Compute average inertia from diagonal elements as scalar approximation
            const float averageGrabbedObjectInertia =
                (grabbedObjectInertiaTensor.GetElement(0, 0) + grabbedObjectInertiaTensor.GetElement(1, 1) +
                 grabbedObjectInertiaTensor.GetElement(2, 2)) /
                3.0f;

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
            // For multiplayer, only calculate if not already set by client
            if (!m_networkPhysicsGrabComponentEnabled || m_isAutonomousClient || m_isHost)
            {
                const AZ::Vector3 initialEffectivePoint = m_offsetGrab ? m_hitPosition : objectTM.GetTranslation();
                const float projectedGrabDistance =
                    (initialEffectivePoint - m_grabbingEntityTransform.GetTranslation()).Dot(m_forwardVector);
                m_grabDistance = AZ::GetClamp(projectedGrabDistance, m_minGrabDistance, m_maxGrabDistance);
            }

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
                    AZ::Entity* grabbedEntity = GetEntityPtr(m_grabbedObjectEntityId);
                    if (grabbedEntity)
                    {
                        AZStd::vector<AZ::EntityId> children;
                        AZ::TransformBus::EventResult(children, m_grabbedObjectEntityId, &AZ::TransformInterface::GetChildren);
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
            else if (m_meshSmoothing)
            {
                AZ_Warning(
                    "PhysicsGrabComponent",
                    false,
                    "Mesh smoothing enabled but no tagged child entity found for %s. Skipping interpolation to avoid physics desync.",
                    GetEntityPtr(m_grabbedObjectEntityId)->GetName().c_str());
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

            // Compute the effective grabbing entity rotation quaternion, handling First Person Controller cases if enabled
            const AZ::Quaternion grabbingEntityRotationQuat = GetEffectiveGrabbingRotation();
            AZ::Quaternion grabbedObjectRotationQuat;

            AZ::TransformBus::EventResult(
                grabbedObjectRotationQuat, m_grabbedObjectEntityId, &AZ::TransformInterface::GetWorldRotationQuaternion);
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
        // Go back to idleState if grab key is not pressed or held (depending on new bool)
        // Other conditionals allow forced state transition to bypass inputs with m_forceTransition, or
        // prevent state transition with m_isStateLocked
        else if (
            (m_forceTransition && m_targetState == PhysicsGrabStates::idleState) ||
            (!m_isStateLocked &&
             !(m_prevGrabKeyValue == 0.f && m_grabKeyValue != 0.f || (m_holdKeyToCheckUntilHit && m_grabKeyValue != 0.f))))
        {
            m_state = PhysicsGrabStates::idleState;
            m_forceTransition = false;
        }
        else
        {
            m_state = PhysicsGrabStates::checkState;
        }
    }

    void PhysicsGrabComponent::HoldObjectState(float deltaTime, const AZ::u8& tickTimestepNetwork)
    {
        if (!m_grabMaintained)
        {
            CheckForObjects();
        }

        if ((tickTimestepNetwork == 1 && (!m_networkPhysicsGrabComponentEnabled || m_isServer || m_isHost)) || tickTimestepNetwork == 2)
        {
            // Compensate for potential velocity change from grab entity
            ComputeGrabbingEntityVelocity(deltaTime);

            // Update dynamic objects on physics fixed time step
            HoldObject(deltaTime);
            // Early return ONLY for physics tick, continue for network tick
            if (tickTimestepNetwork == 1)
            {
                return;
            }
        }

        // Update m_grabbingEntityTransform to ensure it's current
        if (m_grabMaintained && m_enableMaxDropDistance)
        {
#ifdef FIRST_PERSON_CONTROLLER
            if (m_useFPControllerForGrab)
            {
                FirstPersonController::FirstPersonControllerComponentRequestBus::EventResult(
                    m_cameraRotationTransform,
                    GetEntityId(),
                    &FirstPersonController::FirstPersonControllerComponentRequests::GetCameraRotationTransform);
                // Get FPC camera world transform
                m_grabbingEntityTransform = m_cameraRotationTransform->GetWorldTM();
            }
            else
#endif
            {
                // Get our grabbing entity's world transform
                m_grabbingEntityTransform = m_grabbingEntityPtr->GetTransform()->GetWorldTM();
            }
        }

        // Compute current absolute distance and check against threshold
        AZ::Vector3 grabbingEntityTranslation;
        if ((m_isServer || m_isAutonomousClient) && m_useNetworkCameraTransform)
        {
            // On server with network mode, use network camera position (same as grab reference)
            grabbingEntityTranslation = m_networkCameraTranslation;
        }
        else
        {
            // Otherwise use grabbing entity transform
            grabbingEntityTranslation = m_grabbingEntityTransform.GetTranslation();
        }
        AZ::Vector3 grabbedEntityTranslation = AZ::Vector3::CreateZero();
        AZ::TransformBus::EventResult(grabbedEntityTranslation, m_grabbedObjectEntityId, &AZ::TransformBus::Events::GetWorldTranslation);
        const float currentGrabDistance = grabbedEntityTranslation.GetDistance(grabbingEntityTranslation);

        // Drop the object and go back to idle state if sphere cast doesn't hit
        // Other conditionals allow forced state transition to bypass inputs with m_forceTransition, or
        // prevent state transition with m_isStateLocked
        if ((m_forceTransition && m_targetState == PhysicsGrabStates::idleState) ||
            (!m_isStateLocked && (!m_objectSphereCastHit || (m_enableMaxDropDistance && currentGrabDistance > m_maxDropDistance))))
        {
            ReleaseGrabbedObject(true, false);
            m_state = PhysicsGrabStates::idleState;
            m_forceTransition = false;
            return;
        }

        // Update grab distance every frame (non-physics) for reliable input capture
        if (!m_networkPhysicsGrabComponentEnabled || m_isAutonomousClient || m_isHost)
        {
            UpdateGrabDistance(deltaTime);
        }

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
        // Handle throw input and charging logic, transitioning to throw state if triggered
        // Other conditionals prevent state transition with m_isStateLocked or if object was initially kinematic
        else if (!m_isStateLocked && !m_isInitialObjectKinematic)
        {
            // Process throw key input for press, hold (charging), and release events
            // Allow charging in hold state by default
            if (HandleThrowInput(deltaTime, true))
            {
                // Trigger the throw transition if the function signals a throw event (immediate or charged)
                TransitionToThrow(m_enableChargeThrow);
            }
        }
        else
        {
            m_state = PhysicsGrabStates::holdState;
        }
    }

    void PhysicsGrabComponent::RotateObjectState(float deltaTime, const AZ::u8& tickTimestepNetwork)
    {
        if (!m_grabMaintained)
        {
            CheckForObjects();
        }

        if ((tickTimestepNetwork == 1 && (!m_networkPhysicsGrabComponentEnabled || m_isServer || m_isHost)) || tickTimestepNetwork == 2)
        {
            // Compensate for potential velocity change from grab entity
            ComputeGrabbingEntityVelocity(deltaTime);

            // Update dynamic objects on physics fixed time step
            HoldObject(deltaTime);
            RotateObject(deltaTime);
            // Early return ONLY for physics tick, continue for network tick
            if (tickTimestepNetwork == 1)
            {
                return;
            }
        }

        // Update m_grabbingEntityTransform to ensure it's current
        if (m_grabMaintained && m_enableMaxDropDistance)
        {
#ifdef FIRST_PERSON_CONTROLLER
            if (m_useFPControllerForGrab)
            {
                FirstPersonController::FirstPersonControllerComponentRequestBus::EventResult(
                    m_cameraRotationTransform,
                    GetEntityId(),
                    &FirstPersonController::FirstPersonControllerComponentRequests::GetCameraRotationTransform);
                // Get FPC camera world transform
                m_grabbingEntityTransform = m_cameraRotationTransform->GetWorldTM();
                m_forwardVector = m_cameraRotationTransform->GetWorldTM().GetBasisY();
            }
            else
#endif
            {
                // Get our grabbing entity's world transform
                m_grabbingEntityTransform = m_grabbingEntityPtr->GetTransform()->GetWorldTM();
            }
        }
        // Compute current absolute distance and check against threshold
        AZ::Vector3 grabbingEntityTranslation;
        if ((m_isServer || m_isAutonomousClient) && m_useNetworkCameraTransform)
        {
            // On server with network mode, use network camera position (same as grab reference)
            grabbingEntityTranslation = m_networkCameraTranslation;
        }
        else
        {
            // Otherwise use grabbing entity transform
            grabbingEntityTranslation = m_grabbingEntityTransform.GetTranslation();
        }
        AZ::Vector3 grabbedEntityTranslation = AZ::Vector3::CreateZero();
        AZ::TransformBus::EventResult(grabbedEntityTranslation, m_grabbedObjectEntityId, &AZ::TransformBus::Events::GetWorldTranslation);
        const float currentGrabDistance = grabbedEntityTranslation.GetDistance(grabbingEntityTranslation);

        // Drop the object and go back to idle state if sphere cast doesn't hit. Other
        // conditionals allow forced state transition to bypass inputs with
        // m_forceTransition, or prevent state transition with m_isStateLocked
        if ((m_forceTransition && m_targetState == PhysicsGrabStates::idleState) ||
            (!m_isStateLocked && (!m_objectSphereCastHit || (m_enableMaxDropDistance && currentGrabDistance > m_maxDropDistance))))
        {
            // Set Angular Velocity back to zero if sphere cast doesn't hit
            SetGrabbedObjectAngularVelocity(AZ::Vector3::CreateZero());
            ReleaseGrabbedObject(true, true);
            m_state = PhysicsGrabStates::idleState;
            m_forceTransition = false;
            return;
        }

        // Update grab distance every frame (non-physics) for reliable input capture
        if (!m_networkPhysicsGrabComponentEnabled || m_isAutonomousClient || m_isHost)
        {
            UpdateGrabDistance(deltaTime);
        }

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
            const AZ::Quaternion grabbingEntityRotationQuat = GetEffectiveGrabbingRotation();
            AZ::Quaternion grabbedObjectRotationQuat;

            AZ::TransformBus::EventResult(
                grabbedObjectRotationQuat, m_grabbedObjectEntityId, &AZ::TransformInterface::GetWorldRotationQuaternion);
            m_grabbedObjectRelativeQuat = grabbingEntityRotationQuat.GetInverseFull() * grabbedObjectRotationQuat;

            m_tidalLockPidController.Reset();

            m_state = PhysicsGrabStates::holdState;
            PhysicsGrabNotificationBus::Broadcast(&PhysicsGrabNotificationBus::Events::OnRotateStop);
            m_forceTransition = false;
        }
        // Handle throw input and charging logic, transitioning to throw state if triggered
        // Other conditionals prevent state transition with m_isStateLocked or if object was initially kinematic
        else if (!m_isStateLocked && !m_isInitialObjectKinematic)
        {
            // Process throw key input for press, hold (charging), and release events
            // Conditionally allow charging based on rotate state flag
            if (HandleThrowInput(deltaTime, m_enableChargeWhileRotating))
            {
                // Trigger the throw transition if the function signals a throw event (immediate or charged)
                // Adjust for whether charging was enabled in this state
                TransitionToThrow(m_enableChargeThrow && m_enableChargeWhileRotating);
            }
        }
        else
        {
            m_state = PhysicsGrabStates::rotateState;
        }
    }

    void PhysicsGrabComponent::ThrowObjectState(const float& deltaTime)
    {
        // ThrowObject() is only executed once. If setting m_throwStateCounter value via ebus, it
        // is recommended to assign a value equal to m_throwStateMaxTime in order to properly execute ThrowObject()
        if (m_throwStateCounter == m_throwStateMaxTime)
        {
            ThrowObject();
            // Reset entity IDs after throwing to allow grabbing new objects
            m_detectedObjectEntityId = AZ::EntityId();
            m_grabbedObjectEntityId = AZ::EntityId();
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
    // first returned hit to m_detectedObjectEntityId
    void PhysicsGrabComponent::CheckForObjects(bool detectionOnly)
    {
        // Early exit if no valid grabbing entity to prevent null dereference
        if (m_grabbingEntityPtr == nullptr)
        {
            AZ_Error("PhysicsGrabComponent", false, "Grabbing entity is null. Skipping object check.");
            return;
        }

        // Reset detection state at the start
        m_objectSphereCastHit = false;
        if (detectionOnly)
        {
            m_detectedObjectEntityId = AZ::EntityId();
        }

        // On server, use network camera transform if available
        if (m_isServer && m_useNetworkCameraTransform)
        {
            m_grabbingEntityTransform = AZ::Transform::CreateFromQuaternionAndTranslation(m_networkCameraRotation, m_networkCameraTranslation);
            m_forwardVector = m_grabbingEntityTransform.GetBasisY();
        }
        else
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

        // If grabbed object is valid and not detection-only, prioritize hit matching m_grabbedObjectEntityId
        if (m_grabbedObjectEntityId.IsValid() && !detectionOnly)
        {
            for (const AzPhysics::SceneQueryHit& hit : hits.m_hits)
            {
                if (hit.m_entityId == m_grabbedObjectEntityId)
                {
                    if (!m_offsetGrab)
                    {
                        // Compute projected distance to object center and skip if exceeds m_sphereCastDistance (only for center grabs)
                        AZ::Transform objectTM = AZ::Transform::CreateIdentity();
                        AZ::TransformBus::EventResult(objectTM, hit.m_entityId, &AZ::TransformInterface::GetWorldTM);
                        const float centerProjected =
                            (objectTM.GetTranslation() - m_grabbingEntityTransform.GetTranslation()).Dot(m_forwardVector);
                        if (centerProjected > m_sphereCastDistance)
                        {
                            // Skip this hit if center is beyond sphere cast distance
                            continue;
                        }
                    }
                    m_objectSphereCastHit = true;
                    m_detectedObjectEntityId = hit.m_entityId;
                    return;
                }
            }
            if (m_forceTransition)
            {
                m_objectSphereCastHit = true;
                m_detectedObjectEntityId = m_grabbedObjectEntityId;
            }
            return;
        }

        // General case: take the first hit for initial grab or detection-only
        if (hits)
        {
            if (!m_offsetGrab)
            {
                // Compute projected distance to object center for the first hit (only for center grabs)
                AZ::Transform objectTM = AZ::Transform::CreateIdentity();
                AZ::TransformBus::EventResult(objectTM, hits.m_hits.at(0).m_entityId, &AZ::TransformInterface::GetWorldTM);
                const float centerProjected = (objectTM.GetTranslation() - m_grabbingEntityTransform.GetTranslation()).Dot(m_forwardVector);
                if (centerProjected <= m_sphereCastDistance)
                {
                    m_objectSphereCastHit = true;
                    m_detectedObjectEntityId = hits.m_hits.at(0).m_entityId;
                    if (!detectionOnly)
                    {
                        m_grabbedObjectEntityId = m_detectedObjectEntityId;
                        m_hitPosition = hits.m_hits.at(0).m_position;
                    }
                }
            }
            else
            {
                // When offset grab is true, use raw hit without center filter
                m_objectSphereCastHit = true;
                m_detectedObjectEntityId = hits.m_hits.at(0).m_entityId;
                if (!detectionOnly)
                {
                    m_grabbedObjectEntityId = m_detectedObjectEntityId;
                    m_hitPosition = hits.m_hits.at(0).m_position;
                }
            }
        }
    }

    // Hold and move object using physics or translation, based on object's
    // starting Rigid Body type, or if KinematicWhileHeld is enabled
    void PhysicsGrabComponent::HoldObject(float deltaTime)
    {
        // On server, use network camera transform if available
        if ((m_isServer || m_isAutonomousClient) && m_useNetworkCameraTransform)
        {
            // Construct camera transform from network data
            AZ::Transform networkCameraTransform =
                AZ::Transform::CreateFromQuaternionAndTranslation(m_networkCameraRotation, m_networkCameraTranslation);

            // Get forward vector from network camera
            m_forwardVector = networkCameraTransform.GetBasisY();

            // Set grab reference using network camera position and forward vector
            m_grabReference = networkCameraTransform;
            m_grabReference.SetTranslation(m_networkCameraTranslation + m_forwardVector * m_grabDistance);
        }
        // Use user-specified grab entity for Grab Reference
        else
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

                // Use the spherecasts axis direction pose as the virtual Z axis to align with in case the character is rotated on X or Y
                AZ::Vector3 spherecastsAxisDirectionPose = AZ::Vector3::CreateAxisZ();
                FirstPersonController::FirstPersonControllerComponentRequestBus::EventResult(
                    spherecastsAxisDirectionPose,
                    GetEntityId(),
                    &FirstPersonController::FirstPersonControllerComponentRequests::GetSphereCastsAxisDirectionPose);

                m_grabReference.SetTranslation(characterPosition + spherecastsAxisDirectionPose * (eyeHeight + cameraLocalZTravelDistance));
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
                m_grabReference.SetTranslation(m_grabReference.GetTranslation() + m_forwardVector * m_grabDistance);
            }
        }
        // Get object transform once (center of mass transform)
        AZ::Transform objectTM = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(objectTM, m_grabbedObjectEntityId, &AZ::TransformInterface::GetWorldTM);

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
                m_grabbedObjectEntityId, &AZ::TransformInterface::SetWorldTranslation, m_grabReference.GetTranslation());

            // If object is NOT in rotate state, couple the grabbed entity's rotation to
            // the controlling entity's local z rotation (causing object to face controlling entity)
            if (m_state != PhysicsGrabStates::rotateState && m_kinematicTidalLock && (m_tidalLock || m_fullTidalLockForFPC))
            {
                TidalLock(deltaTime);
            }
            // Update mesh entity transform if smoothing is disabled
            if (!m_meshSmoothing && m_meshEntityPtr && m_meshEntityPtr != GetEntityPtr(m_grabbedObjectEntityId))
            {
                AZ::TransformBus::Event(
                    m_meshEntityPtr->GetId(),
                    &AZ::TransformInterface::SetWorldTM,
                    GetEntityPtr(m_grabbedObjectEntityId)->GetTransform()->GetWorldTM());
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
                m_linearImpulse = linearForce * deltaTime;

                if (!m_networkPhysicsGrabComponentEnabled)
                {
                    if (m_offsetGrab && (m_gravityAppliesToPointRotation || m_state != PhysicsGrabStates::rotateState))
                    {
                        Physics::RigidBodyRequestBus::Event(
                            m_grabbedObjectEntityId,
                            &Physics::RigidBodyRequests::ApplyLinearImpulseAtWorldPoint,
                            m_linearImpulse,
                            effectivePoint);
                    }
                    else
                    {
                        Physics::RigidBodyRequestBus::Event(
                            m_grabbedObjectEntityId, &Physics::RigidBodyRequests::ApplyLinearImpulse, m_linearImpulse);
                    }
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
                    m_grabbedObjectEntityId, &Physics::RigidBodyRequests::SetLinearVelocity, targetLinearVelocity + compensation);
            }

            // If object is NOT in rotate state, couple the grabbed entity's rotation to
            // the controlling entity's local z rotation
            if (m_state != PhysicsGrabStates::rotateState && m_dynamicTidalLock && m_tidalLock)
            {
                TidalLock(deltaTime);
            }

            // Update current physics transform for interpolation
            if (m_meshSmoothing)
            {
                AZ::TransformBus::EventResult(m_currentPhysicsTransform, m_grabbedObjectEntityId, &AZ::TransformInterface::GetWorldTM);
            }
            // Update mesh entity transform if smoothing is disabled
            else if (m_meshEntityPtr && m_meshEntityPtr != GetEntityPtr(m_grabbedObjectEntityId))
            {
                AZ::TransformBus::Event(
                    m_meshEntityPtr->GetId(),
                    &AZ::TransformInterface::SetWorldTM,
                    GetEntityPtr(m_grabbedObjectEntityId)->GetTransform()->GetWorldTM());
            }
        }
    }

    // Rotate object using physics or transforms, based on object's starting
    // Rigid Body type, or if KinematicWhileHeld is enabled.
    void PhysicsGrabComponent::RotateObject(float deltaTime)
    {
        if ((m_isServer || m_isAutonomousClient) && m_useNetworkCameraTransform)
        {
            AZ::Transform networkCameraTransform =
                AZ::Transform::CreateFromQuaternionAndTranslation(m_networkCameraRotation, m_networkCameraTranslation);

            m_rightVector = networkCameraTransform.GetBasisX();
            m_upVector = networkCameraTransform.GetBasisZ();
            m_forwardVector = networkCameraTransform.GetBasisY();
        }
        else
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
                m_forwardVector = m_cameraRotationTransform->GetWorldTM().GetBasisY();
            }
            else
#endif
            {
                // Get right vector relative to the grabbing entity's transform
                m_rightVector = m_grabbingEntityPtr->GetTransform()->GetWorldTM().GetBasisX();
                // Get up vector relative to the grabbing entity's transform
                m_upVector = m_grabbingEntityPtr->GetTransform()->GetWorldTM().GetBasisZ();
                // Get forward vector relative to the grabbing entity's transform
                m_forwardVector = m_grabbingEntityPtr->GetTransform()->GetWorldTM().GetBasisY();
            }
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
            AZ::Quaternion rotation = AZ::Quaternion::CreateFromAxisAngle(m_upVector, yawValue * m_kinematicYawRotateScale * 0.01f) +
                AZ::Quaternion::CreateFromAxisAngle(m_rightVector, pitchValue * m_kinematicPitchRotateScale * 0.01f) +
                AZ::Quaternion::CreateFromAxisAngle(m_forwardVector, rollValue * m_kinematicRollRotateScale * 0.01f);

            AZ::Transform transform = AZ::Transform::CreateIdentity();
            AZ::TransformBus::EventResult(transform, m_grabbedObjectEntityId, &AZ::TransformInterface::GetWorldTM);

            transform.SetRotation((rotation * transform.GetRotation()).GetNormalized());

            AZ::TransformBus::Event(m_grabbedObjectEntityId, &AZ::TransformInterface::SetWorldTM, transform);

            // Update mesh entity transform if smoothing is disabled
            if (!m_meshSmoothing && m_meshEntityPtr && m_meshEntityPtr != GetEntityPtr(m_grabbedObjectEntityId))
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
                (m_forwardVector * rollSpeed * m_dynamicRollRotateScale * 0.01) + (m_upVector * yawSpeed * m_dynamicYawRotateScale) * 0.01;

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

            if (!m_networkPhysicsGrabComponentEnabled)
            {
                SetGrabbedObjectAngularVelocity(m_currentAngularVelocity);
            }

            // Reset accumulators after applying in physics branch
            m_accumPitch = 0.0f;
            m_accumYaw = 0.0f;
            m_accumRoll = 0.0f;

            // Update current physics transform for interpolation
            if (m_meshSmoothing)
            {
                AZ::TransformBus::EventResult(m_currentPhysicsTransform, m_grabbedObjectEntityId, &AZ::TransformInterface::GetWorldTM);
            }
            // Update mesh entity transform if smoothing is disabled
            else if (m_meshEntityPtr && m_meshEntityPtr != GetEntityPtr(m_grabbedObjectEntityId))
            {
                AZ::TransformBus::Event(
                    m_meshEntityPtr->GetId(),
                    &AZ::TransformInterface::SetWorldTM,
                    GetEntityPtr(m_grabbedObjectEntityId)->GetTransform()->GetWorldTM());
            }
        }
    }

    // Handles throw transition from hold and rotate
    void PhysicsGrabComponent::TransitionToThrow(bool isChargeEnabled)
    {
        // Determine if transitioning from rotate state to handle rotation-specific resets
        bool fromRotate = (m_state == PhysicsGrabStates::rotateState);

        // Start throw counter for timing the throw state duration
        m_throwStateCounter = m_throwStateMaxTime;

        // Set m_thrownGrabbedObjectEntityId early to preserve ID for ThrowObject impulse after release/reset
        m_thrownGrabbedObjectEntityId = m_grabbedObjectEntityId;

        // Determine whether to charge impulse based on enabled flag
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
        // Transition to throw state after preparing impulse
        m_state = PhysicsGrabStates::throwState;

        // Broadcast hold stop and throw start notifications
        PhysicsGrabNotificationBus::Broadcast(&PhysicsGrabNotificationBus::Events::OnHoldStop);
        PhysicsGrabNotificationBus::Broadcast(&PhysicsGrabNotificationBus::Events::OnThrowStart);

        // Reset charging state after initiating throw
        m_isChargingThrow = false;
        m_currentChargeTime = 0.f;
        m_hasNotifiedChargeComplete = false;

        // Restore object properties and release the grabbed object, with rotate-specific handling if applicable
        ReleaseGrabbedObject(false, fromRotate);
    }

    // Apply linear impulse to object if it is a Dynamic Rigid Body
    void PhysicsGrabComponent::ThrowObject()
    {
        // Query mass for potential scaling (default to 1 if fails)
        float objectMass = 1.0f;
        Physics::RigidBodyRequestBus::EventResult(objectMass, m_thrownGrabbedObjectEntityId, &Physics::RigidBodyRequests::GetMass);

        // Compute base impulse
        AZ::Vector3 base_impulse = m_forwardVector * m_currentThrowImpulse;

        // Optionally scale by mass for mass-independent throw velocity
        AZ::Vector3 impulse = m_massIndependentThrow ? objectMass * base_impulse : base_impulse;

        // Apply a Linear Impulse to the grabbed object
        Physics::RigidBodyRequestBus::Event(
            m_thrownGrabbedObjectEntityId, &Physics::RigidBodyRequestBus::Events::ApplyLinearImpulse, impulse);
    }

    void PhysicsGrabComponent::ReleaseGrabbedObject(bool notifyHoldStop, bool notifyRotateStop)
    {
        // Set Object Current Layer variable back to initial layer
        SetCurrentGrabbedCollisionLayer(m_prevGrabbedCollisionLayer);

        // Set Object Angular Damping back to original value if releasing from rotate state
        if (notifyRotateStop)
        {
            SetCurrentGrabbedObjectAngularDamping(m_prevObjectAngularDamping);
            // Set Angular Velocity back to zero when releasing from rotate state
            SetGrabbedObjectAngularVelocity(AZ::Vector3::CreateZero());
        }

        // Set Object Linear Damping back to original value
        SetCurrentGrabbedObjectLinearDamping(m_prevObjectLinearDamping);

        // Store and disable gravity for dynamic objects
        if (m_disableGravityWhileHeld && !m_isObjectKinematic)
        {
            Physics::RigidBodyRequestBus::Event(
                m_grabbedObjectEntityId, &Physics::RigidBodyRequests::SetGravityEnabled, m_prevGravityEnabled);
        }

        SetGrabbedObjectKinematicElseDynamic(m_isInitialObjectKinematic);
        m_isObjectKinematic = m_isInitialObjectKinematic;

        m_objectSphereCastHit = false;
        m_detectedObjectEntityId = AZ::EntityId();
        m_grabbedObjectEntityId = AZ::EntityId();

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

    bool PhysicsGrabComponent::HandleThrowInput(float deltaTime, bool allowCharging)
    {
        // Detect initial throw key press to start charging or prepare immediate throw
        if (m_prevThrowKeyValue == 0.f && m_throwKeyValue != 0.f)
        {
            // If chargeable throw is disabled or charging not allowed in current state, signal for immediate throw
            if (!m_enableChargeThrow || !allowCharging)
            {
                return true;
            }
            else
            {
                // Start charging process for throw, resetting timer and notification flag
                m_isChargingThrow = true;
                m_currentChargeTime = 0.f;
                m_hasNotifiedChargeComplete = false;
            }
        }
        // Accumulate charge time while the throw key is held
        else if (m_isChargingThrow && m_throwKeyValue != 0.f)
        {
            // Increment the charge timer based on delta time
            m_currentChargeTime += deltaTime;
            // Check if full charge is reached and notify if not already done, to signal completion event
            if (!m_hasNotifiedChargeComplete && m_currentChargeTime >= m_chargeTime)
            {
                PhysicsGrabNotificationBus::Broadcast(&PhysicsGrabNotificationBus::Events::OnChargeComplete);
                m_hasNotifiedChargeComplete = true;
            }
        }
        // Detect throw key release to trigger the charged throw if applicable
        else if (m_prevThrowKeyValue != 0.f && m_throwKeyValue == 0.f)
        {
            // If chargeable throw is enabled and charging was active, signal for charged throw
            if (m_enableChargeThrow && m_isChargingThrow)
            {
                return true;
            }
        }
        // No throw triggered in this input cycle
        return false;
    }

    AZ::Quaternion PhysicsGrabComponent::GetEffectiveGrabbingRotation() const
    {
        // Determine the effective rotation based on First Person Controller usage if enabled
        // Use camera rotation for full tidal lock, or character rotation otherwise
        if ((m_isServer || m_isAutonomousClient) && m_useNetworkCameraTransform)
        {
            return m_networkCameraRotation;
        }

#ifdef FIRST_PERSON_CONTROLLER
        if (m_useFPControllerForGrab)
        {
            if (m_fullTidalLockForFPC)
            {
                // Use camera rotation for full lock
                return m_cameraRotationTransform->GetWorldRotationQuaternion();
            }
            else
            {
                return GetEntity()->GetTransform()->GetWorldRotationQuaternion();
            }
        }
        else
#endif
        {
            return m_grabbingEntityPtr->GetTransform()->GetWorldRotationQuaternion();
        }
    }

    // Apply tidal lock to grabbed object while grabbing it. This keeps the object facing you in its last rotation while in grabbed state
    void PhysicsGrabComponent::TidalLock(float deltaTime)
    {
        // Initialize local variables for the current entity's rotation quaternion and up vector
        AZ::Quaternion grabbingEntityRotationQuat = AZ::Quaternion::CreateIdentity();
        // Compute the effective grabbing entity rotation quaternion, handling First Person Controller cases if enabled
        grabbingEntityRotationQuat = GetEffectiveGrabbingRotation();

        // Compute target object rotation based on stored relative
        AZ::Quaternion targetGrabbedObjectRotation = grabbingEntityRotationQuat * m_grabbedObjectRelativeQuat;

        // Get current object rotation
        AZ::Quaternion currentGrabbedObjectRotation = AZ::Quaternion::CreateIdentity();
        AZ::TransformBus::EventResult(
            currentGrabbedObjectRotation, m_grabbedObjectEntityId, &AZ::TransformInterface::GetWorldRotationQuaternion);

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
            AZ::TransformBus::EventResult(transform, m_grabbedObjectEntityId, &AZ::TransformInterface::GetWorldTM);
            transform.SetRotation(targetGrabbedObjectRotation);
            AZ::TransformBus::Event(m_grabbedObjectEntityId, &AZ::TransformInterface::SetWorldTM, transform);
        }
        else
        {
            // Compute target angular velocity
            AZ::Vector3 targetAngularVelocity = angularError / deltaTime;

            if (m_enablePIDTidalLockDynamics)
            {
                // Use PID controller to compute torque output based on the angular error.
                AZ::Vector3 angularPidOutput = m_tidalLockPidController.Output(angularError, deltaTime, AZ::Vector3::CreateZero());

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
                m_angularImpulse = angularTorque * deltaTime;

                if (!m_networkPhysicsGrabComponentEnabled)
                {
                    // Apply angular impulse to the grabbed object
                    Physics::RigidBodyRequestBus::Event(
                        m_grabbedObjectEntityId, &Physics::RigidBodyRequests::ApplyAngularImpulse, m_angularImpulse);
                }
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
                GetEntityId(),
                &FirstPersonController::FirstPersonControllerComponentRequestBus::Events::UpdateCharacterAndCameraYaw,
                0.f,
                false);
            FirstPersonController::FirstPersonControllerComponentRequestBus::Event(
                GetEntityId(), &FirstPersonController::FirstPersonControllerComponentRequestBus::Events::UpdateCameraPitch, 0.f, false);
        }
        else if (m_freezeCharacterRotation)
        {
            AZ_Warning(
                "Physics Grab Component", false, "No First Person Controller Component handler available to freeze character rotation.")
        }
    }
#endif

    void PhysicsGrabComponent::UpdateGrabDistance(float deltaTime)
    {
        // Grab distance value depends on whether grab distance input key is ignored via SetGrabbedDistanceKeyValue()
        const float grabDistanceValue = m_ignoreGrabDistanceKeyInputValue ? m_grabDistanceKeyValue : m_combinedGrabDistance;

        float grabDistanceChange = 0.0f;
        // Discrete input condition (mouse wheel)
        if (fabs(grabDistanceValue) > 1.0f)
        {
            grabDistanceChange = grabDistanceValue * m_grabDistanceWheelSensitivity * m_grabDistanceSpeed;
        }
        // Continuous/held condition (keyboard or analog)
        else
        {
            grabDistanceChange = grabDistanceValue * m_grabDistanceSpeed * deltaTime;
        }

        // Changes distance between Grabbing Entity and Grabbed object. Minimum and
        // maximum grab distances determined by m_minGrabDistance and m_maxGrabDistance, respectively
        m_grabDistance = AZ::GetClamp(m_grabDistance + grabDistanceChange, m_minGrabDistance, m_maxGrabDistance);
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
    void PhysicsGrabComponent::OnNetworkPhysicsGrabTickStart([[maybe_unused]] const float& deltaTime)
    {
    }
    void PhysicsGrabComponent::OnNetworkPhysicsGrabTickFinish([[maybe_unused]] const float& deltaTime)
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

    AZ::EntityId PhysicsGrabComponent::GetDetectedObjectEntityId() const
    {
        return m_detectedObjectEntityId;
    }

    void PhysicsGrabComponent::SetDetectedObjectEntityId(const AZ::EntityId& new_detectedObjectEntityId)
    {
        m_detectedObjectEntityId = new_detectedObjectEntityId;
    }

    AZ::EntityId PhysicsGrabComponent::GetGrabbedObjectEntityId() const
    {
        return m_grabbedObjectEntityId;
    }

    void PhysicsGrabComponent::SetGrabbedObjectEntityId(const AZ::EntityId& new_grabbedObjectEntityId)
    {
        const AZ::EntityId prevGrabbedObjectEntityId = m_grabbedObjectEntityId;
        m_grabbedObjectEntityId = new_grabbedObjectEntityId;
        // Check if the entity ID that was given is on a layer which the m_grabbedCollisionGroup has set.
        // Otherwise, don't change m_grabbedObjectEntityId.
        if (not m_grabbedCollisionGroup.IsSet(GetCurrentGrabbedCollisionLayer()))
            m_grabbedObjectEntityId = prevGrabbedObjectEntityId;
    }

    AZ::EntityId PhysicsGrabComponent::GetThrownGrabbedObjectEntityId() const
    {
        return m_thrownGrabbedObjectEntityId;
    }

    void PhysicsGrabComponent::SetThrownGrabbedObjectEntityId(const AZ::EntityId& new_thrownGrabbedObjectEntityId)
    {
        m_thrownGrabbedObjectEntityId = new_thrownGrabbedObjectEntityId;
    }

    void PhysicsGrabComponent::SetGrabbingEntity(const AZ::EntityId& new_grabbingEntityId)
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

    bool PhysicsGrabComponent::GetGrabMaintained() const
    {
        return m_grabMaintained;
    }

    void PhysicsGrabComponent::SetGrabMaintained(const bool& new_grabMaintained)
    {
        m_grabMaintained = new_grabMaintained;
    }

    bool PhysicsGrabComponent::GetKinematicWhileHeld() const
    {
        return m_kinematicWhileHeld;
    }

    void PhysicsGrabComponent::SetKinematicWhileHeld(const bool& new_kinematicWhileHeld)
    {
        m_kinematicWhileHeld = new_kinematicWhileHeld;
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

    void PhysicsGrabComponent::SetGrabbedDistanceKeyValue(
        const float& new_grabDistanceKeyValue, const bool& new_ignoreGrabDistanceKeyInputValue)
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

    float PhysicsGrabComponent::GetMaxDropDistance() const
    {
        return m_maxDropDistance;
    }

    void PhysicsGrabComponent::SetMaxDropDistance(const float& new_maxDropDistance)
    {
        m_maxDropDistance = AZ::GetMax(new_maxDropDistance, m_maxGrabDistance);
    }

    bool PhysicsGrabComponent::GetEnableMaxDropDistance() const
    {
        return m_enableMaxDropDistance;
    }

    void PhysicsGrabComponent::SetEnableMaxDropDistance(const bool& new_enableMaxDropDistance)
    {
        m_enableMaxDropDistance = new_enableMaxDropDistance;
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

    bool PhysicsGrabComponent::GetUseFPControllerForGrab() const
    {
        return m_useFPControllerForGrab;
    }

    void PhysicsGrabComponent::SetUseFPControllerForGrab(const bool& new_useFPControllerForGrab)
    {
        m_useFPControllerForGrab = new_useFPControllerForGrab;
    }

    bool PhysicsGrabComponent::GetTidalLockAndUseFPController() const
    {
        return (m_tidalLock && m_useFPControllerForGrab);
    }

    bool PhysicsGrabComponent::GetFullTidalLockForFPC() const
    {
        return m_fullTidalLockForFPC;
    }

    void PhysicsGrabComponent::SetFullTidalLockForFPC(const bool& new_fullTidalLockForFPC)
    {
        m_fullTidalLockForFPC = new_fullTidalLockForFPC;
    }

    bool PhysicsGrabComponent::GetMeshSmoothing() const
    {
        return m_meshSmoothing;
    }

    void PhysicsGrabComponent::SetMeshSmoothing(const bool& new_meshSmoothing)
    {
        m_meshSmoothing = new_meshSmoothing;
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

    AZStd::string PhysicsGrabComponent::GetGrabbedCollisionGroupName() const
    {
        AZStd::string groupName;
        Physics::CollisionRequestBus::BroadcastResult(
            groupName, &Physics::CollisionRequests::GetCollisionGroupName, m_grabbedCollisionGroup);
        return groupName;
    }

    void PhysicsGrabComponent::SetGrabbedCollisionGroupByName(const AZStd::string& new_grabbedCollisionGroupName)
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

    AzPhysics::CollisionGroup PhysicsGrabComponent::GetGrabbedCollisionGroup() const
    {
        return m_grabbedCollisionGroup;
    }

    void PhysicsGrabComponent::SetGrabbedCollisionGroup(const AzPhysics::CollisionGroup& new_grabbedCollisionGroup)
    {
        m_grabbedCollisionGroup = new_grabbedCollisionGroup;
        AZStd::string new_grabbedCollisionGroupName;
        Physics::CollisionRequestBus::BroadcastResult(
            new_grabbedCollisionGroupName, &Physics::CollisionRequests::GetCollisionGroupName, m_grabbedCollisionGroup);
        const AzPhysics::CollisionConfiguration& configuration =
            AZ::Interface<AzPhysics::SystemInterface>::Get()->GetConfiguration()->m_collisionConfig;
        m_grabbedCollisionGroupId = configuration.m_collisionGroups.FindGroupIdByName(new_grabbedCollisionGroupName);
    }

    bool PhysicsGrabComponent::GetCollisionLayerIsInGrabbedGroup(const AzPhysics::CollisionLayer& collisionLayerToCheck) const
    {
        return m_grabbedCollisionGroup.IsSet(collisionLayerToCheck);
    }

    bool PhysicsGrabComponent::GetCollisionLayerNameIsInGrabbedGroup(const AZStd::string& collisionLayerNameToCheck) const
    {
        bool success = false;
        AzPhysics::CollisionLayer collisionLayerToCheck;
        Physics::CollisionRequestBus::BroadcastResult(
            success, &Physics::CollisionRequests::TryGetCollisionLayerByName, collisionLayerNameToCheck, collisionLayerToCheck);
        if (success)
        {
            return m_grabbedCollisionGroup.IsSet(collisionLayerToCheck);
        }
        else
        {
            return false;
        }
    }

    AZStd::string PhysicsGrabComponent::GetCurrentGrabbedCollisionLayerName() const
    {
        AZStd::string currentGrabbedCollisionLayerName;
        Physics::CollisionFilteringRequestBus::EventResult(
            currentGrabbedCollisionLayerName,
            m_grabbedObjectEntityId,
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
                m_grabbedObjectEntityId,
                &Physics::CollisionFilteringRequestBus::Events::SetCollisionLayer,
                m_currentGrabbedCollisionLayerName,
                AZ::Crc32());
        }
    }

    AzPhysics::CollisionLayer PhysicsGrabComponent::GetCurrentGrabbedCollisionLayer() const
    {
        AZStd::string grabbedCollisionLayerName;
        Physics::CollisionFilteringRequestBus::EventResult(
            grabbedCollisionLayerName, m_grabbedObjectEntityId, &Physics::CollisionFilteringRequestBus::Events::GetCollisionLayerName);
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
            m_grabbedObjectEntityId,
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
            m_grabbedObjectEntityId,
            &Physics::CollisionFilteringRequestBus::Events::SetCollisionLayer,
            tempGrabbedCollisionLayerName,
            AZ::Crc32());
    }

    bool PhysicsGrabComponent::GetGrabbedObjectKinematicElseDynamic() const
    {
        bool isObjectKinematic = false;
        Physics::RigidBodyRequestBus::EventResult(
            isObjectKinematic, m_grabbedObjectEntityId, &Physics::RigidBodyRequestBus::Events::IsKinematic);

        return isObjectKinematic;
    }

    void PhysicsGrabComponent::SetGrabbedObjectKinematicElseDynamic(const bool& isKinematic)
    {
        Physics::RigidBodyRequestBus::Event(m_grabbedObjectEntityId, &Physics::RigidBodyRequests::SetKinematic, isKinematic);
    }

    bool PhysicsGrabComponent::GetInitialGrabbedObjectIsKinematic() const
    {
        return m_isInitialObjectKinematic;
    }

    float PhysicsGrabComponent::GetCurrentGrabbedObjectAngularDamping() const
    {
        float currentObjectAngularDamping = 0.f;

        Physics::RigidBodyRequestBus::EventResult(
            currentObjectAngularDamping, m_grabbedObjectEntityId, &Physics::RigidBodyRequests::GetAngularDamping);

        return currentObjectAngularDamping;
    }

    void PhysicsGrabComponent::SetCurrentGrabbedObjectAngularDamping(const float& new_currentObjectAngularDamping)
    {
        m_currentObjectAngularDamping = new_currentObjectAngularDamping;

        Physics::RigidBodyRequestBus::Event(
            m_grabbedObjectEntityId, &Physics::RigidBodyRequests::SetAngularDamping, new_currentObjectAngularDamping);
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
            m_grabbedObjectEntityId, &Physics::RigidBodyRequests::SetAngularDamping, m_tempObjectAngularDamping);
    }

    float PhysicsGrabComponent::GetCurrentGrabbedObjectLinearDamping() const
    {
        float currentObjectLinearDamping = 0.f;

        Physics::RigidBodyRequestBus::EventResult(
            currentObjectLinearDamping, m_grabbedObjectEntityId, &Physics::RigidBodyRequests::GetLinearDamping);

        return currentObjectLinearDamping;
    }

    void PhysicsGrabComponent::SetCurrentGrabbedObjectLinearDamping(const float& new_currentObjectLinearDamping)
    {
        m_currentObjectLinearDamping = new_currentObjectLinearDamping;

        Physics::RigidBodyRequestBus::Event(
            m_grabbedObjectEntityId, &Physics::RigidBodyRequests::SetLinearDamping, new_currentObjectLinearDamping);
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
            m_grabbedObjectEntityId, &Physics::RigidBodyRequests::SetLinearDamping, m_tempObjectLinearDamping);
    }

    AZ::Vector3 PhysicsGrabComponent::GetGrabbedObjectAngularVelocity() const
    {
        AZ::Vector3 grabbedObjectAngularVelocity = AZ::Vector3::CreateZero();

        Physics::RigidBodyRequestBus::EventResult(
            grabbedObjectAngularVelocity, m_grabbedObjectEntityId, &Physics::RigidBodyRequests::GetAngularVelocity);
        return grabbedObjectAngularVelocity;
    }

    void PhysicsGrabComponent::SetGrabbedObjectAngularVelocity(const AZ::Vector3& new_grabbedObjectAngularVelocity)
    {
        m_grabbedObjectAngularVelocity = new_grabbedObjectAngularVelocity;

        Physics::RigidBodyRequestBus::Event(
            m_grabbedObjectEntityId, &Physics::RigidBodyRequests::SetAngularVelocity, m_grabbedObjectAngularVelocity);
    }

    bool PhysicsGrabComponent::GetInitialAngularVelocityZero() const
    {
        return m_initialAngularVelocityZero;
    }

    void PhysicsGrabComponent::SetInitialAngularVelocityZero(const bool& new_initialAngularVelocityZero)
    {
        m_initialAngularVelocityZero = new_initialAngularVelocityZero;
    }

    // 0 == idleState
    // 1 == checkState
    // 2 == holdState
    // 3 == rotateState
    // 4 == throwState
    void PhysicsGrabComponent::ForceTransition(const PhysicsGrabStates& new_targetState)
    {
        m_forceTransition = true;
        m_targetState = new_targetState;
    }

    void PhysicsGrabComponent::ForceGrab(const AZ::EntityId& new_objectId)
    {
        const AZ::EntityId prevGrabbedObjectEntityId = m_grabbedObjectEntityId;
        m_grabbedObjectEntityId = new_objectId;
        AZ::Transform objectTM;
        AZ::TransformBus::EventResult(objectTM, new_objectId, &AZ::TransformInterface::GetWorldTM);
        m_hitPosition = objectTM.GetTranslation();
        // Check if the entity ID that was given is on a layer which the m_grabbedCollisionGroup has set.
        // Otherwise, don't change m_grabbedObjectEntityId and bail on doing the force grab.
        if (not m_grabbedCollisionGroup.IsSet(GetCurrentGrabbedCollisionLayer()))
        {
            m_grabbedObjectEntityId = prevGrabbedObjectEntityId;
        }
        else
        {
            if (m_state == PhysicsGrabStates::idleState)
            {
                // Go to the checkState but set a flag that makes sure m_forceTransition stays true until it gets to holdState
                m_continueToHoldState = true;
                ForceTransition(PhysicsGrabStates::checkState);
            }
            else if (m_state == PhysicsGrabStates::holdState)
            {
                // Temporary set the grabbed object EntityId back to reset some of its physics attributes
                m_grabbedObjectEntityId = prevGrabbedObjectEntityId;
                // If gravity is being disabled while held, set it back to its original value
                if (m_disableGravityWhileHeld)
                {
                    Physics::RigidBodyRequestBus::Event(
                        m_grabbedObjectEntityId, &Physics::RigidBodyRequests::SetGravityEnabled, m_prevGravityEnabled);
                }
                // Set Object Angular Damping back to original value
                SetCurrentGrabbedObjectAngularDamping(m_prevObjectAngularDamping);
                // Set Object Linear Damping back to original value
                SetCurrentGrabbedObjectLinearDamping(m_prevObjectLinearDamping);
                // Set Object Current Layer variable back to initial layer
                SetCurrentGrabbedCollisionLayer(m_prevGrabbedCollisionLayer);
                // Set Object back to kinematic if it originally was
                SetGrabbedObjectKinematicElseDynamic(m_isInitialObjectKinematic);
                // Set the grabbed object EntityId back to the EntityId that was passed in
                m_grabbedObjectEntityId = new_objectId;
            }
            else
            {
                ForceTransition(PhysicsGrabStates::holdState);
            }
        }
    }

    void PhysicsGrabComponent::SetStateLocked(const bool& isLocked)
    {
        m_isStateLocked = isLocked;
    }

    bool PhysicsGrabComponent::GetStateLocked() const
    {
        return m_isStateLocked;
    }

    bool PhysicsGrabComponent::GetHoldKeyToCheckUntilHit() const
    {
        return m_holdKeyToCheckUntilHit;
    }

    void PhysicsGrabComponent::SetHoldKeyToCheckUntilHit(const bool& new_holdKeyToCheckUntilHit)
    {
        m_holdKeyToCheckUntilHit = new_holdKeyToCheckUntilHit;
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

    bool PhysicsGrabComponent::GetIsObjectGrabbable() const
    {
        return m_objectSphereCastHit && m_detectedObjectEntityId.IsValid();
    }

    bool PhysicsGrabComponent::GetDetectInIdle() const
    {
        return m_detectInIdle;
    }

    void PhysicsGrabComponent::SetDetectInIdle(const bool& new_detectInIdle)
    {
        m_detectInIdle = new_detectInIdle;
        if (!m_detectInIdle)
        {
            m_detectedObjectEntityId = AZ::EntityId();
            m_objectSphereCastHit = false;
        }
    }

    bool PhysicsGrabComponent::GetEnablePIDHeldDynamics() const
    {
        return m_enablePIDHeldDynamics;
    }

    void PhysicsGrabComponent::SetEnablePIDHeldDynamics(const bool& new_enablePIDHeldDynamics)
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

    void PhysicsGrabComponent::SetHeldDerivativeMode(const PidController<AZ::Vector3>::DerivativeCalculationMode& new_heldDerivativeMode)
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

    AZ::Vector3 PhysicsGrabComponent::GetHeldLastProportional() const
    {
        return m_pidController.GetLastProportional();
    }

    AZ::Vector3 PhysicsGrabComponent::GetHeldLastIntegral() const
    {
        return m_pidController.GetLastIntegral();
    }

    AZ::Vector3 PhysicsGrabComponent::GetHeldLastDerivative() const
    {
        return m_pidController.GetLastDerivative();
    }

    AZ::Vector3 PhysicsGrabComponent::GetTidalLockLastProportional() const
    {
        return m_tidalLockPidController.GetLastProportional();
    }

    AZ::Vector3 PhysicsGrabComponent::GetTidalLockLastIntegral() const
    {
        return m_tidalLockPidController.GetLastIntegral();
    }

    AZ::Vector3 PhysicsGrabComponent::GetTidalLockLastDerivative() const
    {
        return m_tidalLockPidController.GetLastDerivative();
    }

    AZ::Vector3 PhysicsGrabComponent::GetTargetTranslation() const
    {
        // Target translation based on m_grabReference's position
        return m_grabReference.GetTranslation();
    }

    AZ::Vector3 PhysicsGrabComponent::GetTargetRotation() const
    {
        // Target rotation for tidal lock
        if (!m_tidalLock || !m_grabbedObjectEntityId.IsValid() ||
            (m_state != PhysicsGrabStates::holdState && m_state != PhysicsGrabStates::rotateState))
        {
            return AZ::Vector3::CreateZero();
        }
        AZ::Quaternion grabbingEntityRotationQuat = GetEffectiveGrabbingRotation();
        AZ::Quaternion targetQuat = grabbingEntityRotationQuat * m_grabbedObjectRelativeQuat;
        return targetQuat.GetEulerRadians();
    }

    bool PhysicsGrabComponent::GetIsAutonomousClient() const
    {
        return m_isAutonomousClient;
    }

    bool PhysicsGrabComponent::GetIsServer() const
    {
        return m_isServer;
    }

    bool PhysicsGrabComponent::GetIsHost() const
    {
        return m_isHost;
    }

    bool PhysicsGrabComponent::GetLocallyEnableNetworkPhysicsGrabComponent() const
    {
        return m_networkPhysicsGrabComponentEnabled;
    }

    void PhysicsGrabComponent::SetLocallyEnableNetworkPhysicsGrabComponent(const bool& new_networkPhysicsGrabComponentEnabled)
    {
        m_networkPhysicsGrabComponentEnabled = new_networkPhysicsGrabComponentEnabled;
        NetworkPhysicsGrabComponentRequestBus::Event(
            GetEntityId(), &NetworkPhysicsGrabComponentRequestBus::Events::SetEnabled, m_networkPhysicsGrabComponentEnabled);
    }

    void PhysicsGrabComponent::NetworkPhysicsGrabComponentEnabledIgnoreInputs()
    {
        InputEventNotificationBus::MultiHandler::BusDisconnect();
    }

    void PhysicsGrabComponent::IsAutonomousSoConnect()
    {
        AZ::TickBus::Handler::BusConnect();
    }

    void PhysicsGrabComponent::NotAutonomousSoDisconnect()
    {
        AZ::TickBus::Handler::BusDisconnect();
        m_attachedSceneHandle = AzPhysics::InvalidSceneHandle;
        m_sceneSimulationStartHandler.Disconnect();
        m_sceneSimulationFinishHandler.Disconnect();
    }
} // namespace PhysicsGrab
