@LAZYGLOBAL OFF.

////////////////////////////////////////////////////////////////////////////////
//// LAUNCH PARAMETERS                                                      ////
////////////////////////////////////////////////////////////////////////////////

DECLARE GLOBAL launchToApoapsis                  IS   80.0. // km
DECLARE GLOBAL launchToPeriapsis                 IS   80.0. // km
DECLARE GLOBAL launchToInclination               IS    0.0. // 0° – 180°

// For inclinations between 0° and 90° (equatorial and polar), launching at the
// ascending node means launching northeast; launching at the descending node
// means launching southeast. Since launching southeast from the Kerbal Space
// Center takes us over the ocean, all things being equal we choose to launch
// at the descending node. Make sure you remember this when setting up your
// Kerbal Alarm Clock alarm.
DECLARE GLOBAL launchAtAscendingNode             IS  FALSE.

// Pause the game when the program ends. Useful for unattended launches.
DECLARE GLOBAL pauseGameWhenFinished             IS   TRUE.

// If useTimeWarp is true, we'll use 2× physics warp during those ascent modes
// where it's safe. Ish. Use at your own risk.
DECLARE GLOBAL useTimeWarp                       IS  FALSE.

// Debugging prints more messages to the screen.
DECLARE GLOBAL useDebugging                      IS  FALSE.

////////////////////////////////////////////////////////////////////////////////
//// TUNABLE PARAMETERS                                                     ////
////////////////////////////////////////////////////////////////////////////////

// The pitch program end altitude is the altitude, in meters, at which we
// level out to a pitch of 0° off the horizontal. On Kerbin it's about
// 59,000 meters above sea level.
DECLARE GLOBAL pitchProgramEndAltitude           IS SHIP:BODY:ATM:HEIGHT * 0.85.

// This parameter controls the shape of the ascent curve. Its value has been
// determined more-or-less empirically.
DECLARE GLOBAL pitchProgramExponent              IS  0.7.

// We maintain this time to apoapsis (in seconds) during ascent. This is a crude
// but effective way of keeping our angle of attack as small as possible while
// still staying on a useful trajectory.
DECLARE GLOBAL targetTimeToApoapsis              IS 45.0.

// Dynamic pressure, in atmospheres, at which to jettison the payload fairings.
DECLARE GLOBAL fairingJettisonQ                  IS  0.001.

////////////////////////////////////////////////////////////////////////////////
//// INTERNAL VARIABLES                                                     ////
////////////////////////////////////////////////////////////////////////////////

// Convert kilometers to meters.
SET            launchToApoapsis                  TO launchToApoapsis  * 1_000.0.
SET            launchToPeriapsis                 TO launchToPeriapsis * 1_000.0.

// Compute target semimajor axis from given apoapsis and periapsis.
DECLARE GLOBAL launchToSemimajorAxis             IS SHIP:BODY:RADIUS + (launchToPeriapsis + launchToApoapsis) / 2.

// We will calculate the launch azimuth from the desired inclination in
// MODE PRELAUNCH.
DECLARE GLOBAL launchAzimuth                     IS  0.0.

// You might want to enable RCS on the launchpad before running this program. If
// so, we'll respect that.
DECLARE GLOBAL initialRCSState                   IS RCS.

// These are used to control the vehicle's throttle and steering.
DECLARE GLOBAL controlThrottle                   IS  0.0. // 0% – 100%
DECLARE GLOBAL controlPitch                      IS  0.0. // ° above the horizon

// This varies depending on which mode we're in.
DECLARE GLOBAL gForceLimit                       IS  0.0.

// Keep track of which stage we're on.
DECLARE GLOBAL currentStage                      IS  0.

// And which stages actually exist.
DECLARE GLOBAL stageOneExists                    IS FALSE.
DECLARE GLOBAL stageOneSideBoosterExists         IS FALSE.
DECLARE GLOBAL stageTwoExists                    IS FALSE.

// These lists of parts are iterated over at various points in the program.
DECLARE GLOBAL enginesStageOneAll                IS LIST().
DECLARE GLOBAL enginesStageOneAllThrottleable    IS LIST().
DECLARE GLOBAL enginesStageOneAllThrottleLocked  IS LIST().

DECLARE GLOBAL enginesStageOneCoreAll            IS LIST().
DECLARE GLOBAL enginesStageOneParallelAll        IS LIST().

DECLARE GLOBAL enginesStageTwoAll                IS LIST().
DECLARE GLOBAL enginesStageTwoAllThrottleable    IS LIST().
DECLARE GLOBAL enginesStageTwoAllThrottleLocked  IS LIST().

DECLARE GLOBAL payloadFairingsAll                IS QUEUE().

DECLARE GLOBAL enginesRunningAllThrottleable     IS LIST().
DECLARE GLOBAL enginesRunningAllThrottleLocked   IS LIST().

// PID controllers. Coefficients have been determined empirically.
DECLARE GLOBAL throttlePIDController IS PIDLOOP(0.250, 0.200, 0.000, 0.1, 1.0).
DECLARE GLOBAL rcsForePIDController  IS PIDLOOP(0.001, 0.001, 0.000, 0.0, 1.0).

// These simulate hardware registers. They can be used to hold bits of data that
// need to persist across function calls but which don't need their own global
// variables to hold them.
DECLARE GLOBAL register1                         IS  0.0.
DECLARE GLOBAL register2                         IS  0.0.
DECLARE GLOBAL register3                         IS  0.0.

// This is kind of a hack to damp roll oscillations. It's not 100% clear why
// this is needed or why it works.
SET STEERINGMANAGER:ROLLTS                       TO  5.0.

////////////////////////////////////////////////////////////////////////////////
//// FLIGHT STATE                                                           ////
////////////////////////////////////////////////////////////////////////////////

DECLARE GLOBAL tPrevious                         IS TIME(0).
DECLARE GLOBAL Δt                                IS TIME(0).

// This is a proxy variable for SHIP:SENSORS:ACC. Trying to get that value when
// there's no sensor aboard results in a program crash, so we check for the
// existence of the sensor explicitly. If it's not there, we lock this variable
// to the zero vector.
LOCK           accelerometerReading              TO V(0.0, 0.0, 0.0).

// The component of the vessel's proper acceleration in the forward direction.
// This is a scalar in m/s².
DECLARE GLOBAL flightState_α_long                IS 0.0.

// Maximum dynamic pressure experienced by the vehicle during ascent, in
// atmospheres.
DECLARE GLOBAL flightState_MaxQ                  IS 0.0.

// An estimate of aerodynamic drag based on the difference between the expected
// acceleration (from total thrust and vehicle mass) and the actual acceleration
// (measured by the on-board accelerometer). We compute this as a moving average
// to keep the vehicle from getting into a throttle-control feedback loop.
//
// FD_long = longitudinal component of drag acceleration in m/s².
//
DECLARE GLOBAL flightState_dragWindow            IS 5.
DECLARE GLOBAL flightState_dragData              IS QUEUE().
DECLARE GLOBAL flightState_FD_long               IS 0.0.

// Total available thrust, in kilonewtons, from throttleable engines and engines
// which are not throttleable, like solid-rocket boosters.
//
// ΣF_t = total thrust from throttleable engines
// ΣF_l = total thrust from non-throttleable ("locked") engines
//
DECLARE GLOBAL flightState_ΣF_t                  IS 0.0.
DECLARE GLOBAL flightState_ΣF_l                  IS 0.0.

// Δv calculations, including total Δv expended, Δv gained as orbital velocity
// and a measure of efficiency that's the ratio of the two.
DECLARE GLOBAL flightState_ΔvExpended            IS 0.0.
DECLARE GLOBAL flightState_ΔvGained              IS 0.0.
DECLARE GLOBAL flightState_ΔvEfficiency          IS 0.0.

// Run this once at program startup.

DECLARE GLOBAL FUNCTION InitializeFlightState {

    // Do we have an accelerometer part on board? If not, we just leave the
    // proxy variable locked to the zero vector.

    DECLARE LOCAL allSensors IS LIST().

    LIST SENSORS IN allSensors.
    FOR s in allSensors {
        IF s:TYPE = "ACC" {
            LOCK accelerometerReading TO SHIP:SENSORS:ACC.
            BREAK.
        }
    }

    // Push a bunch of zeros onto the drag average calculation queue. This
    // makes it simple to compute a moving average from time t = 0 without
    // having that average go all crazy from noise in the first few fractions
    // of a second.

    UNTIL flightState_dragData:LENGTH = flightState_dragWindow {
        flightState_dragData:PUSH(0.0).
    }

    RETURN.
}

// Update the flight state variables every time we go through the main loop.
// Important note: This function should be as efficient as possible since it
// gets called every time through.

DECLARE GLOBAL FUNCTION UpdateFlightState {

    SET Δt TO TIME - tPrevious.
    SET tPrevious TO TIME.

    // Compute the vessel's current proper acceleration (α), then the component
    // of α in the vehicle's forward direction (α_long).

    DECLARE LOCAL flightState_α IS accelerometerReading - (-1 * SHIP:UP:FOREVECTOR * (SHIP:BODY:MU / (SHIP:ALTITUDE + SHIP:BODY:RADIUS)^2)).
    SET flightState_α_long TO VDOT(flightState_α, SHIP:FACING:FOREVECTOR).

    // Compute a moving average of the estimated aerodynamic drag on the
    // vehicle. We estimate the drag by calculating what our acceleration
    // should be given the thrust we're producing and the vehicle's current
    // mass, then taking the difference between the expected acceleration and
    // the actual, measured acceleration. This is not 100% accurate because
    // it assumes that our thrust vector is antiparallel to our facing vector
    // at all times, which isn't true when our engines gimbal. But it's close
    // enough.

    IF SHIP:ALTITUDE <= SHIP:BODY:ATM:HEIGHT {
        DECLARE LOCAL αExpected IS ((flightState_ΣF_l + THROTTLE * flightState_ΣF_t) / SHIP:MASS) * SHIP:FACING:FOREVECTOR.
        DECLARE LOCAL drag IS αExpected - flightState_α.
        DECLARE LOCAL drag_long IS MIN(0.0, (-1) * VDOT(drag, SHIP:FACING:FOREVECTOR)).
        flightState_dragData:PUSH(drag_long).
        SET flightState_FD_long TO flightState_FD_long + drag_long/flightState_dragWindow - flightState_dragData:POP()/flightState_dragWindow.
    } ELSE {
        SET flightState_FD_long TO 0.0.
    }

    // Keep track of the maximum dynamic pressure on the vehicle. When we've
    // passed the point of maximum dynamic pressure, we start looking for a
    // point where it's safe to jettison the payload fairings, if any.

    IF SHIP:Q > flightState_MaxQ {
        SET flightState_MaxQ TO SHIP:Q.
    }

    // Compute the total available thrust from both throttleable and
    // non-throttleable engines. These figures are used to command the
    // throttle to produce a given thrust and also to compute expended Δv.

    SET flightState_ΣF_t TO 0.0.
    SET flightState_ΣF_l TO 0.0.

    IF enginesRunningAllThrottleLocked:EMPTY {
        SET flightState_ΣF_t TO SHIP:AVAILABLETHRUST.
    } ELSE {
        FOR e IN enginesRunningAllThrottleable {
            SET flightState_ΣF_t TO flightState_ΣF_t + e:AVAILABLETHRUST.
        }
        FOR e IN enginesRunningAllThrottleLocked {
            SET flightState_ΣF_l TO flightState_ΣF_l + e:AVAILABLETHRUST.
        }
    }

    // Integrate total expended Δv, where Δv = Δv + ΣF × Δt.

    SET flightState_ΔvExpended TO flightState_ΔvExpended + ((flightState_ΣF_l + THROTTLE * flightState_ΣF_t) / SHIP:MASS) * Δt:SECONDS.

    // Calculate Δv gained. Note that this is only Δv gained in the actual
    // direction we want to go, which is tangent to the surface in the direction
    // of our launch azimuth. If we're launching into a prograde equatorial
    // orbit from the Kerbal Space Center, we get a free 175 m/s; contrariwise,
    // if we're launching retrograde, we start out at -175 m/s and have to
    // accelerate up to zero before we can start making headway.

    SET flightState_ΔvGained TO (VDOT(SHIP:VELOCITY:ORBIT, HEADING(launchAzimuth, 0.0):FOREVECTOR)).

    // Calculate flight efficiency as a ratio of Δv gained to Δv expended.

    IF flightState_ΔvExpended > 0 AND flightState_ΔvExpended > flightState_ΔvGained {
        SET flightState_ΔvEfficiency TO MIN(1.0, MAX(0.0, flightState_ΔvGained / flightState_ΔvExpended)).
    }

    RETURN.
}

////////////////////////////////////////////////////////////////////////////////
////                                                                        ////
//// STATE MACHINE                                                          ////
////                                                                        ////
//// The heart of this program is a finite-state machine.                   ////
////                                                                        ////
//// At any given time, the program is in one of a number of modes (i.e.,   ////
//// states). Each mode has a transition-in function, a loop function and   ////
//// possibly a transition-out function. The transition-in function is      ////
//// called once when the machine enters that mode; the loop function is    ////
//// called over and over, once every time through the main program loop.   ////
//// If there's a transition-out function, it's called once as we change to ////
//// another mode.                                                          ////
////                                                                        ////
//// Note carefully that mode loop functions need to be as efficient as     ////
//// possible since they get called very frequently.                        ////
////                                                                        ////
////////////////////////////////////////////////////////////////////////////////

DECLARE GLOBAL StateFunction IS {}.
DECLARE GLOBAL ModeName IS "".

////////////////////////////////////////////////////////////////////////////////
// MODE PRELAUNCH //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

DECLARE GLOBAL FUNCTION ModePrelaunchTransitionInFunction {
    SET ModeName TO "PRELAUNCH".
    MissionLog("MODE TO " + ModeName).

    // Calculate the launch azimuth from the desired orbital inclination. Start
    // by clamping the orbital inclination to the latitude of the launch site.
    // An alternative would be to throw an error when launchToInclination is
    // out of range, but this would be annoying because we will often want to
    // launch to an equatorial orbit (i = 0.000°) from KSC (latitude = -0.097°).
    // It's just easier to set it up so an input inclination of 0° does the
    // right thing, putting us into an orbit that's near enough makes no
    // difference equatorial. A bonus is that putting in a desired inclination
    // of 180° also works, yielding an orbit which is very, very close to
    // retrograde.
    //
    // If you're not launching from the equator, then it's up to you to know
    // that the minimum inclination reachable from some latitude φ is just
    // φ itself.

    IF launchToInclination < ABS(SHIP:LATITUDE) {
        SET launchToInclination TO SHIP:LATITUDE.
    }

    IF launchToInclination > 180 - ABS(SHIP:LATITUDE) {
        SET launchToInclination TO 180 - ABS(SHIP:LATITUDE).
    }

    // This is what our launch azimuth WOULD be if the planet weren't moving.

    DECLARE LOCAL launchAzimuthInertial IS ARCSIN( COS(launchToInclination) / COS(SHIP:LATITUDE) ).

    // To compensate for the rotation of the planet, we need to estimate the
    // orbital velocity we're going to gain during ascent. We can ballpark this
    // by assuming a circular orbit at the pitch program end altitude, but there
    // will necessarily be some very small error. We will correct this error
    // during MODE RAISE APOAPSIS.

    DECLARE LOCAL vApproximate IS SQRT( SHIP:BODY:MU / ( SHIP:BODY:RADIUS + pitchProgramEndAltitude ) ).

    // Construct the components of the launch vector in the rotating frame. In
    // this frame the x axis points north and the y axis points east. (Which is
    // backwards from what you'd intuitively expect, yes.)

    DECLARE LOCAL x IS vApproximate * COS(launchAzimuthInertial).
    DECLARE LOCAL y IS vApproximate * SIN(launchAzimuthInertial) - SHIP:VELOCITY:ORBIT:MAG * COS(SHIP:LATITUDE).

    IF NOT launchAtAscendingNode {
        SET x TO (-1) * x.
    }

    // Finally, the launch azimuth is the angle of the launch vector we just
    // computed measured from the x (north) axis. If our launch vector pointed
    // due east, as it would when our desired inclination is 0°, then the
    // y component of the vector would be zero and ATAN2(y, x) would also
    // be zero.

    SET launchAzimuth TO ARCTAN2(y, x).

    ////////////////////////////////////////////////////////////////////////////

    // Height above the ground in prelaunch. Used to figure out when we've
    // lifted off and cleared moorings.

    SET register1 TO ALT:RADAR.

    ////////////////////////////////////////////////////////////////////////////

    // Infer some things about how the vehicle is constructed.

    DECLARE LOCAL allParts IS LIST().
    LIST PARTS IN allParts.

    DECLARE LOCAL allEngines IS LIST().
    LIST ENGINES IN allEngines.

    // A temporary variable to be a pointer to some part on the vehicle.

    DECLARE LOCAL p IS 0.

    // KSP uses both 0 and -1 as stage numbers, so we have to pick something
    // that the game would never use.

    DECLARE LOCAL kspS1StageNumber IS -2.
    DECLARE LOCAL kspS2StageNumber IS -2.

    // Assume that the first stage is the one that'll be activated as soon as
    // the player presses the space bar.

    SET kspS1StageNumber TO STAGE:NUMBER - 1.

    // Find all engines on the vehicle that are in stage one.

    FOR i IN allEngines {
        IF i:STAGE = kspS1StageNumber {
            enginesStageOneAll:ADD(i).
        }
    }

    // If we can't find any engines on the first stage, this program can't do
    // anything, so we just skip to the end.

    IF enginesStageOneAll:EMPTY {
        SET StateFunction TO ModeEndProgramTransitionInFunction@.
        RETURN.
    } ELSE {
        SET stageOneExists TO TRUE.
    }

    // Separate the engines on stage one into lists of throttleable and non-
    // throttleable engines.

    FOR i IN enginesStageOneAll {
        IF i:THROTTLELOCK {
            enginesStageOneAllThrottleLocked:ADD(i).
        } ELSE {
            enginesStageOneAllThrottleable:ADD(i).
        }
    }

    // Figure out whether each engine on the first stage is part of the core
    // booster or a parallel (i.e., strap-on) booster. This part of the program
    // depends on the fact that stack decouplers have "ModuleDecouple" while
    // radial ones have "ModuleAnchoredDecoupler." Unfortunately this makes
    // this part of the program is rather fragile.

    FOR i IN enginesStageOneAll {
        SET p TO i.
        UNTIL p = SHIP:ROOTPART OR p:HASMODULE("ModuleDecouple") OR p:HASMODULE("ModuleAnchoredDecoupler") {
            SET p TO p:PARENT.

            IF p = SHIP:ROOTPART OR p:HASMODULE("ModuleDecouple") {
                enginesStageOneCoreAll:ADD(i).
            }

            IF p:HASMODULE("ModuleAnchoredDecoupler") {
                enginesStageOneParallelAll:ADD(i).
            }
        }
    }

    IF NOT enginesStageOneParallelAll:EMPTY {
        SET stageOneSideBoosterExists TO TRUE.
    }

    // Pick any stage-one engine on the core booster and walk up the part tree
    // until we find a stack decoupler. Assume the stage above this decoupler's
    // stage is stage two.

    SET p TO enginesStageOneCoreAll[0].
    UNTIL p = SHIP:ROOTPART OR p:HASMODULE("ModuleDecouple") {
        SET p TO p:PARENT.
    }

    IF p:HASMODULE("ModuleDecouple") {
        SET kspS2StageNumber TO p:STAGE - 1.
    }

    // Find all engines on the vehicle that are in stage two.

    FOR i IN allEngines {
        IF i:STAGE = kspS2StageNumber {
            enginesStageTwoAll:ADD(i).
        }
    }

    IF NOT enginesStageTwoAll:EMPTY {
        SET stageTwoExists TO TRUE.
    }

    // Separate the engines on stage two into lists of throttleable and non-
    // throttleable engines.

    FOR i IN enginesStageTwoAll {
        IF i:THROTTLELOCK {
            enginesStageTwoAllThrottleLocked:ADD(i).
        } ELSE {
            enginesStageTwoAllThrottleable:ADD(i).
        }
    }

    // Find all the jettison-able payload fairings on the vehicle. We will
    // jettison these during ascent at a predetermined dynamic pressure. See
    // TUNABLE PARAMETERS.

    FOR i IN allParts {
        IF i:HASMODULE("ModuleProceduralFairing") AND i:GETMODULE("ModuleProceduralFairing"):HASEVENT("deploy") {
            payloadFairingsAll:PUSH(i).
        }
        IF i:HASMODULE("ProceduralFairingDecoupler") AND i:GETMODULE("ProceduralFairingDecoupler"):HASEVENT("jettison fairing") {
            payloadFairingsAll:PUSH(i).
        }
    }

    ////////////////////////////////////////////////////////////////////////////

    SET StateFunction TO ModeS1IgnitionTransitionInFunction@.
    RETURN.
}

// Not currently used.

// DECLARE GLOBAL FUNCTION ModePrelaunchLoopFunction {
//
//     // If we were going to implement some kind of fancy countdown feature, this
//     // is where we'd do it. There's little point in that, though, since once
//     // you stage for the first time everything happens in a single instant.
//
//     SET StateFunction TO ModeS1IgnitionTransitionInFunction@.
//     RETURN.
// }

// Not currently used.

// DECLARE GLOBAL FUNCTION ModePrelaunchTransitionOutFunction {
//
//     SET StateFunction TO {}.
//     RETURN.
// }

////////////////////////////////////////////////////////////////////////////////
// MODE IGNITION ///////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

DECLARE GLOBAL FUNCTION ModeS1IgnitionTransitionInFunction {
    SET ModeName TO "IGNITION".
    MissionLog("MODE TO " + ModeName).

    // Set the throttle to an initial setting that respects our g-force limit.
    // As soon as we start moving this will need to be adjusted to compensate
    // for aerodynamic drag on the vehicle.

    DECLARE LOCAL ΣF_t IS 0.0.
    FOR e IN enginesStageOneAllThrottleable {
        SET ΣF_t TO ΣF_t + e:POSSIBLETHRUST.
    }

    DECLARE LOCAL ΣF_l IS 0.0.
    FOR e IN enginesStageOneAllThrottleLocked {
        SET ΣF_l TO ΣF_l + e:POSSIBLETHRUST.
    }

    SET gForceLimit TO 1.5.
    SET controlThrottle TO MIN(1.0, MAX(0.0, (SHIP:MASS * (gForceLimit * CONSTANT:g0)) - ΣF_l)/ΣF_t).
    LOCK THROTTLE TO controlThrottle.

    // Don't try to maneuver during liftoff.

    LOCK STEERING TO "KILL".

    SET StateFunction TO ModeS1IgnitionLoopFunction@.
    RETURN.
}

DECLARE GLOBAL FUNCTION ModeS1IgnitionLoopFunction {

    IF STAGE:READY {
        STAGE.
        SET currentStage TO 1.
        SET StateFunction TO ModeS1IgnitionTransitionOutFunction@.
        RETURN.
    }

    RETURN.
}

DECLARE GLOBAL FUNCTION ModeS1IgnitionTransitionOutFunction {

    // Wait for all stage-one engines to ignite. Note that if one or more
    // fails to ignite, then we get stuck here forever.

    DECLARE LOCAL enginesIgnited IS TRUE.
    FOR e IN enginesStageOneAll {
        SET enginesIgnited TO enginesIgnited AND e:IGNITION.
    }

    IF NOT enginesIgnited {
        RETURN.
    }

    enginesRunningAllThrottleable:CLEAR().
    enginesRunningAllThrottleLocked:CLEAR().
    FOR e IN enginesStageOneAll {
        IF e:THROTTLELOCK {
            enginesRunningAllThrottleLocked:ADD(e).
        } ELSE {
            enginesRunningAllThrottleable:ADD(e).
        }
    }

    SET StateFunction TO ModeLiftoffTransitionInFunction@.
    RETURN.
}

////////////////////////////////////////////////////////////////////////////////
// MODE LIFTOFF ////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

DECLARE GLOBAL FUNCTION ModeLiftoffTransitionInFunction {
    SET ModeName TO "LIFTOFF".
    MissionLog("MODE TO " + ModeName).

    SET gForceLimit TO 1.5.
    SET controlThrottle TO MIN(1.0, MAX(0.0, (SHIP:MASS * (gForceLimit * CONSTANT:g0)) - flightState_ΣF_l)/flightState_ΣF_t).
    LOCK THROTTLE TO controlThrottle.

    LOCK STEERING TO "KILL".

    SET StateFunction TO ModeLiftoffLoopFunction@.
    RETURN.
}

DECLARE GLOBAL FUNCTION ModeLiftoffLoopFunction {

    SET controlThrottle TO MIN(1.0, MAX(0.0, (SHIP:MASS * (gForceLimit * CONSTANT:g0)) - flightState_ΣF_l)/flightState_ΣF_t).

    // Wait until we've actually lifted off the pad.

    IF ALT:RADAR >= register1 {
        SET StateFunction TO ModeAscentProgramTransitionInFunction@.
        RETURN.
    }

    RETURN.
}

// Not currently used.

// DECLARE GLOBAL FUNCTION ModeLiftoffTransitionOutFunction {
//
//     SET StateFunction TO {}.
//     RETURN.
// }

////////////////////////////////////////////////////////////////////////////////
// MODE ASCENT PROGRAM /////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

DECLARE GLOBAL FUNCTION ModeAscentProgramTransitionInFunction {
    SET ModeName TO "ASCENT PROGRAM".
    MissionLog("MODE TO " + ModeName).

    SET gForceLimit TO 1.5.
    SET controlThrottle TO MIN(1.0, MAX(0.0, (SHIP:MASS * (gForceLimit * CONSTANT:g0)) - flightState_ΣF_l)/flightState_ΣF_t).
    LOCK THROTTLE TO controlThrottle.

    LOCK STEERING TO "KILL".

    SET StateFunction TO ModeAscentProgramLoopFunction@.
    RETURN.
}

DECLARE GLOBAL FUNCTION ModeAscentProgramLoopFunction {

    SET controlThrottle TO MIN(1.0, MAX(0.0, (SHIP:MASS * (gForceLimit * CONSTANT:g0)) - flightState_ΣF_l)/flightState_ΣF_t).

    // Wait until the spacecraft clears its own height.
    IF ALT:RADAR > register1 * 2 {
        SET StateFunction TO ModeRollProgramTransitionInFunction@.
        RETURN.
    }

    RETURN.
}

// Not currently used.

// DECLARE GLOBAL FUNCTION ModeAscentProgramTransitionOutFunction {
//
//     SET StateFunction TO {}.
//     RETURN.
// }

////////////////////////////////////////////////////////////////////////////////
// MODE ROLL PROGRAM ///////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Here we BEGIN rolling such that the vehicle's "up" points in the opposite
// direction of the launch azimuth. Note that how soon this roll maneuver
// finishes is a function of vehicle torque, so the maneuver may not finish
// before we enter the pitch program. This has been found to be okay. If the
// vehicle doesn't finish the roll maneuver before the pitch program starts, the
// vehicle will just pitch and roll at the same time.

DECLARE GLOBAL FUNCTION ModeRollProgramTransitionInFunction {
    SET ModeName TO "ROLL PROGRAM".
    MissionLog("MODE TO " + ModeName).

    SET controlPitch TO 90.0.
    LOCK STEERING TO HEADING(launchAzimuth, controlPitch).

    SET gForceLimit TO 1.5.
    SET controlThrottle TO MIN(1.0, MAX(0.0, (SHIP:MASS * (gForceLimit * CONSTANT:g0 - flightState_FD_long)) - flightState_ΣF_l)/flightState_ΣF_t).
    LOCK THROTTLE to controlThrottle.

    SET StateFunction TO ModeRollProgramLoopFunction@.
    RETURN.
}

DECLARE GLOBAL FUNCTION ModeRollProgramLoopFunction {

    SET controlThrottle TO MIN(1.0, MAX(0.0, (SHIP:MASS * (gForceLimit * CONSTANT:g0 - flightState_FD_long)) - flightState_ΣF_l)/flightState_ΣF_t).

    // Wait until we reach 50 meters per second. This is a "magic number,"
    // determined pretty much empirically.

    IF SHIP:AIRSPEED > 50.0 {
        SET StateFunction TO ModePitchProgramTransitionInFunction@.
        RETURN.
    }

    RETURN.
}

// Not currently used.

// DECLARE GLOBAL FUNCTION ModeRollProgramTransitionOutFunction {
//
//     SET StateFunction TO {}.
//     RETURN.
// }

////////////////////////////////////////////////////////////////////////////////
// MODE PITCH PROGRAM //////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// During the pitch program we command the vehicle to pitch down (toward the
// horizon) at a rate proportional to our vertical speed. Pitch angle is a
// function of altitude: θ = altitudeⁿ, where altitude is represented as a
// fraction of the pitch program end altitude.

DECLARE GLOBAL FUNCTION ModePitchProgramTransitionInFunction {
    SET ModeName TO "PITCH PROGRAM".
    MissionLog("MODE TO " + ModeName).

    SET controlPitch TO MAX( 0.0, 90.0 - 90.0 * (SHIP:ALTITUDE / pitchProgramEndAltitude) ^ pitchProgramExponent ).
    LOCK STEERING TO HEADING(launchAzimuth, controlPitch).

    SET gForceLimit TO ( 1.5 * ( 1.0 - (90.0 - controlPitch)/90.0) ) + ( 2.0 * ((90.0 - controlPitch)/90.0) ).
    SET throttlePIDController:SETPOINT TO targetTimeToApoapsis.
    SET throttlePIDController:MAXOUTPUT TO MIN(1.0, MAX(0.0, (SHIP:MASS * (gForceLimit * CONSTANT:g0 - flightState_FD_long)) - flightState_ΣF_l)/flightState_ΣF_t).
    SET controlThrottle TO throttlePIDController:UPDATE(TIME:SECONDS, ETA:APOAPSIS).
    LOCK THROTTLE TO controlThrottle.

    StartTimeWarp().

    SET StateFunction TO ModePitchProgramLoopFunction@.
    RETURN.
}

DECLARE GLOBAL FUNCTION ModePitchProgramLoopFunction {

    // Trigger fairing jettison if we're past max. Q AND we're at the threshold
    // dynamic pressure.

    IF NOT payloadFairingsAll:EMPTY AND SHIP:Q < flightState_MaxQ AND SHIP:Q < fairingJettisonQ {

        MissionLog("FAIRING JETTISON").

        UNTIL payloadFairingsAll:EMPTY {
            DECLARE LOCAL f IS payloadFairingsAll:POP().
            IF f:HASMODULE("ModuleProceduralFairing") {
                f:GETMODULE("ModuleProceduralFairing"):DOEVENT("deploy").
            }
            IF f:HASMODULE("ProceduralFairingDecoupler") {
                f:GETMODULE("ProceduralFairingDecoupler"):DOEVENT("jettison fairing").
            }
        }
    }

    IF currentStage = 1 AND SHIP:MAXTHRUST = 0 {
        SET StateFunction TO ModeS1ShutdownTransitionInFunction@.
        RETURN.
    }

    IF currentStage = 2 AND SHIP:MAXTHRUST = 0 {
        SET StateFunction TO ModeS2ShutdownTransitionInFunction@.
        RETURN.
    }

    IF currentStage = 1 AND SHIP:ALTITUDE >= pitchProgramEndAltitude AND stageTwoExists {
        SET StateFunction TO ModeS1ShutdownTransitionInFunction@.
        RETURN.
    }

    IF currentStage = 1 AND SHIP:ALTITUDE >= pitchProgramEndAltitude AND SHIP:APOAPSIS < launchToApoapsis AND NOT stageTwoExists {
        SET StateFunction TO ModeRaiseApoapsisTransitionInFunction@.
        RETURN.
    }

    IF currentStage = 1 AND SHIP:ALTITUDE >= pitchProgramEndAltitude AND SHIP:APOAPSIS >= launchToApoapsis AND NOT stageTwoExists {
        SET StateFunction TO ModePoweredCoastTransitionInFunction@.
        RETURN.
    }

    IF currentStage = 2 AND SHIP:ALTITUDE >= pitchProgramEndAltitude AND SHIP:APOAPSIS >= launchToApoapsis {
        SET StateFunction TO ModePoweredCoastTransitionInFunction@.
        RETURN.
    }

    IF currentStage = 2 AND SHIP:ALTITUDE < pitchProgramEndAltitude AND SHIP:APOAPSIS >= launchToApoapsis {
        SET StateFunction TO ModePoweredCoastTransitionInFunction@.
        RETURN.
    }

    SET controlPitch TO MAX( 0.0, 90.0 - 90.0 * (SHIP:ALTITUDE / pitchProgramEndAltitude) ^ pitchProgramExponent ).

    SET gForceLimit TO ( 1.5 * ( 1.0 - (90.0 - controlPitch)/90.0) ) + ( 2.0 * ((90.0 - controlPitch)/90.0) ).
    SET throttlePIDController:MAXOUTPUT TO MIN(1.0, MAX(0.0, (SHIP:MASS * (gForceLimit * CONSTANT:g0 - flightState_FD_long)) - flightState_ΣF_l)/flightState_ΣF_t).
    SET controlThrottle TO throttlePIDController:UPDATE(TIME:SECONDS, ETA:APOAPSIS).

    // If ANY of the side-booster engines flames out, shut down all the engines
    // and jettison the boosters. This may result in a failed launch if the
    // vehicle doesn't have enough Δv in the core and second stages to make up
    // the difference, but it's better than just letting the vehicle spin out
    // of control.

    IF currentStage = 1 AND stageOneSideBoosterExists AND STAGE:READY {

        FOR e IN enginesStageOneParallelAll {
            IF NOT e:IGNITION OR e:FLAMEOUT {
                SET StateFunction TO ModeSideBoosterShutdownTransitionInFunction@.
                RETURN.
            }
        }

        RETURN.
    }

    RETURN.
}

// Not currently used.

// DECLARE GLOBAL FUNCTION ModePitchProgramTransitionOutFunction {
//
//     SET StateFunction TO {}.
//     RETURN.
// }

////////////////////////////////////////////////////////////////////////////////
// MODE SIDE BOOSTER SHUTDOWN //////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// The parallel stages have shut down and it's time to jettison them.

DECLARE GLOBAL FUNCTION ModeSideBoosterShutdownTransitionInFunction {
    SET ModeName TO "SIDE BOOSTER SHUTDOWN".
    MissionLog("MODE TO " + ModeName).

    StopTimeWarp().

    FOR e IN enginesStageOneParallelAll {
        e:SHUTDOWN.
    }

    enginesRunningAllThrottleable:CLEAR().
    enginesRunningAllThrottleLocked:CLEAR().
    FOR e IN enginesStageOneCoreAll {
        IF e:THROTTLELOCK {
            enginesRunningAllThrottleLocked:ADD(e).
        } ELSE {
            enginesRunningAllThrottleable:ADD(e).
        }
    }

    SET controlPitch TO MAX( 0.0, 90.0 - 90.0 * (SHIP:ALTITUDE / pitchProgramEndAltitude) ^ pitchProgramExponent ).

    SET register1 TO TIME.

    SET StateFunction TO ModeSideBoosterShutdownLoopFunction@.
    RETURN.
}

DECLARE GLOBAL FUNCTION ModeSideBoosterShutdownLoopFunction {

    SET controlPitch TO MAX( 0.0, 90.0 - 90.0 * (SHIP:ALTITUDE / pitchProgramEndAltitude) ^ pitchProgramExponent ).

    IF TIME:SECONDS >= register1:SECONDS + 2.0 {
        SET StateFunction TO ModeSideBoosterSeparationTransitionInFunction@.
        RETURN.
    }

    RETURN.
}

// Not currently used.

// DECLARE GLOBAL FUNCTION ModeSideBoosterShutdownTransitionOutFunction {
//
//     SET StateFunction TO {}.
//     RETURN.
// }

////////////////////////////////////////////////////////////////////////////////
// MODE SIDE BOOSTER SEPARATION ////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

DECLARE GLOBAL FUNCTION ModeSideBoosterSeparationTransitionInFunction {
    SET ModeName TO "SIDE BOOSTER SEPARATION".
    MissionLog("MODE TO " + ModeName).

    StopTimeWarp().

    SET controlPitch TO MAX( 0.0, 90.0 - 90.0 * (SHIP:ALTITUDE / pitchProgramEndAltitude) ^ pitchProgramExponent ).

    IF STAGE:READY {

        SET register1 TO TIME.

        STAGE.

        enginesStageOneParallelAll:CLEAR().
        SET enginesStageOneAll TO enginesStageOneCoreAll.

        enginesRunningAllThrottleable:CLEAR().
        enginesRunningAllThrottleLocked:CLEAR().
        FOR e IN enginesStageOneAll {
            IF e:THROTTLELOCK {
                enginesRunningAllThrottleLocked:ADD(e).
            } ELSE {
                enginesRunningAllThrottleable:ADD(e).
            }
        }

        // A common design for rockets with parallel stages is to limit the
        // thrust on the core booster way down so it still has propellant when
        // the side boosters separate, then throttle up after separation.

        FOR e IN enginesStageOneCoreAll {
            SET e:THRUSTLIMIT TO 100.0.
        }

        SET StateFunction TO ModeSideBoosterSeparationLoopFunction@.
        RETURN.
    }

    RETURN.
}

DECLARE GLOBAL FUNCTION ModeSideBoosterSeparationLoopFunction {

    SET controlPitch TO MAX( 0.0, 90.0 - 90.0 * (SHIP:ALTITUDE / pitchProgramEndAltitude) ^ pitchProgramExponent ).

    IF TIME:SECONDS - register1:SECONDS >= 2.0 {
        SET StateFunction TO ModeSideBoosterSeparationTransitionOutFunction@.
        RETURN.
    }

    RETURN.
}

DECLARE GLOBAL FUNCTION ModeSideBoosterSeparationTransitionOutFunction {

    // Our payload fairings may have been jettisoned at side-booster separation.
    // Re-scan the part tree and rebuild the list just in case.

    IF NOT payloadFairingsAll:EMPTY {

        payloadFairingsAll:CLEAR().

        DECLARE LOCAL allParts IS LIST().
        LIST PARTS in allParts.

        FOR p IN allParts {
            IF p:HASMODULE("ModuleProceduralFairing") AND p:GETMODULE("ModuleProceduralFairing"):HASEVENT("deploy") {
                payloadFairingsAll:PUSH(p).
            }
            IF p:HASMODULE("ProceduralFairingDecoupler") AND p:GETMODULE("ProceduralFairingDecoupler"):HASEVENT("jettison fairing") {
                payloadFairingsAll:PUSH(p).
            }
        }
    }

    SET StateFunction TO ModePitchProgramTransitionInFunction@.
    RETURN.
}

////////////////////////////////////////////////////////////////////////////////
// MODE S1 SHUTDOWN ////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

DECLARE GLOBAL FUNCTION ModeS1ShutdownTransitionInFunction {
    SET ModeName TO "S1 SHUTDOWN".
    MissionLog("MODE TO " + ModeName).

    StopTimeWarp().

    enginesRunningAllThrottleable:CLEAR().
    enginesRunningAllThrottleLocked:CLEAR().

    LOCK STEERING TO "KILL".

    SET controlThrottle TO 0.0.
    LOCK THROTTLE TO controlThrottle.

    SET register1 TO TIME.

    SET StateFunction TO ModeS1ShutdownLoopFunction@.
    RETURN.
}

DECLARE GLOBAL FUNCTION ModeS1ShutdownLoopFunction {

    IF THROTTLE = 0.0 AND TIME:SECONDS >= register1:SECONDS + 2.0 {

        IF stageTwoExists {
            SET StateFunction TO ModeS2SeparationTransitionInFunction@.
            RETURN.
        }

        IF SHIP:ALTITUDE < SHIP:BODY:ATM:HEIGHT AND SHIP:APOAPSIS >= launchToApoapsis {
            SET StateFunction TO ModePoweredCoastTransitionInFunction@.
            RETURN.
        }

        IF SHIP:ALTITUDE >= SHIP:BODY:ATM:HEIGHT AND SHIP:APOAPSIS >= launchToApoapsis {
            SET StateFunction TO ModeComputeApoapsisManeuverTransitionInFunction@.
            RETURN.
        }

        SET StateFunction TO ModeEndProgramTransitionInFunction@.
    }

    RETURN.
}

// Not currently used.

// DECLARE GLOBAL FUNCTION ModeS1ShutdownTransitionOutFunction {
//
//     SET StateFunction TO {}.
//     RETURN.
// }

////////////////////////////////////////////////////////////////////////////////
// MODE S2 SEPARATION //////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// During stage two separation one of three things will happen:
//
// * SRBs will fire to push stage two away from stage one.
// * RCS forward thrust will be used to push stage two away from stage one.
// * Neither SRBs nor RCS will be available, in which case we'll coast
//   for five seconds to ensure a clean separation.

DECLARE GLOBAL FUNCTION ModeS2SeparationTransitionInFunction {
    SET ModeName TO "S2 SEPARATION".
    MissionLog("MODE TO " + ModeName).

    IF STAGE:READY {

        SET register1 TO 0.0. // integrated velocity
        SET register2 TO 0.0. // integrated displacement
        SET register3 TO TIME.

        STAGE.

        SET StateFunction TO ModeS2SeparationLoopFunction@.
        RETURN.
    }

    RETURN.
}

DECLARE GLOBAL FUNCTION ModeS2SeparationLoopFunction {

    IF flightState_α_long <= 0.0 {
        RCS ON.
        SET SHIP:CONTROL:FORE TO 1.0.
    }

    SET register1 TO register1 + flightState_α_long * Δt:SECONDS.
    SET register2 TO register2 + register1          * Δt:SECONDS.

    // Wait until EITHER we've moved 5 meters away from stage one OR five
    // seconds have passed.

    IF register2 >= 5.0 OR TIME:SECONDS - register3:SECONDS >= 5.0 {

        SET SHIP:CONTROL:FORE TO 0.0.
        SET SHIP:CONTROL:NEUTRALIZE TO TRUE.
        SET RCS TO initialRCSState.

        SET StateFunction TO ModeS2SeparationTransitionOutFunction@.
        RETURN.
    }

    RETURN.
}

DECLARE GLOBAL FUNCTION ModeS2SeparationTransitionOutFunction {

    // Our payload fairings may have been jettisoned at S2 separation. Re-scan
    // the part tree and rebuild the list just in case.

    IF NOT payloadFairingsAll:EMPTY {

        payloadFairingsAll:CLEAR().

        DECLARE LOCAL allParts IS LIST().
        LIST PARTS in allParts.

        FOR p IN allParts {
            IF p:HASMODULE("ModuleProceduralFairing") AND p:GETMODULE("ModuleProceduralFairing"):HASEVENT("deploy") {
                payloadFairingsAll:PUSH(p).
            }
            IF p:HASMODULE("ProceduralFairingDecoupler") AND p:GETMODULE("ProceduralFairingDecoupler"):HASEVENT("jettison fairing") {
                payloadFairingsAll:PUSH(p).
            }
        }
    }

    SET StateFunction TO ModeS2IgnitionTransitionInFunction@.
    RETURN.
}

////////////////////////////////////////////////////////////////////////////////
// MODE S2 IGNITION ////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

DECLARE GLOBAL FUNCTION ModeS2IgnitionTransitionInFunction {
    SET ModeName TO "S2 IGNITION".
    MissionLog("MODE TO " + ModeName).

    // This is the same logic used in the S1 ignition mode.

    DECLARE LOCAL ΣF_t IS 0.0.
    FOR e IN enginesStageTwoAllThrottleable {
        SET ΣF_t TO ΣF_t + e:POSSIBLETHRUST.
    }

    DECLARE LOCAL ΣF_l IS 0.0.
    FOR e IN enginesStageTwoAllThrottleLocked {
        SET ΣF_l TO ΣF_l + e:POSSIBLETHRUST.
    }

    LOCK STEERING TO "KILL".

    SET gForceLimit TO 1.5.
    SET controlThrottle TO MIN(1.0, MAX(0.0, (SHIP:MASS * (gForceLimit * CONSTANT:g0)) - ΣF_l)/ΣF_t).
    LOCK THROTTLE TO controlThrottle.

    SET StateFunction TO ModeS2IgnitionLoopFunction@.
    RETURN.
}

DECLARE GLOBAL FUNCTION ModeS2IgnitionLoopFunction {

    IF STAGE:READY {
        STAGE.
        SET currentStage TO 2.
        SET StateFunction TO ModeS2IgnitionTransitionOutFunction@.
        RETURN.
    }

    RETURN.
}

DECLARE GLOBAL FUNCTION ModeS2IgnitionTransitionOutFunction {

    // Wait for all stage-two engines to ignite. Note that if one or more
    // fails to ignite, then we get stuck here forever.

    DECLARE LOCAL enginesIgnited IS TRUE.
    FOR e IN enginesStageTwoAll {
        SET enginesIgnited TO enginesIgnited AND e:IGNITION.
    }

    IF NOT enginesIgnited {
        RETURN.
    }

    enginesRunningAllThrottleable:CLEAR().
    enginesRunningAllThrottleLocked:CLEAR().
    FOR e IN enginesStageTwoAll {
        IF e:IGNITION {
            IF e:THROTTLELOCK {
                enginesRunningAllThrottleLocked:ADD(e).
            } ELSE {
                enginesRunningAllThrottleable:ADD(e).
            }
        }
    }

    IF SHIP:ALTITUDE < pitchProgramEndAltitude {
        SET StateFunction TO ModePitchProgramTransitionInFunction@.
        RETURN.
    }

    IF SHIP:ALTITUDE >= pitchProgramEndAltitude AND SHIP:APOAPSIS < launchToApoapsis {
        SET StateFunction TO ModeRaiseApoapsisTransitionInFunction@.
        RETURN.
    }

    IF SHIP:ALTITUDE >= pitchProgramEndAltitude AND SHIP:APOAPSIS >= launchToApoapsis {
        SET StateFunction TO ModePoweredCoastTransitionInFunction@.
        RETURN.
    }

    RETURN.
}

////////////////////////////////////////////////////////////////////////////////
// MODE RAISE APOAPSIS /////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// In this mode we've finished the pitch program but our apoapsis is still too
// low, so we keep burning the second stage until our apoapsis is correct. Along
// the way, we correct the inclination error that resulted from our approximate
// calculation of the launch azimuth.

DECLARE GLOBAL FUNCTION ModeRaiseApoapsisTransitionInFunction {
    SET ModeName TO "RAISE APOAPSIS".
    MissionLog("MODE TO " + ModeName).

    SET controlPitch TO 0.0.
    LOCK STEERING TO HEADING(launchAzimuth, controlPitch).

    SET gForceLimit TO 3.0.
    SET throttlePIDController:SETPOINT TO targetTimeToApoapsis.
    SET throttlePIDController:MAXOUTPUT TO MIN(1.0, MAX(0.0, (SHIP:MASS * (gForceLimit * CONSTANT:g0 - flightState_FD_long)) - flightState_ΣF_l)/flightState_ΣF_t).
    SET controlThrottle TO throttlePIDController:UPDATE(TIME:SECONDS, ETA:APOAPSIS).
    LOCK THROTTLE TO controlThrottle.

    StartTimeWarp().

    SET StateFunction TO ModeRaiseApoapsisLoopFunction@.
    RETURN.
}

DECLARE GLOBAL FUNCTION ModeRaiseApoapsisLoopFunction {

    IF currentStage = 1 AND SHIP:MAXTHRUST = 0 {
        SET StateFunction TO ModeS1ShutdownTransitionInFunction@.
        RETURN.
    }

    IF currentStage = 2 AND SHIP:MAXTHRUST = 0 {
        SET StateFunction TO ModeS2ShutdownTransitionInFunction@.
        RETURN.
    }

    IF currentStage = 1 AND SHIP:APOAPSIS >= launchToApoapsis {
        SET StateFunction TO ModeS1ShutdownTransitionInFunction@.
        RETURN.
    }

    IF currentStage = 2 AND SHIP:APOAPSIS >= launchToApoapsis {
        SET StateFunction TO ModeS2ShutdownTransitionInFunction@.
        RETURN.
    }

    // Trigger fairing jettison if we're past max. Q AND we're at the threshold
    // dynamic pressure.

    IF NOT payloadFairingsAll:EMPTY AND SHIP:Q < flightState_MaxQ AND SHIP:Q < fairingJettisonQ {

        MissionLog("FAIRING JETTISON").

        UNTIL payloadFairingsAll:EMPTY {
            DECLARE LOCAL f IS payloadFairingsAll:POP().
            IF f:HASMODULE("ModuleProceduralFairing") {
                f:GETMODULE("ModuleProceduralFairing"):DOEVENT("deploy").
            }
            IF f:HASMODULE("ProceduralFairingDecoupler") {
                f:GETMODULE("ProceduralFairingDecoupler"):DOEVENT("jettison fairing").
            }
        }
    }

    // This puts us into an attitude where we're pointed in the same direction
    // as orbit prograde, but pointed at the horizon. This keeps us from burning
    // "up" and raising our apoapsis too quickly.

    IF STEERING = HEADING(launchAzimuth, controlPitch) AND SHIP:ORBIT:INCLINATION >= launchToInclination {
        LOCK STEERING TO LOOKDIRUP( VECTOREXCLUDE(SHIP:UP:FOREVECTOR, SHIP:PROGRADE:FOREVECTOR), SHIP:UP:FOREVECTOR ).
    }

    SET throttlePIDController:MAXOUTPUT TO MIN(1.0, MAX(0.0, (SHIP:MASS * (gForceLimit * CONSTANT:g0 - flightState_FD_long)) - flightState_ΣF_l)/flightState_ΣF_t).
    SET controlThrottle TO throttlePIDController:UPDATE(TIME:SECONDS, ETA:APOAPSIS).

    RETURN.
}

// Not currently used.

// DECLARE GLOBAL FUNCTION ModeRaiseApoapsisTransitionOutFunction {
//
//     SET StateFunction TO ModeS2ShutdownTransitionInFunction@.
//     RETURN.
// }

////////////////////////////////////////////////////////////////////////////////
// MODE S2 SHUTDOWN ////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

DECLARE GLOBAL FUNCTION ModeS2ShutdownTransitionInFunction {
    SET ModeName TO "S2 SHUTDOWN".
    MissionLog("MODE TO " + ModeName).

    StopTimeWarp().

    SET controlThrottle TO 0.0.
    LOCK THROTTLE TO controlThrottle.

    LOCK STEERING TO "KILL".

    SET register1 TO TIME.

    SET StateFunction TO ModeS2ShutdownLoopFunction@.
    RETURN.
}

DECLARE GLOBAL FUNCTION ModeS2ShutdownLoopFunction {

    IF THROTTLE = 0.0 AND TIME:SECONDS >= register1:SECONDS + 2.0 {

        IF SHIP:APOAPSIS < launchToApoapsis {
            SET StateFunction TO ModeEndProgramTransitionInFunction@.
            RETURN.
        }

        IF SHIP:APOAPSIS >= launchToApoapsis AND SHIP:ALTITUDE < SHIP:BODY:ATM:HEIGHT {
            SET StateFunction TO ModePoweredCoastTransitionInFunction@.
            RETURN.
        }

        IF SHIP:APOAPSIS >= launchToApoapsis AND SHIP:ALTITUDE >= SHIP:BODY:ATM:HEIGHT {
            SET StateFunction TO ModeComputeApoapsisManeuverTransitionInFunction@.
            RETURN.
        }

        SET StateFunction TO ModeEndProgramTransitionInFunction@.
    }

    RETURN.
}

// Not currently used.

// DECLARE GLOBAL FUNCTION ModeS2ShutdownTransitionOutFunction {
//
//     SET StateFunction TO {}.
//     RETURN.
// }

////////////////////////////////////////////////////////////////////////////////
// MODE POWERED COAST //////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// In this mode our apoapsis is correct but we're still inside the atmosphere,
// so we do a powered coast to space using RCS thrust to counteract atmospheric
// drag. Obviously if the stage has no forward RCS thrust then we'll fall back
// somewhat, but we should still end up close to our desired orbit.

DECLARE GLOBAL FUNCTION ModePoweredCoastTransitionInFunction {
    SET ModeName TO "POWERED COAST".
    MissionLog("MODE TO " + ModeName).

    SET controlPitch TO 0.0.
    LOCK STEERING TO SHIP:SRFPROGRADE.

    SET controlThrottle TO 0.0.
    LOCK THROTTLE TO controlThrottle.

    PANELS ON.

    RCS ON.
    SET rcsForePIDController:SETPOINT TO launchToApoapsis.
    SET SHIP:CONTROL:FORE TO rcsForePIDController:UPDATE(TIME:SECONDS, ALT:APOAPSIS).

    StartTimeWarp().

    SET StateFunction TO ModePoweredCoastLoopFunction@.
    RETURN.
}

DECLARE GLOBAL FUNCTION ModePoweredCoastLoopFunction {

    IF SHIP:ALTITUDE >= SHIP:BODY:ATM:HEIGHT {
        SET StateFunction TO ModePoweredCoastTransitionOutFunction@.
        RETURN.
    }

    // Trigger fairing jettison if we're past max. Q AND we're at the threshold
    // dynamic pressure.

    IF NOT payloadFairingsAll:EMPTY AND SHIP:Q < flightState_MaxQ AND SHIP:Q < fairingJettisonQ {

        MissionLog("FAIRING JETTISON").

        UNTIL payloadFairingsAll:EMPTY {
            DECLARE LOCAL f IS payloadFairingsAll:POP().
            IF f:HASMODULE("ModuleProceduralFairing") {
                f:GETMODULE("ModuleProceduralFairing"):DOEVENT("deploy").
            }
            IF f:HASMODULE("ProceduralFairingDecoupler") {
                f:GETMODULE("ProceduralFairingDecoupler"):DOEVENT("jettison fairing").
            }
        }
    }

    SET SHIP:CONTROL:FORE TO rcsForePIDController:UPDATE(TIME:SECONDS, ALT:APOAPSIS).

    RETURN.
}

DECLARE GLOBAL FUNCTION ModePoweredCoastTransitionOutFunction {

    StopTimeWarp().

    SET SHIP:CONTROL:FORE TO 0.0.
    SET SHIP:CONTROL:NEUTRALIZE TO TRUE.
    SET RCS TO initialRCSState.

    Set StateFunction TO ModeComputeApoapsisManeuverTransitionInFunction@.
    RETURN.
}

////////////////////////////////////////////////////////////////////////////////
// MODE COMPUTE APOAPSIS MANEUVER //////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Here we merely calculate the apoapsis maneuver necessary to put us into the
// target orbit. We create a maneuver node for it, but we don't try to execute
// it or anything. MechJeb does a fine job of executing maneuver nodes.

DECLARE GLOBAL FUNCTION ModeComputeApoapsisManeuverTransitionInFunction {
    SET ModeName TO "COMPUTE APOAPSIS MANEUVER".
    MissionLog("MODE TO " + ModeName).

    PANELS ON.

    DECLARE LOCAL v1 IS SQRT( SHIP:BODY:MU * ( (2/(SHIP:BODY:RADIUS + ALT:APOAPSIS)) - (1/SHIP:ORBIT:SEMIMAJORAXIS) ) ).
    DECLARE LOCAL v2 IS SQRT( SHIP:BODY:MU * ( (2/(SHIP:BODY:RADIUS + ALT:APOAPSIS)) - (1/launchToSemimajorAxis) ) ).
    DECLARE LOCAL Δv IS v2 - v1.

    DECLARE LOCAL apoapsisManeuver TO NODE( TIME:SECONDS + ETA:APOAPSIS, 0.0, 0.0, Δv).
    ADD apoapsisManeuver.

    SET StateFunction TO ModeNullRatesTransitionInFunction@.
    RETURN.
}

// Not currently used.

// DECLARE GLOBAL FUNCTION ModeComputeApoapsisManeuverTransitionOutFunction {
//
//     SET StateFunction TO {}.
//     RETURN.
// }

////////////////////////////////////////////////////////////////////////////////
// MODE NULL RATES /////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

DECLARE GLOBAL FUNCTION ModeNullRatesTransitionInFunction {
    SET ModeName TO "NULL RATES".
    MissionLog("MODE TO " + ModeName).

    SET SHIP:CONTROL:NEUTRALIZE TO TRUE.
    LOCK THROTTLE TO 0.0.
    LOCK STEERING TO "KILL".

    SET StateFunction TO ModeNullRatesLoopFunction@.
    RETURN.
}

DECLARE GLOBAL FUNCTION ModeNullRatesLoopFunction {

    // How about NOT spinning out of control?

    IF SHIP:ANGULARVEL:MAG < 0.010 {

        SET StateFunction TO ModeEndProgramTransitionInFunction@.
        RETURN.
    }
}

// Not currently used.

// DECLARE GLOBAL FUNCTION ModeNullRatesTransitionOutFunction {
//
//     SET StateFunction TO {}.
//     RETURN.
// }

////////////////////////////////////////////////////////////////////////////////
// MODE END PROGRAM ////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

DECLARE GLOBAL FUNCTION ModeEndProgramTransitionInFunction {
    SET ModeName TO "END PROGRAM".
    MissionLog("MODE TO " + ModeName).

    UNLOCK THROTTLE.
    UNLOCK STEERING.
    SET SHIP:CONTROL:NEUTRALIZE TO TRUE.
    SET RCS TO initialRCSState.

    MissionLog("CTRL-C TO END PROGRAM").

    IF pauseGameWhenFinished {
        KUNIVERSE:PAUSE().
    }

    SET StateFunction TO {}.
    RETURN.
}

// Not currently used.

// DECLARE GLOBAL FUNCTION ModeEndProgramLoopFunction {
//
//     SET StateFunction TO {}.
//     RETURN.
// }

// Not currently used.

// DECLARE GLOBAL FUNCTION ModeEndProgramTransitionOutFunction {
//
//     SET StateFunction TO {}.
//     RETURN.
// }

////////////////////////////////////////////////////////////////////////////////
//// TIMEWARP CONTROL                                                       ////
////////////////////////////////////////////////////////////////////////////////

DECLARE GLOBAL FUNCTION StartTimeWarp {
    IF useTimeWarp AND KUNIVERSE:TIMEWARP:RATE <> 2.0 {
        SET KUNIVERSE:TIMEWARP:MODE TO "PHYSICS".
        SET KUNIVERSE:TIMEWARP:RATE TO 2.0.
    }
}

DECLARE GLOBAL FUNCTION StopTimeWarp {
    IF useTimeWarp AND KUNIVERSE:TIMEWARP:RATE <> 1.0 {
        KUNIVERSE:TIMEWARP:CANCELWARP().
    }
}

////////////////////////////////////////////////////////////////////////////////
//// SCREEN FUNCTIONS                                                       ////
////////////////////////////////////////////////////////////////////////////////

// This function turns a number into a string representation with a fixed
// number of digits after the decimal point. For example, RoundZero(1.6, 2)
// returns "1.60" with that extra zero as padding.

DECLARE GLOBAL FUNCTION RoundZero {
    DECLARE PARAMETER n.
    DECLARE PARAMETER desiredPlaces IS 0.

    DECLARE LOCAL str IS ROUND(n, desiredPlaces):TOSTRING.

    IF desiredPlaces > 0 {
        DECLARE LOCAL hasPlaces IS 0.
        IF str:CONTAINS(".") {
            SET hasPlaces TO str:LENGTH - str:FIND(".") - 1.
        } ELSE {
            SET str TO str + ".".
        }
        IF hasPlaces < desiredPlaces {
            FROM { LOCAL i IS 0. } UNTIL i = desiredPlaces - hasPlaces STEP { SET i TO i + 1. } DO {
                SET str TO str + "0".
            }
        }
    }

    RETURN str.
}

DECLARE GLOBAL FUNCTION InitializeScreen {
    IF useDebugging {
        RETURN.
    }

    CLEARSCREEN.

//                   1         2         3         4         5
//         012345678901234567890123456789012345678901234567890
    PRINT "+-----------------------------------------------+ ". //  0
    PRINT "| T+00:00:00                                    | ". //  1
    PRINT "+-----------------------+-----------------------+ ". //  2
    PRINT "| THROTTLE     XXX %    | TGT APO   XXXX.XXX km | ". //  3
    PRINT "| RCS FWD      XXX %    | TGT PER   XXXX.XXX km | ". //  4
    PRINT "| α_long     XX.XX m/s² | AZIMUTH    XXX.XXX °  | ". //  5
    PRINT "| G           X.XX      +-----------------------+ ". //  6
    PRINT "| FD_long    XX.XX m/s² | Δv EXPENDED  XXXX m/s | ". //  7
    PRINT "| ΣF_t      XXXXXX kN   | Δv GAINED    XXXX m/s | ". //  8
    PRINT "| ΣF_l      XXXXXX kN   | EFFICIENCY    XXX %   | ". //  9
    PRINT "+-----------------------+-----------------------+ ". // 10
    PRINT "                                                  ". // 11

    RETURN.
}

DECLARE GLOBAL FUNCTION UpdateScreen {
    IF useDebugging {
        RETURN.
    }

    IF MISSIONTIME > 0 {
        PRINT TIME(MISSIONTIME):CLOCK                               AT ( 4,  1).
    }
    PRINT ModeName:PADLEFT(34)                                      AT (13,  1).

    PRINT RoundZero(THROTTLE * 100):PADLEFT(3)                      AT (15,  3).
    PRINT RoundZero(SHIP:CONTROL:FORE * 100):PADLEFT(3)             AT (15,  4).

    IF flightState_α_long < 99.99 {
        PRINT RoundZero(flightState_α_long, 2):PADLEFT(6)           AT (12,  5).
    } ELSE {
        PRINT "--.--":PADLEFT(6)                                    AT (12,  5).
    }
    IF flightState_α_long/CONSTANT:g0 < 9.99 {
        PRINT RoundZero(flightState_α_long/CONSTANT:g0, 2):PADLEFT(6) AT (12,  6).
    } ELSE {
        PRINT "-.--":PADLEFT(5)                                     AT (12,  6).
    }
    PRINT RoundZero(flightState_FD_long, 2):PADLEFT(6)              AT (12,  7).

    PRINT RoundZero(flightState_ΣF_t):PADLEFT(6)                    AT (12,  8).
    PRINT RoundZero(flightState_ΣF_l):PADLEFT(6)                    AT (12,  9).

    PRINT RoundZero(launchToApoapsis / 1000.0, 3):PADLEFT(8)        AT (36,  3).
    PRINT RoundZero(launchToPeriapsis / 1000.0, 3):PADLEFT(8)       AT (36,  4).
    PRINT RoundZero(launchAzimuth, 3):PADLEFT(7)                    AT (37,  5).

    PRINT RoundZero(flightState_ΔvExpended):PADLEFT(4)              AT (39,  7).
    PRINT RoundZero(flightState_ΔvGained):PADLEFT(4)                AT (39,  8).
    PRINT RoundZero(flightState_ΔvEfficiency * 100):PADLEFT(3)      AT (40,  9).

    RETURN.
}

DECLARE GLOBAL missionLogLineNumber IS 12.
DECLARE GLOBAL FUNCTION MissionLog {
    DECLARE PARAMETER line.

    IF useDebugging {
        PRINT "T+" + TIME(MISSIONTIME):CLOCK + " " + line.
    } ELSE {
        PRINT "T+" + TIME(MISSIONTIME):CLOCK + " " + line AT (0, missionLogLineNumber).
        SET missionLogLineNumber TO missionLogLineNumber + 1.
    }

    RETURN.
}

DECLARE GLOBAL FUNCTION MissionDebug {
    DECLARE PARAMETER line.

    IF useDebugging {
        MissionLog(line).
    }

    RETURN.
}

////////////////////////////////////////////////////////////////////////////////
//// MAIN LOOP                                                              ////
////////////////////////////////////////////////////////////////////////////////

SET StateFunction TO ModePrelaunchTransitionInFunction@.

InitializeScreen().
InitializeFlightState().

UNTIL FALSE {

    UpdateFlightState().

    StateFunction:CALL().

    UpdateScreen().

    // Yield until the next physics tick.
    WAIT 0.
}