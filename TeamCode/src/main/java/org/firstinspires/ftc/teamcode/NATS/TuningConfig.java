package org.firstinspires.ftc.teamcode.NATS;

import com.acmerobotics.dashboard.config.Config;

/**
 * Settings for the FTC-AutoTune tuner OpModes: {@link ShooterAutoTune} and {@link TurretAutoTune}.
 *
 * Every field is a plain static (not final), so FTC Dashboard shows them under "TuningConfig" and
 * they can be edited live without redeploying.
 *
 * The tuners use FTC-AutoTune (https://ftcautotune.dev, library v1.4.3):
 *  - RelayAutoTuner drives a relay (bang-bang) oscillation and measures Ku / Tu (Astrom-Hagglund).
 *  - ZieglerNicholsCalculator turns Ku / Tu into six candidate gain sets, conservative to aggressive.
 *  - ShooterAutoTune also characterizes the feedforward with an open-loop sweep, fitted as
 *    power = kS + kV * RPM; kV becomes the candidate kF, because the library's own fit goes through
 *    the origin and cannot see kS.
 *
 * The tuners measure in the units of the production controllers, so a candidate can be pasted
 * straight in:
 *   shooter: flywheel RPM, gains in power per RPM  -> Shooter.kP / kI / kD / kV / kS
 *   turret:  encoder ticks, gains in power per tick -> converted to Turret's radians on apply
 */
@Config
public class TuningConfig {

    // =====================================================================
    // SHOOTER (flywheel velocity) - ShooterAutoTune
    // =====================================================================

    /** Speed the loop is tuned at, in flywheel RPM. Use a speed you actually shoot at. */
    public static double SHOOTER_TARGET_RPM = 2200;

    /**
     * Relay swing on top of the feedforward power, 0-1. The relay is biased to the power that holds
     * SHOOTER_TARGET_RPM, so this is the size of the oscillation, not the whole output.
     * Bigger = stronger oscillation and a shorter test, but more stress on the flywheel.
     */
    public static double SHOOTER_RELAY_AMPLITUDE = 0.08;

    /** Relay deadband around the target, in RPM. Stops encoder noise from chattering the relay. */
    public static double SHOOTER_HYSTERESIS_RPM = 40;

    /** Oscillation cycles averaged into Ku / Tu, and settling cycles thrown away first. */
    public static int SHOOTER_CYCLES_TO_COLLECT = 5;
    public static int SHOOTER_CYCLES_TO_IGNORE = 2;

    /** Give up on the relay test after this long (seconds). */
    public static double SHOOTER_TIMEOUT_S = 60;

    /** Open-loop powers for the feedforward sweep. Keep the highest below full power. */
    public static double[] SHOOTER_FF_POWERS = {0.4, 0.6, 0.8};
    public static double SHOOTER_FF_SETTLE_S = 3.0;
    public static double SHOOTER_FF_SAMPLE_S = 1.0;

    /** Safety cutoff: the tuner stops the flywheel above this speed. */
    public static double SHOOTER_ABORT_RPM = 3400;

    /** false = PD-only candidates (kI stays 0), recommended for a flywheel. true = full PID family. */
    public static boolean SHOOTER_TUNE_INTEGRAL = false;

    /** Candidate the live test starts on: 0 = most conservative ... 5 = most aggressive (4 = classic ZN). */
    public static int SHOOTER_LIVE_CANDIDATE = 4;

    // =====================================================================
    // TURRET (position) - TurretAutoTune
    // =====================================================================

    /**
     * Where the relay test oscillates, in degrees from where the turret sits when you press START.
     * Negative = toward a decreasing encoder count. The turret has 180 degrees each side, so -60
     * is comfortable when it starts near 0; pick the side with the most free travel.
     */
    public static double TURRET_TARGET_OFFSET_DEG = -60;

    /** Relay output magnitude, clamped to RobotConfig.TURRET_MAX_POWER. 0.2-0.4 suits a geared turret. */
    public static double TURRET_RELAY_AMPLITUDE = 0.3;

    /** Relay deadband, in degrees. Raise it if the turret chatters instead of oscillating cleanly. */
    public static double TURRET_HYSTERESIS_DEG = 1.0;

    /** Oscillation cycles averaged into Ku / Tu, and settling cycles thrown away first. */
    public static int TURRET_CYCLES_TO_COLLECT = 5;
    public static int TURRET_CYCLES_TO_IGNORE = 2;

    /** Give up on the relay test after this long (seconds). */
    public static double TURRET_TIMEOUT_S = 30;

    /**
     * Hard safety net: the tuner stops the motor if the turret moves this far (degrees) from its start.
     * The turret has 180 degrees each side of its zero, so 120 leaves room for the oscillation even if
     * it is not perfectly centred when you start.
     */
    public static double TURRET_MAX_EXCURSION_DEG = 120;

    /** false = PD-only candidates, true = full PID family (PI / PID). */
    public static boolean TURRET_TUNE_INTEGRAL = false;

    /** Candidate the live test starts on: 2 = "no overshoot", the safest for a position mechanism. */
    public static int TURRET_LIVE_CANDIDATE = 2;
}
