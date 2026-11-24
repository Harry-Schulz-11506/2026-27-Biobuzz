package org.firstinspires.ftc.teamcode.NATS;

import com.acmerobotics.dashboard.config.Config;

/**
 * NATS robot: EVERY setting in the whole program lives in this one file.
 *
 * Nothing else defines a tunable number any more: Shooter, Turret, AutoActions, MainTeleOp,
 * TwoTipAuto, ShooterAutoTune and TurretAutoTune all read the fields below. Edit them here, in the
 * Dashboard under "RobotConfig", or with the tuner OpModes (which write back into this class).
 *
 * Values marked  TODO  are placeholders: set them to match your Robot Configuration (names) or measure
 * them on the robot with FTC Dashboard (servo positions). Everything that is a plain "public static"
 * (not final) is live-editable in the Dashboard.
 *
 * PATH POSES ARE THE ONE EXCEPTION: the 19 poses that build the auto's paths stay in TwoTipAuto.java,
 * because tools/autopath-preview/build.py reads them from there to draw the path visual. Everything that
 * is a setting (alliance, shot counts, timings) is here in section 9.
 */
@Config
public class RobotConfig {

    // =====================================================================
    // 1. HARDWARE CONFIG NAMES  (must match the Robot Configuration on the Control Hub exactly)
    // =====================================================================
    // Drive + odometry names live in pedro/Constants.java: fl, fr, bl, br and "pinpoint".
    public static final String DRIVE_FRONT_LEFT  = "fl";
    public static final String DRIVE_FRONT_RIGHT = "fr";
    public static final String DRIVE_BACK_LEFT   = "bl";
    public static final String DRIVE_BACK_RIGHT  = "br";
    public static final String PINPOINT          = "pinpoint";

    public static final String TURRET_MOTOR      = "turret";
    public static final String SHOOTER_MOTOR     = "shooter";

    // Intake + transfer are driven by 2 motors that always run together at the same power.
    public static final String INTAKE_MOTOR_1    = "intake";     // TODO name of the first intake/transfer motor
    public static final String INTAKE_MOTOR_2    = "intake2";   // TODO name of the second intake/transfer motor

    // The intake pivots on 2 servos that move together.
    public static final String PIVOT_SERVO_LEFT  = "pivotL";     // TODO
    public static final String PIVOT_SERVO_RIGHT = "pivotR";     // TODO

    public static final String FLOWER_SERVO      = "flower";     // TODO delete if the flower servo is gone
    public static final String STOP_SERVO        = "stop";       // TODO delete if the stop servo is gone

    public static final String HUSKYLENS         = "huskylens";  // TODO name of the HuskyLens (I2C) on the turret

    // =====================================================================
    // 2. SPEEDS / POWERS
    // =====================================================================
    // ---- Drive (TeleOp) ----
    // There is one drive mode: full stick. The strafe/spin sign lives in Turret.loop().

    // ---- Intake / transfer (2 motors) ----
    /** Both intake/transfer motors run at this power, always together (2 motors, one mechanism). */
    public static double INTAKE_POWER   = 1.0;     // TODO
    public static double OUTTAKE_POWER  = -1.0;    // unjam (negative = reversed)
    public static double SHOOT_STOP_DELAY_SEC = 4.0;
    /** Right bumper (TeleOp) and the auto's gate only feed once the flywheel is at speed; false = feed immediately. */
    public static boolean REQUIRE_AT_SPEED = true;
    /** AutoActions gate: feed a shot only while the flywheel is at speed. */
    public static boolean GATE_FEED_ON_SPEED = true;

    // ---- Turret ----
    public static double TURRET_MAX_POWER = 0.8;
    public static double TURRET_kP = 5.0;          // power per radian of error. Raised from 1.2 (too weak near the target); run TurretAutoTune for real values
    /** Smallest power the turret is given while it is still outside the tolerance, to beat friction (0 = off). */
    public static double TURRET_MIN_POWER = 0.05;
    public static double TURRET_kI = 0.0;          // left at 0 unless TurretAutoTune says otherwise
    public static double TURRET_kD = 0.0;
    public static double TURRET_I_LIMIT_POWER = 0.25; // the integral term may never ask for more than this
    /** +1 when positive power raises the encoder count; TurretAutoTune measures this and writes it back. */
    public static int    TURRET_ENCODER_SIGN = -1;
    public static double TURRET_TOLERANCE_DEG = 1.0;
    /** How close the auto's aim has to be before it counts as ready to shoot. */
    public static double TURRET_AIM_TOLERANCE_DEG = 2.0;
    // The turret turns 180 degrees each way from its zero (360 total), so 0 is the middle of travel.
    public static double TURRET_MIN_ANGLE_DEG = -180.0;
    public static double TURRET_MAX_ANGLE_DEG = 180.0;
    public static double TURRET_LIMIT_MARGIN_DEG = 5.0;
    public static double TRIM_RATE_DEG_PER_SEC = 20.0;

    // =====================================================================
    // 3. MOTOR DIRECTIONS  (flip these instead of rewiring or editing code)
    // =====================================================================
    // The drive motors are NOT here: their direction is set in pedro/Constants.java
    // (frontLeftDirection, frontRightDirection, backLeftDirection, backRightDirection).

    /** Flywheel motor: true = reverse its power direction. */
    public static boolean SHOOTER_MOTOR_REVERSED = false;
    /**
     * Flywheel encoder: true = the measured velocity is negated. Use this when the flywheel spins the
     * right way but the RPM comes out negative (or the velocity loop fights itself). It does NOT change
     * which way the motor spins — that is SHOOTER_MOTOR_REVERSED.
     */
    public static boolean SHOOTER_ENCODER_REVERSED = true;

    /** Turret motor power direction (the encoder side is TURRET_ENCODER_SIGN in section 2). */
    public static boolean TURRET_MOTOR_REVERSED = true;

    /** The 2 intake/transfer motors (they always run together at INTAKE_POWER). */
    public static boolean INTAKE_MOTOR_1_REVERSED = false;   // intake  = Direction.FORWARD
    public static boolean INTAKE_MOTOR_2_REVERSED = true;    // intake2 = Direction.REVERSE

    // =====================================================================
    // 4. SHOOTER  (flywheel velocity loop)
    // =====================================================================
    // ---- Speed ----
    public static double SHOOTER_MIN_RPM = 0;
    public static double SHOOTER_MAX_RPM = 10000;
    public static double RPM_STEP = 100;          // gamepad 1 D-pad
    public static double RPM_STEP_GAMEPAD2 = 50;  // gamepad 2 D-pad up / down
    /** Fixed speed used when the auto-RPM table is switched off, and the starting point for tuning. */
    public static double SHOOTER_TARGET_RPM = 2000;

    // ---- Velocity PIDF (used by Shooter, written by ShooterAutoTune) ----
    public static double SHOOTER_kP = 0.00279;   // power per RPM of error
    public static double SHOOTER_kI = 0.0;
    public static double SHOOTER_kD = 0.00024;
    public static double SHOOTER_kV = 0.000177;   // power per RPM  (~1 / 3333 at 12 V)
    public static double SHOOTER_kS = 0.06562;    // static offset power while target > 0
    public static double SHOOTER_I_LIMIT_POWER = 0.25;
    public static double SHOOTER_NOMINAL_VOLTAGE = 12.0;

    // ---- Shot-drop compensation ----
    // While a shot is being fed, the pollen drags the RPM down. The controller then pushes harder:
    // the P gain is multiplied and a fixed extra power is added. Set both to neutral (1 and 0) to switch off.
    public static double SHOOTER_FEED_KP_MULT = 1.0;     // kP is multiplied by this while feeding (1 = no change)
    public static double SHOOTER_FEED_BOOST_POWER = 0.0; // extra motor power added while feeding (0 = none)

    // ---- Dip recovery ----
    // Once the flywheel has reached its target, the first time the RPM falls DIP_THRESHOLD_RPM or more below
    // the target (a shot dragging it down), extra power is added until the RPM is back within
    // RECOVERY_END_RPM of the target. Set RECOVERY_BOOST_POWER to 0 to switch it off.
    public static double SHOOTER_DIP_THRESHOLD_RPM = 75.0;
    public static double SHOOTER_RECOVERY_BOOST_POWER = 0.35;
    public static double SHOOTER_RECOVERY_END_RPM = 20.0;

    // ---- Voltage compensation ----
    // Power needed for a given RPM rises as the battery sags. The feedforward (kS + kV*RPM) is scaled by
    // NOMINAL_VOLTAGE / battery voltage, so tune kS/kV on a fresh battery at NOMINAL_VOLTAGE.
    public static boolean SHOOTER_VOLTAGE_COMP_ENABLED = true;
    public static double SHOOTER_VOLTAGE_COMP_STRENGTH = 1.0; // 0 = none, 1 = full, >1 = overcompensate
    public static double SHOOTER_VOLTAGE_READ_SEC = 0.1;      // how often the battery is read (reads are slow)
    public static double SHOOTER_VOLTAGE_FILTER = 0.7;        // 0 = raw reading, closer to 1 = smoother
    public static double SHOOTER_MIN_VOLTAGE = 7.0;           // readings below this are treated as this

    // ---- Geometry (bare goBILDA 5000 series, 28 PPR, 10T -> 10T/18T flywheel stage) ----
    public static double SHOOTER_MOTOR_TICKS_PER_REV = 28;
    public static double SHOOTER_MOTOR_TEETH = 10;
    public static double SHOOTER_FLYWHEEL_TEETH = 10;

    // ---- Behaviour ----
    public static double SHOOTER_AT_SPEED_TOLERANCE_RPM = 25;
    public static double SHOOTER_AT_SPEED_HOLD_SEC = 0.25;
    public static double SHOOTER_VELOCITY_FILTER = 0.6;  // 0 = no filtering, closer to 1 = heavier filtering

    // ---- Distance -> RPM table (ShooterRegression) ----
    public static boolean AUTO_RPM_FROM_DISTANCE = true;
    // 1 = straight line through the measured shots (safe outside 31-67 in).
    // 2 = curve, fits the measured band tighter but turns back down past about 70 in.
    public static int REGRESSION_DEGREE = 1;

    // =====================================================================
    // 5. TURRET GEARING (435 rpm goBILDA Yellow Jacket -> 24T pulley -> 75T pulley)
    // =====================================================================
    public static final double TURRET_MOTOR_TICKS_PER_REV = 384.5;  // 435 rpm Yellow Jacket, 384.5 PPR at the output shaft
    public static final double TURRET_DRIVE_TEETH = 24.0;           // pulley on the motor
    public static final double TURRET_DRIVEN_TEETH = 75.0;          // pulley on the turret
    public static final double TURRET_RATIO = TURRET_DRIVEN_TEETH / TURRET_DRIVE_TEETH;          // 3.125 : 1
    public static final double TURRET_TICKS_PER_REV = TURRET_MOTOR_TICKS_PER_REV * TURRET_RATIO; // ~1198.75 ticks per turret turn

    // =====================================================================
    // 6. SERVO POSITIONS  (0.0 - 1.0)   TODO: measure every one of these on the robot
    // =====================================================================
    // Intake pivot (2 servos). If the right servo is mounted mirrored, set PIVOT_RIGHT_REVERSED = true.
    public static double PIVOT_UP    = 0.9;   // TODO stowed / up
    public static double PIVOT_DOWN  = 0.2;   // TODO down on the floor, intaking
    public static double PIVOT_MID   = 0.55;  // TODO optional middle position
    public static boolean PIVOT_RIGHT_REVERSED = true;

    public static double FLOWER_UP   = 0.9;   // carried over from AutoActions
    public static double FLOWER_DOWN = 0.2;
    public static double STOP_DOWN     = 0.5; // open: pollen pass
    public static double STOP_BLOCKING = 1.0; // closed: pollen held back
    public static double FLOWER_ADJUST_PER_SEC = 0.3; // controller 2 bumpers

    // =====================================================================
    // 7. FIELD CONSTANTS (Pedro field inches, 144 x 144)
    // =====================================================================
    public static final double FIELD_SIZE = 144.0;
    // Measured on the robot: 16 in along the heading, 14 in across. Every path must leave room for
    // this footprint, not just for the robot's centre point.
    public static double ROBOT_LENGTH = 16.0;     // along the robot's heading
    public static double ROBOT_WIDTH  = 14.0;
    // Mid-point of the robot to its wheels' outer edge is not needed: corner poses below use half of the sizes.

    // ---- Goal cells (the hive tips): cell 1 and cell 2 ----
    public static double RED_CELL1_X = 56.0;  public static double RED_CELL1_Y = 87.0;
    public static double RED_CELL2_X = 56.0;  public static double RED_CELL2_Y = 52.0;
    public static double BLUE_CELL1_X = 83.0; public static double BLUE_CELL1_Y = 52.0;
    public static double BLUE_CELL2_X = 83.0; public static double BLUE_CELL2_Y = 87.0;

    // ---- TeleOp start pose + goal tracking (Turret / MainTeleOp) ----
    /** true = TeleOp starts where the auto ended (PoseStorage); false = always use the start pose below. */
    public static boolean USE_AUTO_END_POSE = true;
    public static double TELEOP_START_X = 6.0;
    public static double TELEOP_START_Y = 6.0;
    public static double TELEOP_START_HEADING_DEG = 0.0;
    /** The turret pivot sits this far (inches) from the robot centre, in the robot's own frame. */
    public static double TURRET_OFFSET_X = -2.0;   // 2 in behind the centre
    public static double TURRET_OFFSET_Y = 0.0;

    // ---- Auto cell swap: cross the middle of the field and the tracked cell follows the robot's side ----
    public static boolean AUTO_CELL_SWAP = true;
    public static double CELL_SWAP_MIDLINE_Y = 72.0;    // half of 144
    public static double CELL_SWAP_HYSTERESIS_IN = 3.0; // inches past the line before it swaps (stops flicker on it)
    public static boolean CELL_SWAP_FAR_SIDE = false;   // true = pick the cell on the OPPOSITE side to the robot

    // =====================================================================
    // 8. CORNER RELOCALIZATION  (gamepad 2: hold START and press a button, see FieldCorners)
    // =====================================================================
    // Robot centre when the robot sits square in the corner. Heading in degrees.
    // TODO: set each heading to the way the robot really faces when you back it into that corner.
    public static double CORNER_BL_HEADING = 0;    // bottom-left  (x = 0,   y = 0,   audience side)
    public static double CORNER_BR_HEADING = 180;  // bottom-right (x = 144, y = 0)
    public static double CORNER_TL_HEADING = 0;    // top-left     (x = 0,   y = 144)
    public static double CORNER_TR_HEADING = 180;  // top-right    (x = 144, y = 144)

    // =====================================================================
    // 9. TWO TIP AUTO  (TwoTipAuto; the path poses themselves live in that file)
    // =====================================================================
    /** true = RED alliance, false = BLUE. The paths in TwoTipAuto are the RED ones. */
    public static boolean AUTO_RED_ALLIANCE = true;
    /** Which cell the PRELOAD is shot at. That is also the first hive, unless its tip has already fallen. */
    public static boolean AUTO_PRELOAD_CELL_IS_1 = true;
    /** Added to the flywheel RPM for every shot in the auto (negative = slower). Other OpModes are not affected. */
    public static double AUTO_RPM_OFFSET = 0;

    /** The bottom spot (54, 10) shoots the LOWER tip, which is CELL 2 in RobotConfig. */
    public static boolean AUTO_HIVE1_IS_CELL1 = false;
    /** The top spot (54, 130) shoots the UPPER tip, which is CELL 1 in RobotConfig. */
    public static boolean AUTO_HIVE2_IS_CELL1 = true;

    /** The one flower visit. Set false to skip it entirely. */
    public static boolean AUTO_FLOWER_TRIP = true;
    /** Shots the FIRST hive gets. Its tip normally falls on the first one, so 1 is enough. */
    public static int AUTO_FIRST_HIVE_MAX_SHOTS = 1;
    /** Shots the other hive gets. Its tip normally needs 2-3 before it falls. */
    public static int AUTO_OTHER_HIVE_MAX_SHOTS = 3;
    /** No new shot is started after this long from START. 0 = no limit. */
    public static long AUTO_BUDGET_MS = 27000;

    // ---- auto timings ----
    /** Feeding window for the preload shot. */
    public static long AUTO_PRELOAD_FEED_MS = 1000;
    /** Feeding window for each hive shot. */
    public static long AUTO_FEED_MS = 1000;
    /** How long to wait for "aimed + flywheel at speed" before feeding anyway. */
    public static long AUTO_READY_TIMEOUT_MS = 2500;
    /** How long to watch the preload hive's tip before choosing the first hive. */
    public static long AUTO_TIP_CHOICE_MS = 800;
    /** How long to watch a hive's tip after each shot, waiting for the tag to drop. */
    public static long AUTO_TIP_WATCH_MS = 2500;
    /** Extra intake time at the end of the ball line (the intake runs the whole way along it). */
    public static long AUTO_INTAKE_MS = 800;
    /** How long the intake runs while the robot is pushed into the flower. */
    public static long AUTO_FLOWER_INTAKE_MS = 900;
    /** How long to wait for the turret to come back to 0 deg at the end of the auto. */
    public static long AUTO_AIM_TIMEOUT_MS = 800;
    /** How long to wait for Pedro to report a newly started path as busy (so a path is never skipped). */
    public static long AUTO_PICKUP_TIMEOUT_MS = 500;

    // =====================================================================
    // 10. HUSKYLENS / APRILTAGS  (tip detection)
    // =====================================================================
    // Tag IDs by cell. "Audience side" = low Y (the audience FLOWER is at y = 0).
    //   RED  far side      30-33  -> RED  CELL 1 (y 87)
    //   RED  audience side 34-37  -> RED  CELL 2 (y 52)
    //   BLUE audience side 38-41  -> BLUE CELL 1 (y 52)
    //   BLUE far side      42-45  -> BLUE CELL 2 (y 87)
    public static final int[] TAGS_RED_CELL1  = {30, 31, 32, 33};
    public static final int[] TAGS_RED_CELL2  = {34, 35, 36, 37};
    public static final int[] TAGS_BLUE_CELL1 = {38, 39, 40, 41};
    public static final int[] TAGS_BLUE_CELL2 = {42, 43, 44, 45};

    // ---- Tip height: UP at or above 40 in, DOWN below 30 in, UNKNOWN in between ----
    public static double TIP_UP_MIN_IN    = 40.0;
    public static double TIP_DOWN_MAX_IN  = 30.0;
    /** true = a tip that is UP means the auto must do a second cycle; false = a tip that is DOWN does. */
    public static boolean SECOND_CYCLE_WHEN_UP = true;   // TODO confirm which one means "needs another cycle"
    /** What to do when the tip is not seen or sits between 30 and 40 in: true = do the second cycle anyway. */
    public static boolean SECOND_CYCLE_IF_UNKNOWN = false;

    // ---- Height estimate from the camera image (all TODO: measure on the robot) ----
    public static double TAG_SIZE_IN        = 2.0;   // TODO printed size of the AprilTag's black square, inches
    public static double CAM_HEIGHT_IN      = 12.0;  // TODO camera lens height above the floor, inches
    public static double CAM_PITCH_DEG      = 20.0;  // TODO camera tilt up from horizontal, degrees
    public static double CAM_HFOV_DEG       = 60.0;  // TODO HuskyLens horizontal field of view, degrees
    public static double CAM_IMAGE_W        = 320;   // HuskyLens image size
    public static double CAM_IMAGE_H        = 240;
    public static double TIP_HEIGHT_FILTER  = 0.7;   // 0 = raw, closer to 1 = smoother
    public static long   TIP_STALE_MS       = 750;   // a reading older than this is dropped
    /**
     * A tip counts as TIPPED once the tag has dropped this far (inches) below the highest height the
     * camera has seen since TipDetector.resetTip(). The hive tips fall, so a lower tag = a used tip.
     */
    public static double TIP_DROP_IN        = 6.0;

    // ---- Turret fine aim from the tag ----
    public static boolean VISION_AIM_ENABLED = false; // OFF until CAM_X_SIGN and the camera numbers are checked: a wrong sign fights the tracking
    public static double VISION_AIM_GAIN     = 0.7;  // 1 = trust the tag fully each loop
    public static double VISION_MAX_CORRECTION_DEG = 15.0; // never move more than this away from the pose-based aim
    public static int    CAM_X_SIGN          = -1;   // TODO flip to +1 if the turret turns the wrong way toward the tag

    // =====================================================================
    // 11. TUNER SETTINGS  (ShooterAutoTune and TurretAutoTune, FTC-AutoTune v1.4.3)
    // =====================================================================
    // Shooter tuning is in flywheel RPM and power-per-RPM; turret tuning is in degrees from where the
    // turret sits when START is pressed. Both write their results back into sections 2 and 4 above.

    // ---- ShooterAutoTune ----
    /** Speed the loop is tuned at, in flywheel RPM. Use a speed you actually shoot at. */
    public static double TUNE_SHOOTER_TARGET_RPM = 2200;
    /**
     * Relay swing on top of the feedforward power, 0-1. The relay is biased to the power that holds
     * TUNE_SHOOTER_TARGET_RPM, so this is the size of the oscillation, not the whole output.
     * Bigger = stronger oscillation and a shorter test, but more stress on the flywheel.
     */
    public static double TUNE_SHOOTER_RELAY_AMPLITUDE = 0.08;
    /** Relay deadband around the target, in RPM. Stops encoder noise from chattering the relay. */
    public static double TUNE_SHOOTER_HYSTERESIS_RPM = 40;
    /** Oscillation cycles averaged into Ku / Tu, and settling cycles thrown away first. */
    public static int TUNE_SHOOTER_CYCLES_TO_COLLECT = 5;
    public static int TUNE_SHOOTER_CYCLES_TO_IGNORE = 2;
    /** Give up on the relay test after this long (seconds). */
    public static double TUNE_SHOOTER_TIMEOUT_S = 60;
    /** Open-loop powers for the feedforward sweep. Keep the highest below full power. */
    public static double[] TUNE_SHOOTER_FF_POWERS = {0.4, 0.6, 0.8};
    public static double TUNE_SHOOTER_FF_SETTLE_S = 3.0;
    public static double TUNE_SHOOTER_FF_SAMPLE_S = 1.0;
    /** Safety cutoff: the tuner stops the flywheel above this speed. */
    public static double TUNE_SHOOTER_ABORT_RPM = 10000;
    /** false = PD-only candidates (kI stays 0), recommended for a flywheel. true = full PID family. */
    public static boolean TUNE_SHOOTER_TUNE_INTEGRAL = false;
    /** Candidate the live test starts on: 0 = most conservative ... 5 = most aggressive (4 = classic ZN). */
    public static int TUNE_SHOOTER_LIVE_CANDIDATE = 4;

    // ---- TurretAutoTune ----
    /**
     * Where the relay test oscillates, in degrees from where the turret sits when you press START.
     * Negative = toward a decreasing encoder count. The turret has 180 degrees each side, so -60
     * is comfortable when it starts near 0; pick the side with the most free travel.
     */
    public static double TUNE_TURRET_TARGET_OFFSET_DEG = -60;
    /** Power of the 0.25 s direction probe that learns which way the encoder counts. */
    public static double TUNE_TURRET_PROBE_POWER = 0.5;
    /** Relay output magnitude, clamped to TURRET_MAX_POWER. 0.2-0.4 suits a geared turret. */
    public static double TUNE_TURRET_RELAY_AMPLITUDE = 0.3;
    /** Relay deadband, in degrees. Raise it if the turret chatters instead of oscillating cleanly. */
    public static double TUNE_TURRET_HYSTERESIS_DEG = 1.0;
    /** Oscillation cycles averaged into Ku / Tu, and settling cycles thrown away first. */
    public static int TUNE_TURRET_CYCLES_TO_COLLECT = 5;
    public static int TUNE_TURRET_CYCLES_TO_IGNORE = 2;
    /** Give up on the relay test after this long (seconds). */
    public static double TUNE_TURRET_TIMEOUT_S = 30;
    /**
     * Hard safety net: the tuner stops the motor if the turret moves this far (degrees) from its start.
     * The turret has 180 degrees each side of its zero, so 120 leaves room for the oscillation even if
     * it is not perfectly centred when you start.
     */
    public static double TUNE_TURRET_MAX_EXCURSION_DEG = 120;
    /** false = PD-only candidates, true = full PID family (PI / PID). */
    public static boolean TUNE_TURRET_TUNE_INTEGRAL = false;
    /** Candidate the live test starts on: 2 = "no overshoot", the safest for a position mechanism. */
    public static int TUNE_TURRET_LIVE_CANDIDATE = 2;
}