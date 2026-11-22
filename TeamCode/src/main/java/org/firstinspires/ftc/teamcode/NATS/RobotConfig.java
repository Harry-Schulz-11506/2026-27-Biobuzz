package org.firstinspires.ftc.teamcode.NATS;

import com.acmerobotics.dashboard.config.Config;

/**
 * NATS robot: every hardware config name, speed, servo position and field constant in ONE place.
 *
 * Values marked  TODO  are placeholders: set them to match your Robot Configuration (names) or measure
 * them on the robot with FTC Dashboard (servo positions). Everything that is a plain "public static"
 * (not final) can be changed live in Dashboard under RobotConfig.
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

    // Intake + transfer are now driven by 2 motors in total.
    public static final String INTAKE_MOTOR_1    = "intake";     // TODO name of the first intake/transfer motor
    public static final String INTAKE_MOTOR_2    = "transfer";   // TODO name of the second intake/transfer motor

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
    public static double SLOW_DRIVE_SPEED = 0.7;
    public static double SLOW_TURN_SCALE  = 0.6;   // extra multiplier on turning in slow mode only
    public static double FAST_DRIVE_SPEED = 1.0;

    // ---- Intake / transfer (2 motors) ----
    public static double INTAKE_POWER   = 1.0;     // TODO
    public static double TRANSFER_POWER = 1.0;     // TODO (second motor)
    public static double OUTTAKE_POWER  = -1.0;    // unjam (negative = reversed)
    public static double SHOOT_STOP_DELAY_SEC = 4.0;

    // ---- Turret ----
    public static double TURRET_MAX_POWER = 0.8;
    public static double TURRET_kP = 1.2;          // TODO re-tune: the ratio changed, start lower if it oscillates
    public static double TURRET_TOLERANCE_DEG = 1.0;
    public static double TURRET_MIN_ANGLE_DEG = -315.0; // TODO re-check the limits after the rebuild
    public static double TURRET_MAX_ANGLE_DEG = 45.0;
    public static double TURRET_LIMIT_MARGIN_DEG = 5.0;
    public static double TRIM_RATE_DEG_PER_SEC = 20.0;

    // ---- Shooter ----
    public static double SHOOTER_MIN_RPM = 1000;
    public static double SHOOTER_MAX_RPM = 3300;
    public static double RPM_STEP = 100;
    // PIDF gains stay in Shooter.java (kP, kI, kV, kS) and are tuned with ShooterAutoTune.

    // =====================================================================
    // 3. TURRET GEARING (435 rpm goBILDA Yellow Jacket -> 24T pulley -> 75T pulley)
    // =====================================================================
    public static final double TURRET_MOTOR_TICKS_PER_REV = 383.6;  // 435 rpm Yellow Jacket, encoder at the output shaft (13.7:1 x 28)
    public static final double TURRET_DRIVE_TEETH = 24.0;           // pulley on the motor
    public static final double TURRET_DRIVEN_TEETH = 75.0;          // pulley on the turret
    public static final double TURRET_RATIO = TURRET_DRIVEN_TEETH / TURRET_DRIVE_TEETH;          // 3.125 : 1
    public static final double TURRET_TICKS_PER_REV = TURRET_MOTOR_TICKS_PER_REV * TURRET_RATIO; // ~1198.75 ticks per turret turn

    // =====================================================================
    // 4. SERVO POSITIONS  (0.0 - 1.0)   TODO: measure every one of these on the robot
    // =====================================================================
    // Intake pivot (2 servos). If the right servo is mounted mirrored, set PIVOT_RIGHT_REVERSED = true.
    public static double PIVOT_UP    = 0.9;   // TODO stowed / up
    public static double PIVOT_DOWN  = 0.2;   // TODO down on the floor, intaking
    public static double PIVOT_MID   = 0.55;  // TODO optional middle position
    public static boolean PIVOT_RIGHT_REVERSED = true;

    public static double FLOWER_UP   = 0.9;   // carried over from AutoActions
    public static double FLOWER_DOWN = 0.2;
    public static double STOP_DOWN     = 0.5; // open: artifacts pass
    public static double STOP_BLOCKING = 1.0; // closed: artifacts held back
    public static double FLOWER_ADJUST_PER_SEC = 0.3; // controller 2 bumpers

    // =====================================================================
    // 5. FIELD CONSTANTS (Pedro field inches, 144 x 144)
    // =====================================================================
    public static final double FIELD_SIZE = 144.0;
    public static double ROBOT_LENGTH = 14.0;     // along the robot's heading (from the Pedro Visualizer file)
    public static double ROBOT_WIDTH  = 12.0;
    // Mid-point of the robot to its wheels' outer edge is not needed: corner poses below use half of the sizes.

    // ---- Goal cells (the hive tips): cell 1 and cell 2 ----
    public static double RED_CELL1_X = 56.0;  public static double RED_CELL1_Y = 87.0;
    public static double RED_CELL2_X = 56.0;  public static double RED_CELL2_Y = 52.0;
    public static double BLUE_CELL1_X = 83.0; public static double BLUE_CELL1_Y = 52.0;
    public static double BLUE_CELL2_X = 83.0; public static double BLUE_CELL2_Y = 87.0;

    // =====================================================================
    // 6. CORNER RELOCALIZATION  (gamepad 2: hold START and press a button, see FieldCorners)
    // =====================================================================
    // Robot centre when the robot sits square in the corner. Heading in degrees.
    // TODO: set each heading to the way the robot really faces when you back it into that corner.
    public static double CORNER_BL_HEADING = 0;    // bottom-left  (x = 0,   y = 0,   audience side)
    public static double CORNER_BR_HEADING = 180;  // bottom-right (x = 144, y = 0)
    public static double CORNER_TL_HEADING = 0;    // top-left     (x = 0,   y = 144)
    public static double CORNER_TR_HEADING = 180;  // top-right    (x = 144, y = 144)

    // =====================================================================
    // 7. HUSKYLENS / APRILTAGS  (tip detection)
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

    // ---- Turret fine aim from the tag ----
    public static boolean VISION_AIM_ENABLED = true;
    public static double VISION_AIM_GAIN     = 0.7;  // 1 = trust the tag fully each loop
    public static double VISION_MAX_CORRECTION_DEG = 15.0; // never move more than this away from the pose-based aim
    public static int    CAM_X_SIGN          = -1;   // TODO flip to +1 if the turret turns the wrong way toward the tag
}
