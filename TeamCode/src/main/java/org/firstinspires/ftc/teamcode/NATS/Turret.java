package org.firstinspires.ftc.teamcode.NATS;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Turret: Subsystem & Standalone OpMode for turret position control and tracking.
 *
 * Tracking, in four steps every loop (see {@link #updateTurret(Pose, double, double, boolean)}):
 *   1. the goal angle in the robot's own frame,
 *   2. the AprilTag (HuskyLens) fine-aim nudge on top of it,
 *   3. ONE wrap, 180 deg each side of zero, then a clamp to the travel the turret really has,
 *   4. the motor's own (stock) position PID, RUN_TO_POSITION, given that target in encoder ticks.
 *
 * There is no hand-written PID and no "shortest way round" decision any more: the target is a single
 * tick position inside the travel, so the motor just drives to it.
 */
@Config
@TeleOp(name = "Turret Tracking Subsystem", group = "Subsystems")
public class Turret extends OpMode {

    // ============================
    // PEDRO PATHING
    // ============================
    public Follower follower;

    // ============================
    // GOAL SELECTION (alliance cell it tracks)
    // BACK toggles RED <-> BLUE, right stick button toggles CELL 1 <-> CELL 2.
    // ============================

    // ============================
    // MOTOR + EXTERNAL ENCODER
    // ============================
    public DcMotorEx turret;

    public long zeroTicks = 0;

    /** HuskyLens tip detector (null = no vision). Its tag nudges the aim when {@link RobotConfig#VISION_AIM_ENABLED}. */
    public TipDetector vision = null;

    // ---- Driver 2 aim trim (gamepad2 D-pad left/right; Y resets it) ----
    public double trimRad = 0.0;                       // added to the tracked target angle
    private long lastTrimNanos = 0;
    private double visionOffsetRad = 0.0;              // AprilTag fine aim: learned, bounded by VISION_MAX_CORRECTION_DEG
    private double lastVisionNanos = 0;                // seconds, for the tag correction's dt
    private boolean stockPidActive = false;            // true while the motor is in RUN_TO_POSITION
    private Pose startPose = new Pose(RobotConfig.TELEOP_START_X, RobotConfig.TELEOP_START_Y, Math.toRadians(RobotConfig.TELEOP_START_HEADING_DEG)); // where this TeleOp started (auto end pose or the default)

    public double targetAngle = 0.0;
    public double turretPower = 0.0;
    public boolean trackingEnabled = false;
    private boolean cornerMode = false;        // true while gamepad 2 START is held (corner relocalization buttons)
    public boolean isCornerMode() { return cornerMode; }
    private boolean holdZero = false;           // true after gamepad2 B: tracking is off and the turret is driven to 0 deg and held there
    public boolean previousA = false;
    private boolean previousX = false;

    /**
     * true  = Turret calls telemetry.update() itself (running as its own OpMode).
     * false = the owner (MainTeleOp) calls telemetry.update() once per loop, so the turret lines
     *         are shown together with the shooter lines instead of being overwritten.
     */
    public boolean updateTelemetry = true;

    private boolean redAlliance = true;   // starts on RED
    private boolean firstCell = true;     // starts on CELL 1
    private int lastSide = 0;                 // +1 = robot above RobotConfig.CELL_SWAP_MIDLINE_Y, -1 = below, 0 = not decided yet
    private boolean previousBack = false;
    private boolean previousR3 = false;

    public void setOpModeContext(com.qualcomm.robotcore.hardware.HardwareMap hw, Gamepad g1, Gamepad g2, Telemetry t) {
        this.hardwareMap = hw;
        this.gamepad1 = g1;
        this.gamepad2 = g2;
        this.telemetry = t;
    }

    @Override
    public void init() {
        if (hardwareMap != null) {
            follower = TurretInitializer.createFollower(hardwareMap);
            startPose = (RobotConfig.USE_AUTO_END_POSE && PoseStorage.lastPose != null)
                    ? new Pose(PoseStorage.lastPose.x(), PoseStorage.lastPose.y(), PoseStorage.lastPose.heading())
                    : new Pose(RobotConfig.TELEOP_START_X, RobotConfig.TELEOP_START_Y, Math.toRadians(RobotConfig.TELEOP_START_HEADING_DEG));
            follower.setPose(startPose);
            VoltageComp.init(hardwareMap); // battery reading for voltage compensation

            turret = TurretInitializer.initMotor(hardwareMap);
            zeroTicks = TurretInitializer.initZeroTicks(turret, RobotConfig.USE_AUTO_END_POSE);
            holdZero = false;
            resetVisionOffset();
            TurretInitializer.configureStockPid(turret);
            vision = TurretInitializer.initVision(hardwareMap);
        }

        if (telemetry != null) {
            telemetry.addLine("Turret initialized");
            telemetry.addLine("Tracking disabled");
            telemetry.addLine("Gamepad2 A / B: tracking on / off, X: relocalize");
            telemetry.addLine("Gamepad2 D-pad L/R: aim trim, D-pad up/down: RPM +/-, Y: reset both trims");
            if (updateTelemetry) telemetry.update();
        }
    }

    public double getTurretAngle() {
        if (turret == null) return 0.0;
        long ticks = (long) turret.getCurrentPosition() - zeroTicks;
        return ticks * RobotConfig.TURRET_ENCODER_SIGN * (2.0 * Math.PI / RobotConfig.TURRET_TICKS_PER_REV);
    }

    public double getAngle() {
        return Math.toDegrees(getTurretAngle());
    }

    /** The turret angle wrapped to 180 deg each side of zero, in degrees. */
    public double getWrappedAngle() {
        return Math.toDegrees(wrap180Rad(getTurretAngle()));
    }

    public double getTicksPerDegree() {
        return RobotConfig.TURRET_TICKS_PER_REV / 360.0;
    }

    public long getRawTicks() {
        if (turret == null) return 0;
        return (long) turret.getCurrentPosition() - zeroTicks;
    }

    /**
     * THE wrap: an angle in radians into the half-open range from -pi up to pi, i.e. 180 deg each
     * side of the turret's zero.
     * One full turn either way is the same angle, so there is exactly one value per physical angle.
     */
    public static double wrap180Rad(double rad) {
        double twoPi = 2.0 * Math.PI;
        double wrapped = (rad + Math.PI) % twoPi;
        if (wrapped < 0) wrapped += twoPi;
        return wrapped - Math.PI;
    }

    /** The same wrap in degrees: from -180 up to 180. */
    public static double wrap180(double deg) {
        return Math.toDegrees(wrap180Rad(Math.toRadians(deg)));
    }

    /** The turret range wrap: 180 deg each side of zero (TURRET_MIN/MAX_ANGLE_DEG are -180 / +180). */
    public static double wrapToTurretRange(double angle) {
        return wrap180Rad(angle);
    }

    /** Clamp an angle (radians) into the travel the turret really has: TURRET_MIN..TURRET_MAX_ANGLE_DEG. */
    public static double clampToTravel(double rad) {
        double min = Math.toRadians(RobotConfig.TURRET_MIN_ANGLE_DEG);
        double max = Math.toRadians(RobotConfig.TURRET_MAX_ANGLE_DEG);
        if (min > max) { double t = min; min = max; max = t; }
        return Range.clip(rad, min, max);
    }

    /** Raw encoder position (zeroTicks included) that puts the turret at angleRad. */
    public long ticksForAngle(double angleRad) {
        double ticksPerRad = RobotConfig.TURRET_TICKS_PER_REV / (2.0 * Math.PI);
        int sign = RobotConfig.TURRET_ENCODER_SIGN < 0 ? -1 : 1;
        return zeroTicks + Math.round(angleRad * ticksPerRad / sign);
    }

    // ============================
    // STOCK POSITION PID
    // ============================

    /**
     * Put the motor into its own position PID: RUN_TO_POSITION with the gains from RobotConfig
     * (TURRET_STOCK_kP / _kI / _kD / _kF). From then on the turret only hands it a target tick position.
     */
    public void configureStockPid() {
        if (turret == null) return;
        TurretInitializer.configureStockPid(turret);
        stockPidActive = true;
    }

    public void pushStockGains() {
        TurretInitializer.pushStockGains(turret);
    }

    /**
     * The velocity feedforward sent to the hub at the current battery voltage.
     *
     * TURRET_VELOCITY_kF is "full power (32767) at full speed" measured at VOLTAGE_NOMINAL. A sagging
     * battery gives less torque per unit of power, so the same velocity goal needs more of it: kF is
     * scaled by NOMINAL / battery voltage. That is the whole of the turret's voltage compensation - the
     * position P turns the position error into a velocity goal in ticks/s, which does not care about
     * the battery, and the motor's own velocity loop uses this kF to reach it.
     */
    public double velocityFeedforward() {
        return RobotConfig.TURRET_VELOCITY_kF * VoltageComp.scale();
    }

    // ============================
    // GOAL SELECTION
    // ============================

    /** Pick the goal directly: red = RED alliance, cell1 = CELL 1 (upper) instead of CELL 2. */
    public void selectGoal(boolean red, boolean cell1) {
        redAlliance = red;
        firstCell = cell1;
        resetVisionOffset();
    }

    public void setRedAlliance(boolean red) { redAlliance = red; resetVisionOffset(); }
    public void setCell1(boolean cell1) { firstCell = cell1; resetVisionOffset(); }

    /** Swap the tracked goal between the RED and the BLUE cell of the same number. */
    public void toggleRedBlue() { redAlliance = !redAlliance; resetVisionOffset(); }

    /** Swap the tracked goal between CELL 1 and CELL 2 of the same alliance. */
    public void toggleCell() { firstCell = !firstCell; resetVisionOffset(); }

    /** A different goal means different tags, so forget the offset learned from the old ones. */
    private void resetVisionOffset() { visionOffsetRad = 0.0; lastVisionNanos = 0; }

    public boolean isRedAlliance() { return redAlliance; }
    public boolean isCell1() { return firstCell; }

    public String getGoalName() {
        return (redAlliance ? "RED" : "BLUE") + " CELL " + (firstCell ? "1" : "2");
    }

    /** Field X of the goal currently selected (Pedro inches, Dashboard-editable above). */
    public double getGoalX() {
        if (redAlliance) return firstCell ? RobotConfig.RED_CELL1_X : RobotConfig.RED_CELL2_X;
        return firstCell ? RobotConfig.BLUE_CELL1_X : RobotConfig.BLUE_CELL2_X;
    }

    /** Field Y of the goal currently selected (Pedro inches, Dashboard-editable above). */
    public double getGoalY() {
        if (redAlliance) return firstCell ? RobotConfig.RED_CELL1_Y : RobotConfig.RED_CELL2_Y;
        return firstCell ? RobotConfig.BLUE_CELL1_Y : RobotConfig.BLUE_CELL2_Y;
    }

    public double calculateTarget(Pose pose) {
        return calculateTarget(pose, getGoalX(), getGoalY());
    }

    public double calculateTarget(Pose pose, double targetX, double targetY) {
        double heading = pose.heading();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double turretX = pose.x() + RobotConfig.TURRET_OFFSET_X * cos - RobotConfig.TURRET_OFFSET_Y * sin;
        double turretY = pose.y() + RobotConfig.TURRET_OFFSET_X * sin + RobotConfig.TURRET_OFFSET_Y * cos;

        double fieldAngle = Math.atan2(targetY - turretY, targetX - turretX);
        double relativeAngle = fieldAngle - heading;

        return wrapToTurretRange(relativeAngle);
    }

    /**
     * Call every loop. When the robot has crossed RobotConfig.CELL_SWAP_MIDLINE_Y (past the hysteresis band), selects the
     * alliance cell on the robot's side (or the far side if RobotConfig.CELL_SWAP_FAR_SIDE). Does nothing in between,
     * so a manual R3 swap sticks until the next crossing.
     */
    public void autoSelectCell(Pose pose) {
        if (!RobotConfig.AUTO_CELL_SWAP) { lastSide = 0; return; }
        int side = lastSide;
        if (pose.y() > RobotConfig.CELL_SWAP_MIDLINE_Y + RobotConfig.CELL_SWAP_HYSTERESIS_IN) side = 1;
        else if (pose.y() < RobotConfig.CELL_SWAP_MIDLINE_Y - RobotConfig.CELL_SWAP_HYSTERESIS_IN) side = -1;
        if (side == lastSide) return;
        lastSide = side;

        double y1 = redAlliance ? RobotConfig.RED_CELL1_Y : RobotConfig.BLUE_CELL1_Y;
        double y2 = redAlliance ? RobotConfig.RED_CELL2_Y : RobotConfig.BLUE_CELL2_Y;
        boolean wantUpper = (side > 0) != RobotConfig.CELL_SWAP_FAR_SIDE;
        firstCell = (y1 >= y2) == wantUpper; // cell 1 is the upper one when its Y is larger
    }

    /** Read the HuskyLens for the selected goal. Call every loop (TeleOp loop and AutoActions do). */
    public void updateVision() {
        if (vision != null) vision.update(redAlliance, firstCell);
    }

    public void updateTurret(Pose pose) {
        updateTurret(pose, getGoalX(), getGoalY(), trackingEnabled);
    }

    public void updateTurret(Pose pose, double targetX, double targetY, boolean tracking) {
        this.trackingEnabled = tracking;
        if (tracking) holdZero = false;

        // 1. Where the goal is, in the robot's own frame, plus the driver's trim.
        double aim = calculateTarget(pose, targetX, targetY) + trimRad;
        if (!trackingEnabled && holdZero) aim = 0.0; // B pressed: go back to 0 deg and hold it

        // 2. AprilTag adjustment: while a tag of the selected cell is in view, learn a small offset that
        //    walks the aim until the tag sits in the middle of the camera. It is rate limited and clamped,
        //    so a noisy or wrong tag cannot spin the turret: the pose maths still does the coarse aiming
        //    and the tag only trims what the pose maths cannot know (the turret's zero, wheel slip, etc.).
        updateVisionOffset(trackingEnabled);
        aim += visionOffsetRad;

        // 3. ONE wrap, 180 deg each side of zero, then a clamp to the travel the turret really has.
        //    What comes out is a single, reachable target: no direction to choose, no "shortest way
        //    round" that could point through the stop at 180 deg.
        targetAngle = clampToTravel(wrapToTurretRange(aim));

        if (turret == null) {
            turretPower = 0.0;
            return;
        }

        if (!trackingEnabled && !holdZero) {
            // Tracking off: stop where it is, so switching it back on does not jump.
            turretPower = 0.0;
            turret.setTargetPosition(turret.getCurrentPosition());
            turret.setPower(0);
            return;
        }

        // 4. The motor's own (stock) position PID does the work: hand it the target in encoder ticks.
        if (!stockPidActive) configureStockPid();
        else pushStockGains();
        turret.setTargetPosition((int) TurretInitializer.ticksForAngle(turret, zeroTicks, targetAngle));
        turret.setPower(RobotConfig.TURRET_MAX_POWER);
        turretPower = turret.getPower();
    }

    /**
     * Learn the AprilTag fine-aim offset. The HuskyLens says how far the tag is left/right of the camera
     * centre; while tracking, that error walks {@link #visionOffsetRad} toward centring the tag, at most
     * VISION_AIM_RATE_DEG_PER_SEC and never more than VISION_MAX_CORRECTION_DEG. VISION_AIM_DEADBAND_DEG
     * stops a centred tag from chattering. The offset resets whenever the tracked goal changes.
     */
    private void updateVisionOffset(boolean tracking) {
        double now = System.nanoTime() / 1e9;
        double dt = (lastVisionNanos == 0) ? 0 : Math.min(now - lastVisionNanos, 0.1);
        lastVisionNanos = now;
        if (!tracking || !RobotConfig.VISION_AIM_ENABLED || vision == null || !vision.hasTag() || dt <= 0) return;

        double error = vision.getAimErrorRad(); // + = tag is left of centre: turn + (CCW) to centre it
        if (Math.abs(error) < Math.toRadians(RobotConfig.VISION_AIM_DEADBAND_DEG)) return;

        double maxStep = Math.toRadians(RobotConfig.VISION_AIM_RATE_DEG_PER_SEC) * dt;
        double limit = Math.toRadians(RobotConfig.VISION_MAX_CORRECTION_DEG);
        visionOffsetRad = Range.clip(visionOffsetRad + Range.clip(error, -maxStep, maxStep), -limit, limit);
    }

    @Override
    public void loop() {
        if (follower != null && gamepad1 != null) {
            VoltageComp.update(); // one battery read per loop, shared with the turret's kF

            // One drive mode only: full stick, strafe and spin in the directions the robot uses now.
            // If they ever feel mirrored, flip FLIP_STRAFE_AND_SPIN to +1.0 (the only place that matters).
            // RobotConfig.DRIVE_POWER_SCALE is 1.0 = full power; Pedro then normalises the four wheel
            // powers to at most 1.0, so at a full stick every motor is commanded 1.0.
            // Voltage compensation scales the sticks by NOMINAL / battery voltage, so half a stick turns
            // the wheels the same amount on a flat battery as on a fresh one (at full stick there is
            // nothing left to give, so 1.0 stays 1.0).
            final double FLIP_STRAFE_AND_SPIN = -1.0;
            double driveScale = Range.clip(RobotConfig.DRIVE_POWER_SCALE * VoltageComp.scale(), 0.0, 1.0);
            follower.manual(
                    driveScale * -gamepad1.left_stick_y,
                    driveScale * FLIP_STRAFE_AND_SPIN * gamepad1.left_stick_x,
                    driveScale * FLIP_STRAFE_AND_SPIN * gamepad1.right_stick_x
            );
            follower.update();
            Pose pose = follower.pose();

            // Hold gamepad 2 START + X/A/Y/B: relocalize to a field corner (see FieldCorners). While START
            // is held the buttons below are NOT used for their normal jobs.
            cornerMode = FieldCorners.update(gamepad2, follower);
            if (gamepad2 != null && cornerMode) {
                previousA = gamepad2.a;
                previousX = gamepad2.x;
                pose = follower.pose();
            }

            if (gamepad2 != null && !cornerMode) {
                long nowN = System.nanoTime();
                double dtTrim = lastTrimNanos == 0 ? 0 : Math.min((nowN - lastTrimNanos) / 1e9, 0.1);
                lastTrimNanos = nowN;
                double dir = (gamepad2.dpad_right ? -1 : 0) + (gamepad2.dpad_left ? 1 : 0); // left = CCW (+)
                trimRad += Math.toRadians(RobotConfig.TRIM_RATE_DEG_PER_SEC) * dir * dtTrim;
                if (gamepad2.y) trimRad = 0; // Y: reset the aim trim (MainTeleOp resets the RPM trim on the same press)

                boolean a = gamepad2.a;
                if (a && !previousA) {
                    trackingEnabled = true; // A: turret tracking on
                }
                previousA = a;
                boolean x = gamepad2.x;
                if (x && !previousX && follower != null) {
                    follower.setPose(startPose); // X: relocalize to the start pose
                }
                previousX = x;
                if (gamepad2.b) {
                    trackingEnabled = false;
                    holdZero = true; // B: stop tracking and set the turret to 0 deg
                }
            }

            // BACK: swap RED <-> BLUE. Right stick button: swap CELL 1 <-> CELL 2.
            if (gamepad1.back && !previousBack) toggleRedBlue();
            if (gamepad1.right_stick_button && !previousR3) toggleCell();
            previousBack = gamepad1.back;
            previousR3 = gamepad1.right_stick_button;

            autoSelectCell(pose);
            updateVision();
            updateTurret(pose);

            if (telemetry != null) {
                double currentAngle = getTurretAngle();
                double error = targetAngle - currentAngle;
                long rawTicks = getRawTicks();

                telemetry.addData("Robot X", "%.2f", pose.x());
                telemetry.addData("Robot Y", "%.2f", pose.y());
                telemetry.addData("Robot Heading (deg)", "%.2f", Math.toDegrees(pose.heading()));
                telemetry.addData("Goal", "%s at %.1f, %.1f", getGoalName(), getGoalX(), getGoalY());
                telemetry.addLine();
                telemetry.addData("Turret Angle (deg)", "%.2f", Math.toDegrees(currentAngle));
                telemetry.addData("Target Angle (deg)", "%.2f", Math.toDegrees(targetAngle));
                telemetry.addData("Error (deg)", "%.2f", Math.toDegrees(error));
                telemetry.addData("Encoder Ticks", rawTicks);
                telemetry.addData("Turret Power", "%.3f", turretPower);
                telemetry.addData("Battery / Volts comp", "%.2f V / x%.3f", VoltageComp.voltage(), VoltageComp.scale());
                telemetry.addData("Velocity kF (compensated / nominal)", "%.2f / %.2f", velocityFeedforward(), RobotConfig.TURRET_VELOCITY_kF);
                telemetry.addData("Stock PID (power/tick)", "kP %.5f  kI %.5f  kD %.5f",
                        RobotConfig.TURRET_STOCK_kP, RobotConfig.TURRET_STOCK_kI, RobotConfig.TURRET_STOCK_kD);
                telemetry.addData("At Target", "%s (%.1f deg tolerance)",
                        Math.abs(Math.toDegrees(error)) <= RobotConfig.TURRET_TOLERANCE_DEG ? "YES" : "no",
                        RobotConfig.TURRET_TOLERANCE_DEG);
                telemetry.addData("Tracking", trackingEnabled ? "ON" : "OFF");
                if (vision != null) vision.addTelemetry(telemetry);
                telemetry.addData("Aim Trim (deg)", "%.1f", Math.toDegrees(trimRad));
                telemetry.addData("Tag Aim Offset (deg)", "%.1f / limit %.0f", Math.toDegrees(visionOffsetRad),
                        RobotConfig.VISION_MAX_CORRECTION_DEG);
                if (updateTelemetry) telemetry.update();
            }
        }
    }

    public void zeroHere() {
        zeroTicks = turret != null ? turret.getCurrentPosition() : 0;
        targetAngle = 0.0;
        if (turret != null) {
            // Hold where it is now: the new zero is the current tick count, so it must not move.
            turret.setTargetPosition((int) zeroTicks);
            turret.setPower(0);
        }
        PoseStorage.turretZeroTicks = (int) zeroTicks;
    }

    public void setTarget(double deg) {
        targetAngle = wrapToTurretRange(Math.toRadians(deg));
    }

    public void nudgeTarget(double deltaDeg) {
        targetAngle = wrapToTurretRange(targetAngle + Math.toRadians(deltaDeg));
    }

    public void update() {
        if (follower != null) {
            updateTurret(follower.pose());
        }
    }

    public double getTarget() { return Math.toDegrees(targetAngle); }
    /** Aim error in degrees, wrapped to 180 each side (a 2 deg miss at the seam reads 2, not 358). */
    public double getError() { return Math.toDegrees(wrap180Rad(targetAngle - getTurretAngle())); }
    public double getPower() { return turretPower; }

    public void setOpenLoopPower(double power) {
        if (turret == null) {
            turretPower = 0.0;
            return;
        }
        double currentAngle = getTurretAngle();
        if (power > 0 && currentAngle >= Math.toRadians(RobotConfig.TURRET_MAX_ANGLE_DEG)) power = 0;
        if (power < 0 && currentAngle <= Math.toRadians(RobotConfig.TURRET_MIN_ANGLE_DEG)) power = 0;
        if (power != 0 && stockPidActive) {
            // Leaving RUN_TO_POSITION: back to plain power. Tracking re-enables the stock PID.
            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            stockPidActive = false;
        }
        turretPower = VoltageComp.scaledPower(power); // open loop too: same speed at any battery
        turret.setPower(turretPower);
    }

    @Override
    public void stop() {
        trackingEnabled = false;
        if (turret != null) turret.setPower(0);
    }
}
