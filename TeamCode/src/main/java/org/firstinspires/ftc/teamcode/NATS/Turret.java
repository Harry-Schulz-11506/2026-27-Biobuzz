package org.firstinspires.ftc.teamcode.NATS;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Turret: Subsystem & Standalone OpMode for turret position control and tracking.
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

    // Smallest thing left here: every value below (start pose, cell points, cell-swap line, turret
    // offset, limits, gains) lives in RobotConfig.

    // ============================
    // MOTOR + EXTERNAL ENCODER
    // ============================
    public DcMotorEx turret;

    public long zeroTicks = 0;

    /** HuskyLens tip detector (null = no vision). Its tag nudges the aim when VISION_AIM_ENABLED. */
    public TipDetector vision = null;

    // ---- Driver 2 aim trim (gamepad2 D-pad left/right; Y resets it) ----
    public double trimRad = 0.0;                       // added to the tracked target angle
    private long lastTrimNanos = 0;
    private double integral = 0.0;              // RobotConfig.TURRET_kI accumulator, in rad*s (stays 0 unless RobotConfig.TURRET_kI != 0)
    private double lastAngleForD = Double.NaN;  // previous angle, for the RobotConfig.TURRET_kD term
    private double lastPidTime = -1;            // seconds, for dt
    private Pose startPose = new Pose(RobotConfig.TELEOP_START_X, RobotConfig.TELEOP_START_Y, RobotConfig.TELEOP_START_HEADING_DEG); // where this TeleOp started (auto end pose or the default)

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

    public void setOpModeContext(HardwareMap hw, Gamepad g1, Gamepad g2, Telemetry t) {
        this.hardwareMap = hw;
        this.gamepad1 = g1;
        this.gamepad2 = g2;
        this.telemetry = t;
    }

    @Override
    public void init() {
        if (hardwareMap != null) {
            follower = Constants.create(hardwareMap);
            startPose = (RobotConfig.USE_AUTO_END_POSE && PoseStorage.lastPose != null)
                    ? new Pose(PoseStorage.lastPose.x(), PoseStorage.lastPose.y(), PoseStorage.lastPose.heading())
                    : new Pose(RobotConfig.TELEOP_START_X, RobotConfig.TELEOP_START_Y, RobotConfig.TELEOP_START_HEADING_DEG);
            follower.setPose(startPose);

            turret = hardwareMap.get(DcMotorEx.class, RobotConfig.TURRET_MOTOR);
            turret.setDirection(RobotConfig.TURRET_MOTOR_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turret.setPower(0);

            // The encoder is not reset between auto and TeleOp, so reuse the auto's zero: the turret then
            // starts TeleOp at the angle the auto left it. With no stored zero it is 0 deg right here.
            if (RobotConfig.USE_AUTO_END_POSE && PoseStorage.turretZeroTicks != null) {
                zeroTicks = PoseStorage.turretZeroTicks;
            } else {
                zeroTicks = turret.getCurrentPosition();
            }
            PoseStorage.turretZeroTicks = (int) zeroTicks;
            holdZero = false;

            vision = new TipDetector();
            vision.init(hardwareMap);
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

    public double getWrappedAngle() {
        return Math.toDegrees(getTurretAngle());
    }

    public double getTicksPerDegree() {
        return RobotConfig.TURRET_TICKS_PER_REV / 360.0;
    }

    public long getRawTicks() {
        if (turret == null) return 0;
        return (long) turret.getCurrentPosition() - zeroTicks;
    }

    public static double wrapToTurretRange(double angle) {
        double twoPi = 2.0 * Math.PI;
        return ((angle - Math.toRadians(RobotConfig.TURRET_MIN_ANGLE_DEG)) % twoPi + twoPi) % twoPi + Math.toRadians(RobotConfig.TURRET_MIN_ANGLE_DEG);
    }

    public static double wrap180(double deg) {
        double rad = Math.toRadians(deg);
        return Math.toDegrees(wrapToTurretRange(rad));
    }

    // ============================
    // GOAL SELECTION
    // ============================

    /** Pick the goal directly: red = RED alliance, cell1 = CELL 1 (upper) instead of CELL 2. */
    public void selectGoal(boolean red, boolean cell1) {
        redAlliance = red;
        firstCell = cell1;
    }

    public void setRedAlliance(boolean red) { redAlliance = red; }
    public void setCell1(boolean cell1) { firstCell = cell1; }

    /** Swap the tracked goal between the RED and the BLUE cell of the same number. */
    public void toggleRedBlue() { redAlliance = !redAlliance; }

    /** Swap the tracked goal between CELL 1 and CELL 2 of the same alliance. */
    public void toggleCell() { firstCell = !firstCell; }

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
        targetAngle = wrapToTurretRange(calculateTarget(pose, targetX, targetY) + trimRad);
        if (!trackingEnabled && holdZero) targetAngle = 0.0; // B pressed: go to 0 deg and stay there

        // HuskyLens fine aim: when a tag of the selected cell is in view, steer to centre it, but never
        // more than VISION_MAX_CORRECTION_DEG away from the pose-based aim (a wrong tag cannot spin the turret).
        if (trackingEnabled && RobotConfig.VISION_AIM_ENABLED && vision != null && vision.hasTag()) {
            double visionTarget = getTurretAngle() + RobotConfig.VISION_AIM_GAIN * vision.getAimErrorRad();
            double diff = visionTarget - targetAngle;
            diff = Math.atan2(Math.sin(diff), Math.cos(diff));
            double maxDiff = Math.toRadians(RobotConfig.VISION_MAX_CORRECTION_DEG);
            targetAngle = wrapToTurretRange(targetAngle + Range.clip(diff, -maxDiff, maxDiff));
        }
        double currentAngle = getTurretAngle();
        // Direct difference, NOT the shortest way round: the travel is exactly -180..+180, so the turret may never
        // cross the +-180 seam. targetAngle is already inside the range, so this always turns the long way
        // when that is the only way, and never drives into a limit.
        double error = targetAngle - currentAngle;

        if ((!trackingEnabled && !holdZero) || turret == null) {
            turretPower = 0.0;
            integral = 0.0;
            lastAngleForD = Double.NaN;
            lastPidTime = -1;
            if (turret != null) turret.setPower(0);
            return;
        }

        double now = System.nanoTime() / 1e9;
        double dt = (lastPidTime < 0) ? 0 : Math.min(now - lastPidTime, 0.1);
        lastPidTime = now;

        // RobotConfig.TURRET_kI and RobotConfig.TURRET_kD are 0 by default, so this is still a plain P controller until TurretAutoTune
        // writes gains into RobotConfig.TURRET_kP / RobotConfig.TURRET_kI / RobotConfig.TURRET_kD.
        if (RobotConfig.TURRET_kI != 0) {
            integral += error * dt;
            double limit = RobotConfig.TURRET_I_LIMIT_POWER / Math.abs(RobotConfig.TURRET_kI);
            integral = Range.clip(integral, -limit, limit);
        } else {
            integral = 0;
        }
        double derivative = 0;
        if (RobotConfig.TURRET_kD != 0 && dt > 1e-4 && !Double.isNaN(lastAngleForD)) {
            derivative = (currentAngle - lastAngleForD) / dt;
        }
        lastAngleForD = currentAngle;

        double power = Range.clip(RobotConfig.TURRET_kP * error + RobotConfig.TURRET_kI * integral - RobotConfig.TURRET_kD * derivative, -RobotConfig.TURRET_MAX_POWER, RobotConfig.TURRET_MAX_POWER);

        if (Math.abs(error) < Math.toRadians(RobotConfig.TURRET_TOLERANCE_DEG)) {
            power = 0.0;
        } else if (Math.abs(power) < RobotConfig.TURRET_MIN_POWER) {
            power = Math.signum(power) * RobotConfig.TURRET_MIN_POWER; // small errors still get enough to move
        }

        if (power > 0) {
            double remaining = Math.toRadians(RobotConfig.TURRET_MAX_ANGLE_DEG) - currentAngle;
            if (remaining <= 0) {
                power = 0;
            } else if (remaining < Math.toRadians(RobotConfig.TURRET_LIMIT_MARGIN_DEG)) {
                power *= remaining / Math.toRadians(RobotConfig.TURRET_LIMIT_MARGIN_DEG);
            }
        } else if (power < 0) {
            double remaining = currentAngle - Math.toRadians(RobotConfig.TURRET_MIN_ANGLE_DEG);
            if (remaining <= 0) {
                power = 0;
            } else if (remaining < Math.toRadians(RobotConfig.TURRET_LIMIT_MARGIN_DEG)) {
                power *= remaining / Math.toRadians(RobotConfig.TURRET_LIMIT_MARGIN_DEG);
            }
        }

        // Already past a limit (overshoot, or the zero was off): drive back inside instead of just cutting power.
        double minA = Math.toRadians(RobotConfig.TURRET_MIN_ANGLE_DEG), maxA = Math.toRadians(RobotConfig.TURRET_MAX_ANGLE_DEG);
        if (currentAngle > maxA) power = -Math.min(RobotConfig.TURRET_MAX_POWER, Math.max(0.15, RobotConfig.TURRET_kP * (currentAngle - maxA)));
        else if (currentAngle < minA) power = Math.min(RobotConfig.TURRET_MAX_POWER, Math.max(0.15, RobotConfig.TURRET_kP * (minA - currentAngle)));

        turretPower = power;
        turret.setPower(power);
    }

    @Override
    public void loop() {
        if (follower != null && gamepad1 != null) {
            // One drive mode only: full stick, strafe and spin in the directions the robot uses now.
            // If they ever feel mirrored, flip FLIP_STRAFE_AND_SPIN to +1.0 (the only place that matters).
            final double FLIP_STRAFE_AND_SPIN = -1.0;
            follower.manual(
                    -gamepad1.left_stick_y,
                    FLIP_STRAFE_AND_SPIN * gamepad1.left_stick_x,
                    FLIP_STRAFE_AND_SPIN * gamepad1.right_stick_x
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
                telemetry.addData("Tracking", trackingEnabled ? "ON" : "OFF");
                if (vision != null) vision.addTelemetry(telemetry);
                telemetry.addData("Aim Trim (deg)", "%.1f", Math.toDegrees(trimRad));
                if (updateTelemetry) telemetry.update();
            }
        }
    }

    public void zeroHere() {
        if (turret != null) {
            zeroTicks = turret.getCurrentPosition();
            PoseStorage.turretZeroTicks = (int) zeroTicks;
        }
        targetAngle = 0.0;
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
    public double getError() { return Math.toDegrees(targetAngle - getTurretAngle()); }
    public double getPower() { return turretPower; }

    public void setOpenLoopPower(double power) {
        double currentAngle = getTurretAngle();
        if (power > 0 && currentAngle >= Math.toRadians(RobotConfig.TURRET_MAX_ANGLE_DEG)) power = 0;
        if (power < 0 && currentAngle <= Math.toRadians(RobotConfig.TURRET_MIN_ANGLE_DEG)) power = 0;
        turretPower = power;
        if (turret != null) turret.setPower(power);
    }

    @Override
    public void stop() {
        trackingEnabled = false;
        if (turret != null) turret.setPower(0);
    }
}