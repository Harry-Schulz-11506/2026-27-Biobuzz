package org.firstinspires.ftc.teamcode.VibeCodeing;

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
    public static double START_X = 6.0;
    public static double START_Y = 6.0;
    public static double START_HEADING = 0.0;

    // ============================
    // TARGET POSITION
    // ============================
    // Legacy fixed point. Nothing aims or measures with it any more: the turret and the shooter
    // distance follow the selected goal below.
    public static double TARGET_X = 60.0;
    public static double TARGET_Y = 48.0;

    // ============================
    // GOAL SELECTION (alliance cell it tracks)
    // BACK toggles RED <-> BLUE, right stick button toggles CELL 1 <-> CELL 2.
    // ============================
    public static double RED_CELL1_X = 56.0;  public static double RED_CELL1_Y = 87.0;
    public static double RED_CELL2_X = 56.0;  public static double RED_CELL2_Y = 52.0;
    public static double BLUE_CELL1_X = 83.0; public static double BLUE_CELL1_Y = 52.0;
    public static double BLUE_CELL2_X = 83.0; public static double BLUE_CELL2_Y = 87.0;

    // ============================
    // TURRET PIVOT POSITION
    // ============================
    public static double OFFSET_X = -2.0; // 2 inches behind robot center
    public static double OFFSET_Y = 0.0;

    // ============================
    // MOTOR + EXTERNAL ENCODER
    // ============================
    public DcMotorEx turret;
    public static double TICKS_PER_REV = ((((1.0 + (46.0 / 17.0))) * (1.0 + (46.0 / 17.0))) * 28.0) * (80.0 / 20.0);
    public static double RADIANS_PER_TICK = (2.0 * Math.PI) / TICKS_PER_REV;
    public static int ENCODER_SIGN = 1;

    public long zeroTicks = 0;

    // ============================
    // MECHANICAL LIMITS
    // ============================
    public static double MIN_ANGLE = Math.toRadians(-315.0);
    public static double MAX_ANGLE = Math.toRadians(45.0);
    public static double LIMIT_MARGIN = Math.toRadians(5.0);

    // ============================
    // POSITION CONTROL
    // ============================
    public static double kP = 0.7;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double MAX_POWER = 0.5;
    public static double TOLERANCE = Math.toRadians(1.0);

    // ---- Driver 2 aim trim (gamepad2 D-pad left/right, D-pad down resets) ----
    public static double TRIM_RATE_DEG_PER_SEC = 20.0; // how fast the trim moves while the D-pad is held
    public static double TRIM_MAX_DEG = 30.0;          // trim is limited to +/- this
    public double trimRad = 0.0;                       // added to the tracked target angle
    private long lastTrimNanos = 0;

    public double targetAngle = 0.0;
    public double turretPower = 0.0;
    public boolean trackingEnabled = false;
    public boolean previousA = false;

    /**
     * true  = Turret calls telemetry.update() itself (running as its own OpMode).
     * false = the owner (MainTeleOp) calls telemetry.update() once per loop, so the turret lines
     *         are shown together with the shooter lines instead of being overwritten.
     */
    public boolean updateTelemetry = true;

    private boolean redAlliance = true;   // starts on RED
    private boolean firstCell = true;     // starts on CELL 1
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
            follower.setPose(new Pose(START_X, START_Y, START_HEADING));

            turret = hardwareMap.get(DcMotorEx.class, "turret");
            turret.setDirection(DcMotorSimple.Direction.FORWARD);
            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turret.setPower(0);

            zeroTicks = turret.getCurrentPosition();
        }

        if (telemetry != null) {
            telemetry.addLine("Turret initialized");
            telemetry.addLine("Tracking disabled");
            telemetry.addLine("Gamepad2 A: enable tracking");
            telemetry.addLine("Gamepad2 B: disable tracking");
            telemetry.addLine("Gamepad2 D-pad L/R: aim trim, D-pad down: reset trim");
            if (updateTelemetry) telemetry.update();
        }
    }

    public double getTurretAngle() {
        if (turret == null) return 0.0;
        long ticks = (long) turret.getCurrentPosition() - zeroTicks;
        return ticks * ENCODER_SIGN * RADIANS_PER_TICK;
    }

    public double getAngle() {
        return Math.toDegrees(getTurretAngle());
    }

    public double getWrappedAngle() {
        return Math.toDegrees(getTurretAngle());
    }

    public double getTicksPerDegree() {
        return TICKS_PER_REV / 360.0;
    }

    public long getRawTicks() {
        if (turret == null) return 0;
        return (long) turret.getCurrentPosition() - zeroTicks;
    }

    public static double wrapToTurretRange(double angle) {
        double twoPi = 2.0 * Math.PI;
        return ((angle - MIN_ANGLE) % twoPi + twoPi) % twoPi + MIN_ANGLE;
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
        if (redAlliance) return firstCell ? RED_CELL1_X : RED_CELL2_X;
        return firstCell ? BLUE_CELL1_X : BLUE_CELL2_X;
    }

    /** Field Y of the goal currently selected (Pedro inches, Dashboard-editable above). */
    public double getGoalY() {
        if (redAlliance) return firstCell ? RED_CELL1_Y : RED_CELL2_Y;
        return firstCell ? BLUE_CELL1_Y : BLUE_CELL2_Y;
    }

    public double calculateTarget(Pose pose) {
        return calculateTarget(pose, getGoalX(), getGoalY());
    }

    public double calculateTarget(Pose pose, double targetX, double targetY) {
        double heading = pose.heading();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double turretX = pose.x() + OFFSET_X * cos - OFFSET_Y * sin;
        double turretY = pose.y() + OFFSET_X * sin + OFFSET_Y * cos;

        double fieldAngle = Math.atan2(targetY - turretY, targetX - turretX);
        double relativeAngle = fieldAngle - heading;

        return wrapToTurretRange(relativeAngle);
    }

    public void updateTurret(Pose pose) {
        updateTurret(pose, getGoalX(), getGoalY(), trackingEnabled);
    }

    public void updateTurret(Pose pose, double targetX, double targetY, boolean tracking) {
        this.trackingEnabled = tracking;
        targetAngle = wrapToTurretRange(calculateTarget(pose, targetX, targetY) + trimRad);
        double currentAngle = getTurretAngle();
        double error = targetAngle - currentAngle;

        if (!trackingEnabled || turret == null) {
            turretPower = 0.0;
            if (turret != null) turret.setPower(0);
            return;
        }

        double power = Range.clip(kP * error, -MAX_POWER, MAX_POWER);

        if (Math.abs(error) < TOLERANCE) {
            power = 0.0;
        }

        if (power > 0) {
            double remaining = MAX_ANGLE - currentAngle;
            if (remaining <= 0) {
                power = 0;
            } else if (remaining < LIMIT_MARGIN) {
                power *= remaining / LIMIT_MARGIN;
            }
        } else if (power < 0) {
            double remaining = currentAngle - MIN_ANGLE;
            if (remaining <= 0) {
                power = 0;
            } else if (remaining < LIMIT_MARGIN) {
                power *= remaining / LIMIT_MARGIN;
            }
        }

        turretPower = power;
        turret.setPower(power);
    }

    @Override
    public void loop() {
        if (follower != null && gamepad1 != null) {
            follower.manual(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );
            follower.update();
            Pose pose = follower.pose();

            if (gamepad2 != null) {
                long nowN = System.nanoTime();
                double dtTrim = lastTrimNanos == 0 ? 0 : Math.min((nowN - lastTrimNanos) / 1e9, 0.1);
                lastTrimNanos = nowN;
                double dir = (gamepad2.dpad_right ? -1 : 0) + (gamepad2.dpad_left ? 1 : 0); // left = CCW (+)
                trimRad += Math.toRadians(TRIM_RATE_DEG_PER_SEC) * dir * dtTrim;
                trimRad = Range.clip(trimRad, -Math.toRadians(TRIM_MAX_DEG), Math.toRadians(TRIM_MAX_DEG));
                if (gamepad2.dpad_down) trimRad = 0;

                boolean a = gamepad2.a;
                if (a && !previousA) {
                    trackingEnabled = true;
                }
                previousA = a;
                if (gamepad2.b) {
                    trackingEnabled = false;
                }
            }

            // BACK: swap RED <-> BLUE. Right stick button: swap CELL 1 <-> CELL 2.
            if (gamepad1.back && !previousBack) toggleRedBlue();
            if (gamepad1.right_stick_button && !previousR3) toggleCell();
            previousBack = gamepad1.back;
            previousR3 = gamepad1.right_stick_button;

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
                telemetry.addData("Aim Trim (deg)", "%.1f", Math.toDegrees(trimRad));
                if (updateTelemetry) telemetry.update();
            }
        }
    }

    public void zeroHere() {
        if (turret != null) zeroTicks = turret.getCurrentPosition();
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
        if (power > 0 && currentAngle >= MAX_ANGLE) power = 0;
        if (power < 0 && currentAngle <= MIN_ANGLE) power = 0;
        turretPower = power;
        if (turret != null) turret.setPower(power);
    }

    @Override
    public void stop() {
        trackingEnabled = false;
        if (turret != null) turret.setPower(0);
    }
}