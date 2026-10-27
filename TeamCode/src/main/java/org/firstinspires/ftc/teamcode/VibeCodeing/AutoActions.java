package org.firstinspires.ftc.teamcode.VibeCodeing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Auto-side helpers for the VibeCodeing mechanisms: turret tracking, shooter spin-up + feed,
 * intake and transfer.
 *
 * Nothing in here overrides TeleOp: it drives the same hardware through the same Turret,
 * Shooter and ShooterRegression classes, so the gains and the measured RPM/distance table
 * (ShooterRegression.DEFAULT_DIST / DEFAULT_RPM) are shared between TeleOp and auto.
 *
 * Call {@link #update()} EVERY loop, right after follower.update(), so the turret PID and the
 * flywheel PID keep running while Pedro drives. Every method is non-blocking; use the
 * is...() predicates to decide when the next path or feed step should start.
 *
 * Typical auto:
 *
 * <pre>
 *   follower = Constants.create(hardwareMap);
 *   follower.setPose(start);
 *   follower.update();
 *   AutoActions mech = new AutoActions(hardwareMap, follower); // turret must point forward here
 *   mech.spinUpForDistance();                                  // or mech.spinUp(3200)
 *
 *   follower.follow(pathToShootingSpot);
 *   while (opModeIsActive()) {
 *       follower.update();
 *       mech.update();                                         // turret tracks + flywheel spins
 *       if (!follower.isBusy() &amp;&amp; mech.isReadyToShoot()) mech.shoot();
 *       if (mech.isFeeding() &amp;&amp; !mech.isShooterAtSpeed()) mech.stopShoot(); // jams, re-spin
 *   }
 * </pre>
 *
 * The tracking target and the distance used by the RPM fit default to the goal the Turret has selected
 * (RED/BLUE, CELL 1/2); call {@link #trackGoal(boolean, boolean)} to pick one, or {@link #aimAt(double, double)}
 * to track any other field point.
 */
@Config
public class AutoActions {

    // ---- Dashboard configuration ----
    public static double INTAKE_POWER = 1.0;
    public static double TRANSFER_POWER = 1.0;
    public static boolean GATE_FEED_ON_SPEED = true;   // feed only once the flywheel is at speed
    public static int REGRESSION_DEGREE = 1;           // same default as MainTeleOp
    public static double MIN_AUTO_RPM = 1000;
    public static double MAX_AUTO_RPM = 3300;
    public static double AIM_TOLERANCE_DEG = 2.0;

    private final Turret turret = new Turret();
    private final Shooter shooter = new Shooter();
    private final ShooterRegression regression;
    private final DcMotorEx intake, transfer;
    private final Follower follower;

    private static final double STRAIGHT_AHEAD_LOOKAHEAD_IN = 24.0;

    private double targetX = 0;
    private double targetY = 0;
    private boolean tracking = true;
    private boolean straightAhead = false;
    private boolean feeding = false;
    private double distance = 0;
    private double commandedRpm = 0;

    /**
     * @param hw       hardware map
     * @param follower the auto's Pedro follower; its pose feeds the turret and the distance regression.
     *                 May be null (tracking is then skipped, distance stays 0).
     */
    public AutoActions(HardwareMap hw, Follower follower) {
        this.follower = follower;
        initTurret(hw);
        shooter.init(hw);
        regression = new ShooterRegression(REGRESSION_DEGREE);

        intake = hw.get(DcMotorEx.class, "intake");
        transfer = hw.get(DcMotorEx.class, "transfer");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);
        transfer.setPower(0);

        // Start on the goal the Turret has selected, and measure the distance to it right away so
        // spinUpForDistance() is right even before the first update().
        targetX = turret.getGoalX();
        targetY = turret.getGoalY();
        if (follower != null) updateDistance(follower.pose());
    }

    /**
     * Turret setup copied from Turret.init() on purpose: calling Turret.init() here would build a
     * second Pedro Follower (and a second Pinpoint localizer on the same I2C device) inside the auto.
     */
    private void initTurret(HardwareMap hw) {
        turret.turret = hw.get(DcMotorEx.class, "turret");
        turret.turret.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.turret.setPower(0);
        turret.zeroTicks = turret.turret.getCurrentPosition(); // turret 0 deg = straight ahead at init
        turret.follower = follower;
    }

    // ===================== LOOP =====================

    /** Run the turret tracker, the flywheel controller and the gated feed. Call every loop. */
    public void update() {
        update(follower != null ? follower.pose() : null);
    }

    /** Same as {@link #update()} but with a pose you already have. */
    public void update(Pose pose) {
        if (regression.getRequestedDegree() != REGRESSION_DEGREE) regression.refit(REGRESSION_DEGREE);

        if (pose != null) {
            updateDistance(pose);
            if (straightAhead) {
                // Aim at a point 24 in ahead of the robot: that is 0 deg relative to its heading.
                double h = pose.heading();
                turret.updateTurret(pose,
                        pose.x() + STRAIGHT_AHEAD_LOOKAHEAD_IN * Math.cos(h),
                        pose.y() + STRAIGHT_AHEAD_LOOKAHEAD_IN * Math.sin(h),
                        tracking);
            } else {
                turret.updateTurret(pose, targetX, targetY, tracking);
            }
        }

        shooter.update();

        if (feeding) {
            boolean ready = !GATE_FEED_ON_SPEED || shooter.atSpeed();
            transfer.setPower(ready ? TRANSFER_POWER : 0);
        }
    }

    private void updateDistance(Pose pose) {
        distance = Math.hypot(pose.x() - targetX, pose.y() - targetY);
    }

    // ===================== TURRET / TRACKING =====================

    /** Track the given field point (Pedro field inches). Enables tracking. */
    public void aimAt(double fieldX, double fieldY) {
        targetX = fieldX;
        targetY = fieldY;
        tracking = true;
    }

    /** Track the goal the Turret currently has selected (RED/BLUE, CELL 1/2). */
    public void aimAtSelectedGoal() {
        straightAhead = false;
        aimAt(turret.getGoalX(), turret.getGoalY());
    }

    /**
     * Track one of the Turret goal cells: red = RED alliance, cell1 = CELL 1 (upper) instead of CELL 2.
     * The coordinates come from Turret's Dashboard-editable RED_CELL*_X/Y / BLUE_CELL*_X/Y.
     */
    public void trackGoal(boolean red, boolean cell1) {
        straightAhead = false;
        turret.selectGoal(red, cell1);
        aimAt(turret.getGoalX(), turret.getGoalY());
    }


    /** Stop tracking a goal and hold the turret at 0 deg (straight ahead of the robot). */
    public void aimStraightAhead() {
        straightAhead = true;
    }

    /** Resume tracking the current goal after {@link #aimStraightAhead()}. */
    public void resumeTracking() {
        straightAhead = false;
    }

    public boolean isStraightAhead() { return straightAhead; }

    /** false = stop the turret where it is (the motor is set to zero power). */
    public void setTracking(boolean on) {
        tracking = on;
        if (!on) {
            Pose pose = follower != null ? follower.pose() : null;
            if (pose != null) {
                turret.updateTurret(pose, targetX, targetY, false);
            } else {
                turret.setOpenLoopPower(0);
            }
        }
    }

    public boolean isTracking() { return tracking; }
    public double getTargetX() { return targetX; }
    public double getTargetY() { return targetY; }

    /** Turret pointing at the target within AIM_TOLERANCE_DEG. */
    public boolean isAimed() { return isAimed(AIM_TOLERANCE_DEG); }

    public boolean isAimed(double toleranceDeg) {
        return Math.abs(getTurretErrorDeg()) <= toleranceDeg;
    }

    public double getTurretTargetDeg() { return turret.getTarget(); }
    public double getTurretAngleDeg() { return turret.getAngle(); }
    public double getTurretErrorDeg() { return turret.getError(); }

    // ===================== SHOOTER =====================

    /** Spin the flywheel at a fixed RPM. */
    public void spinUp(double rpm) {
        commandedRpm = Math.max(0, Math.min(MAX_AUTO_RPM, rpm));
        shooter.setTargetRpm(commandedRpm);
    }

    /** Spin the flywheel to the RPM the measured distance table wants at the current pose. */
    public void spinUpForDistance() {
        spinUp(regression.predict(getDistanceToTarget(), MIN_AUTO_RPM, MAX_AUTO_RPM));
    }

    public void spinDown() {
        commandedRpm = 0;
        stopShoot();
        shooter.setTargetRpm(0);
    }

    public boolean isShooterAtSpeed() { return shooter.atSpeed(); }
    public double getShooterRpm() { return shooter.getRpm(); }
    public double getShooterTargetRpm() { return shooter.getTargetRpm(); }
    public double getAutoRpm() { return commandedRpm; }
    public double getDistanceToTarget() { return distance; }

    /** Aimed and at speed: the two conditions worth waiting for before feeding. */
    public boolean isReadyToShoot() {
        return isAimed() && isShooterAtSpeed();
    }

    // ===================== FEED (transfer) =====================

    /** Start feeding. With GATE_FEED_ON_SPEED on, the transfer only runs once the flywheel is at speed. */
    public void shoot() {
        feeding = true;
    }

    public void stopShoot() {
        feeding = false;
        transfer.setPower(0);
    }

    public boolean isFeeding() { return feeding; }

    // ===================== INTAKE / TRANSFER (raw) =====================

    public void intakeIn() { intake.setPower(INTAKE_POWER); }
    public void intakeOut() { intake.setPower(-INTAKE_POWER); }
    public void intakeStop() { intake.setPower(0); }

    /** Intake in and transfer in together, the way drive practice picks up artifacts. */
    public void intakeInWithTransfer() {
        intakeIn();
        transferIn();
    }

    public void transferIn() { transfer.setPower(TRANSFER_POWER); }
    public void transferOut() { transfer.setPower(-TRANSFER_POWER); }
    public void transferStop() { transfer.setPower(0); }

    // ===================== TEARDOWN =====================

    /** Turret off, flywheel off, intake and transfer off. Call at the end of auto (TeleOp init resets too). */
    public void stopAll() {
        tracking = false;
        turret.setOpenLoopPower(0);
        spinDown();
        intake.setPower(0);
        transfer.setPower(0);
    }

    // ===================== ACCESS / DEBUG =====================

    public Turret getTurret() { return turret; }
    public Shooter getShooter() { return shooter; }
    public ShooterRegression getRegression() { return regression; }

    /** One-call telemetry for auto tuning: distance, aim, RPM and feed state. */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shot Distance (in)", "%.1f", distance);
        telemetry.addData("Turret Target / Angle / Error (deg)", "%.1f / %.1f / %.1f",
                getTurretTargetDeg(), getTurretAngleDeg(), getTurretErrorDeg());
        telemetry.addData("Shooter RPM", "%.0f / %.0f", getShooterRpm(), getShooterTargetRpm());
        telemetry.addData("Ready To Shoot", isReadyToShoot());
        telemetry.addData("Feeding", feeding);
    }
}
