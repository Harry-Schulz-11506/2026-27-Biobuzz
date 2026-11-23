package org.firstinspires.ftc.teamcode.NATS;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Auto-side helpers for the NATS mechanisms: turret tracking, shooter spin-up + feed,
 * intake and transfer, and the flower / stop servos.
 *
 * Nothing in here overrides TeleOp: it drives the same hardware through the same Turret,
 * Shooter and ShooterRegression classes, so the gains and the measured RPM/distance table
 * (ShooterRegression X, Y -> RPM rows) are shared between TeleOp and auto.
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


    // ---- Flower / stop servo positions ----
    // Servo positions are 0.0 to 1.0; the minus in "0 up / -0.6 down" is the direction of travel.
    // RobotConfig.STOP_DOWN is the open position (pollen pass), RobotConfig.STOP_BLOCKING holds them back.

    private final Turret turret = new Turret();
    private final Shooter shooter = new Shooter();
    private final ShooterRegression regression;
    private final DcMotorEx intake, transfer;
    private final Servo flowerServo, stopServo;          // may be null if the robot no longer has them
    private final Servo pivotLeft, pivotRight;            // intake pivot (2 servos)
    private final Follower follower;

    private static final double STRAIGHT_AHEAD_LOOKAHEAD_IN = 24.0;

    private double targetX = 0;
    private double targetY = 0;
    private boolean tracking = true;
    private boolean straightAhead = false;
    private boolean feeding = false;
    private double distance = 0;   // distance to the tracked goal (telemetry only)
    private double poseX = 0, poseY = 0; // robot field position: what the RPM table uses
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
        regression = new ShooterRegression(RobotConfig.REGRESSION_DEGREE);

        intake = hw.get(DcMotorEx.class, RobotConfig.INTAKE_MOTOR_1);
        transfer = hw.get(DcMotorEx.class, RobotConfig.INTAKE_MOTOR_2);
        intake.setDirection(RobotConfig.INTAKE_MOTOR_1_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(RobotConfig.INTAKE_MOTOR_2_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);
        transfer.setPower(0);

        // Servos hold their last position, so start them at their rest state: flower up, stop
        // blocking, since 0 is the open position.
        flowerServo = hw.tryGet(Servo.class, RobotConfig.FLOWER_SERVO);
        stopServo = hw.tryGet(Servo.class, RobotConfig.STOP_SERVO);
        if (flowerServo != null) flowerServo.setPosition(RobotConfig.FLOWER_UP);
        if (stopServo != null) stopServo.setPosition(RobotConfig.STOP_BLOCKING);

        // Intake pivot: 2 servos that move together; the right one is mirrored when PIVOT_RIGHT_REVERSED.
        pivotLeft = hw.tryGet(Servo.class, RobotConfig.PIVOT_SERVO_LEFT);
        pivotRight = hw.tryGet(Servo.class, RobotConfig.PIVOT_SERVO_RIGHT);
        pivotUp();

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
        turret.turret = hw.get(DcMotorEx.class, RobotConfig.TURRET_MOTOR);
        turret.turret.setDirection(RobotConfig.TURRET_MOTOR_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        turret.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.turret.setPower(0);
        turret.zeroTicks = turret.turret.getCurrentPosition(); // turret 0 deg = straight ahead at init
        PoseStorage.turretZeroTicks = (int) turret.zeroTicks;  // TeleOp keeps this zero, so the turret position carries over
        turret.follower = follower;
        turret.vision = new TipDetector(); // HuskyLens: fine aim + tip height
        turret.vision.init(hw);
    }

    // ===================== LOOP =====================

    /** Run the turret tracker, the flywheel controller and the gated feed. Call every loop. */
    public void update() {
        update(follower != null ? follower.pose() : null);
    }

    /** Same as {@link #update()} but with a pose you already have. */
    public void update(Pose pose) {
        if (regression.getRequestedDegree() != RobotConfig.REGRESSION_DEGREE) regression.refit(RobotConfig.REGRESSION_DEGREE);
        if (pose != null) PoseStorage.lastPose = pose; // TeleOp starts from where the auto left the robot

        if (pose != null) {
            updateDistance(pose);
            turret.updateVision();
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

        shooter.setFeeding(feeding && feedLatched); // fight the RPM drop while a shot is being fed
        shooter.update();

        if (feeding) {
            // Wait for the flywheel to be at speed ONCE, then keep feeding until stopShoot(). Checking
            // at speed every loop stopped the transfer each time a ball dragged the RPM out of the band.
            if (!feedLatched && (!RobotConfig.GATE_FEED_ON_SPEED || shooter.atSpeed())) feedLatched = true;
            double power = feedLatched ? RobotConfig.INTAKE_POWER : 0;
            intake.setPower(power);
            transfer.setPower(power); // both motors always get the same command
        }
    }

    private void updateDistance(Pose pose) {
        distance = Math.hypot(pose.x() - targetX, pose.y() - targetY);
        poseX = pose.x();
        poseY = pose.y();
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

    /** Turret pointing at the target within RobotConfig.TURRET_AIM_TOLERANCE_DEG. */
    public boolean isAimed() { return isAimed(RobotConfig.TURRET_AIM_TOLERANCE_DEG); }

    public boolean isAimed(double toleranceDeg) {
        return Math.abs(getTurretErrorDeg()) <= toleranceDeg;
    }

    public double getTurretTargetDeg() { return turret.getTarget(); }
    public double getTurretAngleDeg() { return turret.getAngle(); }
    public double getTurretErrorDeg() { return turret.getError(); }

    // ===================== SHOOTER =====================

    /** Spin the flywheel at a fixed RPM. */
    public void spinUp(double rpm) {
        commandedRpm = Math.max(0, Math.min(RobotConfig.SHOOTER_MAX_RPM, rpm));
        shooter.setTargetRpm(commandedRpm);
    }

    /** Spin the flywheel to the RPM the shot table (field X, Y) wants at the current pose. */
    public void spinUpForDistance() {
        spinUp(regression.predict(poseX, poseY, targetX, targetY, !turret.isRedAlliance(), shooter.getVoltage(), RobotConfig.SHOOTER_MIN_RPM, RobotConfig.SHOOTER_MAX_RPM));
    }

    /** Same, plus rpmOffset (negative = slower). Only the autos that pass an offset are affected. */
    public void spinUpForDistance(double rpmOffset) {
        spinUp(regression.predict(poseX, poseY, targetX, targetY, !turret.isRedAlliance(), shooter.getVoltage(), RobotConfig.SHOOTER_MIN_RPM, RobotConfig.SHOOTER_MAX_RPM) + rpmOffset);
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

    /** Start feeding. With RobotConfig.GATE_FEED_ON_SPEED on, the transfer only runs once the flywheel is at speed. */
    public void shoot() {
        stopDown(); // stopper open so the pollen can reach the flywheel
        feeding = true;
        feedLatched = false;
    }

    /** Start feeding immediately, without waiting for the flywheel to be at speed (used after a ready timeout). */
    public void shootNow() {
        stopDown(); // stopper open so the pollen can reach the flywheel
        feeding = true;
        feedLatched = true;
    }

    public void stopShoot() {
        stopBlocking(); // shot over: stopper closed again
        feeding = false;
        feedLatched = false;
        transfer.setPower(0);
        intake.setPower(0);
    }

    public boolean isFeeding() { return feeding; }

    private boolean feedLatched = false; // true once the flywheel was at speed for this shot

    // ===================== INTAKE / TRANSFER (raw) =====================

    // The intake and the transfer always spin together: every one of these moves both motors.

    public void intakeIn() { bothIn(); }
    public void intakeOut() { bothOut(); }
    public void intakeStop() { bothStop(); }

    /** Intake in and transfer in together, the way drive practice picks up pollen. */
    public void intakeInWithTransfer() { bothIn(); }

    public void transferIn() { bothIn(); }
    public void transferOut() { bothOut(); }
    public void transferStop() { bothStop(); }

    // The stopper is open (RobotConfig.STOP_DOWN) only while shooting; it blocks the rest of the time, intaking included.
    private void bothIn() { stopBlocking(); intake.setPower(RobotConfig.INTAKE_POWER); transfer.setPower(RobotConfig.INTAKE_POWER); }
    private void bothOut() { intake.setPower(-RobotConfig.INTAKE_POWER); transfer.setPower(-RobotConfig.INTAKE_POWER); }
    private void bothStop() { intake.setPower(0); transfer.setPower(0); stopBlocking(); }

    // ===================== FLOWER / STOP SERVOS =====================

    /** Flower up (position 0). */
    public void flowerUp() { if (flowerServo != null) flowerServo.setPosition(RobotConfig.FLOWER_UP); }

    /** Flower down (position 0.6). */
    public void flowerDown() { if (flowerServo != null) flowerServo.setPosition(RobotConfig.FLOWER_DOWN); }

    /** Stop servo down (position 0): the open position, pollen pass through. */
    public void stopDown() { if (stopServo != null) stopServo.setPosition(RobotConfig.STOP_DOWN); }

    /** Stop servo raised (position 0.5): the stopping position, pollen are held back. */
    public void stopBlocking() { if (stopServo != null) stopServo.setPosition(RobotConfig.STOP_BLOCKING); }

    // ===================== INTAKE PIVOT (2 servos) =====================

    /** Move both pivot servos to a position (0..1). The right servo is mirrored if PIVOT_RIGHT_REVERSED. */
    public void setPivot(double pos) {
        if (pivotLeft != null) pivotLeft.setPosition(pos);
        if (pivotRight != null) pivotRight.setPosition(RobotConfig.PIVOT_RIGHT_REVERSED ? 1.0 - pos : pos);
    }
    public void pivotUp() { setPivot(RobotConfig.PIVOT_UP); }
    public void pivotDown() { setPivot(RobotConfig.PIVOT_DOWN); }
    public void pivotMid() { setPivot(RobotConfig.PIVOT_MID); }

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
    /** The HuskyLens tip detector (height UP/DOWN, second-cycle decision). */
    public TipDetector getTipDetector() { return turret.vision; }
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
        if (flowerServo != null && stopServo != null) {
            telemetry.addData("Flower / Stop Servo", "%.2f / %.2f", flowerServo.getPosition(), stopServo.getPosition());
        }
        if (turret.vision != null) turret.vision.addTelemetry(telemetry);
    }
}