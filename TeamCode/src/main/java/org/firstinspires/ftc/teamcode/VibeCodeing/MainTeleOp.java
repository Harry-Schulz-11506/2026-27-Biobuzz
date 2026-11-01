package org.firstinspires.ftc.teamcode.VibeCodeing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Main TeleOp coordinating Driving/Turret (via Turret subsystem), Shooter/Intake/Transfer and the
 * Flower/Stop servos.
 *
 * The flywheel never stops: it always tracks the regression RPM for the distance we are standing at.
 * Controller 1 A (while held) runs the intake and transfer and opens the stop servo; the triggers on
 * their own run the intake and transfer with the stop servo left blocking. Dpad-left toggles the
 * flower servo.
 */
@Config
@TeleOp(name = "Main TeleOp", group = "Main")
public class MainTeleOp extends OpMode {

    // ---- Dashboard Configuration ----
    public static double INTAKE_POWER = 1.0;
    public static double TRANSFER_POWER = -0.8;
    public static boolean TRANSFER_WITH_INTAKE = true;
    public static double INTAKE_TRANSFER_POWER = 1.0;
    public static boolean REQUIRE_AT_SPEED = true;
    public static double RPM_STEP = 100;

    // ---- Ball counting by transfer motor current ----
    // Each artifact pulled in makes the transfer motor draw a spike of current. While the intake trigger is
    // held, those spikes are counted; at BALL_LIMIT the intake and transfer stop. The count resets when you
    // shoot (right bumper), reverse (left bumper) or press gamepad1 Y (manual reset / override).
    public static boolean BALL_COUNT_ENABLED = false;
    public static int BALL_LIMIT = 4;
    public static double BALL_CURRENT_AMPS = 3.0;      // current above this counts as a ball going through
    public static double BALL_RELEASE_AMPS = 2.0;      // current must fall below this before the next ball counts
    public static double BALL_MIN_GAP_SEC = 0.15;      // ignores spikes closer together than this

    // ---- Jam protection ----
    // If the transfer draws OVERCURRENT_AMPS or more (for OVERCURRENT_HOLD_SEC) the intake and transfer are
    // locked off. They stay off until controller 2 presses A.
    public static boolean OVERCURRENT_ENABLED = true;
    public static double OVERCURRENT_AMPS = 10;
    public static double OVERCURRENT_HOLD_SEC = 0.1;   // must stay above the limit this long (ignores brief spikes)

    // Shooter distance regression
    public static boolean AUTO_RPM_FROM_DISTANCE = true;
    // 1 = straight line through the measured shots (safe outside 31-67 in).
    // 2 = curve, fits the measured band tighter but turns back down past about 70 in.
    public static int REGRESSION_DEGREE = 1;
    public static double MIN_AUTO_RPM = 1000;
    public static double MAX_AUTO_RPM = 6000;

    private Turret turret;
    private Shooter shooter;
    private ShooterRegression regression;
    private boolean feedLatched = false;
    private DcMotorEx intake, transfer;
    private Servo flowerServo, stopServo;

    private boolean intakeLocked = false;
    private double overSince = -1;
    private boolean prevPad2A = false;
    private int ballCount = 0;
    private boolean ballSpikeActive = false;
    private double lastBallTime = -1;

    private double lastDistance = 0;
    private double rpmTrim = 0;
    private double commandedRpm = 0;

    private boolean flowerIsDown = false;
    private boolean prevUp, prevDown, prevLeft, prevL3;
    private double lastLoopTime = -1;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turret = new Turret();
        turret.hardwareMap = hardwareMap;
        turret.gamepad1 = gamepad1;
        turret.gamepad2 = gamepad2;
        turret.telemetry = telemetry;
        turret.updateTelemetry = false; // MainTeleOp calls telemetry.update() once per loop
        turret.init();

        shooter = new Shooter();
        shooter.init(hardwareMap);
        regression = new ShooterRegression(REGRESSION_DEGREE);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servos hold their last position, so start them at their rest state: flower up, stop
        // blocking (0 is the open position, 0.5 holds the artifacts). The positions live in
        // AutoActions so TeleOp and auto cannot drift apart.
        flowerServo = hardwareMap.get(Servo.class, "flower");
        stopServo = hardwareMap.get(Servo.class, "stop");
        flowerServo.setPosition(AutoActions.FLOWER_UP);
        stopServo.setPosition(AutoActions.STOP_BLOCKING);

        telemetry.addLine("Main TeleOp Initialized.");
        telemetry.update();
    }

    @Override
    public void start() {
        turret.start();
        lastLoopTime = System.nanoTime() / 1e9;
    }

    @Override
    public void loop() {
        double now = System.nanoTime() / 1e9;
        double dt = now - lastLoopTime;
        lastLoopTime = now;

        // ---- Turret & Drive Subsystem Loop ----
        turret.loop();

        // Distance to the goal the turret is tracking (BACK / right stick button), used by the RPM regression.
        if (turret.follower != null) {
            Pose pose = turret.follower.pose();
            // Distance to the virtual goal (the goal moved back by the robot's velocity), so the RPM is right on the move.
            lastDistance = Math.hypot(pose.x() - turret.getAimX(), pose.y() - turret.getAimY());
        }

        // ---- Shooter Control (always spinning: there is no on/off toggle any more) ----
        boolean autoRpm = AUTO_RPM_FROM_DISTANCE;
        if (gamepad1.dpad_up && !prevUp) {
            if (autoRpm) rpmTrim += RPM_STEP; else Shooter.TARGET_RPM += RPM_STEP;
        }
        if (gamepad1.dpad_down && !prevDown) {
            if (autoRpm) rpmTrim -= RPM_STEP; else Shooter.TARGET_RPM = Math.max(0, Shooter.TARGET_RPM - RPM_STEP);
        }
        if (regression.getRequestedDegree() != REGRESSION_DEGREE) regression.refit(REGRESSION_DEGREE);
        if (autoRpm) {
            commandedRpm = regression.predict(lastDistance, shooter.getVoltage(), MIN_AUTO_RPM, MAX_AUTO_RPM) + rpmTrim;
            commandedRpm = Math.max(0, Math.min(MAX_AUTO_RPM, commandedRpm));
        } else {
            commandedRpm = Shooter.TARGET_RPM;
        }
        if (gamepad1.left_stick_button && !prevL3) { regression.addSample(lastDistance, commandedRpm, shooter.getVoltage()); }
        shooter.setTargetRpm(commandedRpm);
        shooter.update();

        // ---- Intake / Transfer Control ----
        // A (held) = intake in + transfer in + the stop servo open. The triggers on their own spin
        // the intake and transfer but leave the stop servo blocking, which is what holds the
        // artifacts in. A wins if both are held.
        boolean feedButton = gamepad1.a;
        double intakePower = feedButton
                ? INTAKE_POWER
                : (gamepad1.right_trigger - gamepad1.left_trigger) * INTAKE_POWER;

        // The intake and the transfer always spin together: whichever one is asked to move, both do.
        // Wait for at speed once per bumper press, then keep feeding while it is held. Checking at speed
        // every loop stopped the transfer each time a ball dragged the RPM out of the band.
        if (!gamepad1.right_bumper) feedLatched = false;
        else if (!feedLatched && (!REQUIRE_AT_SPEED || shooter.atSpeed())) feedLatched = true;
        boolean feedToShooter = gamepad1.right_bumper && feedLatched;
        shooter.setFeeding(feedToShooter); // fight the RPM drop while a shot is being fed
        double transferPower = 0;
        if (feedButton) {
            transferPower = INTAKE_TRANSFER_POWER;                 // A: intake and transfer together
        } else if (feedToShooter) {
            transferPower = TRANSFER_POWER;                        // right bumper, once at speed
        } else if (gamepad1.left_bumper) {
            transferPower = -TRANSFER_POWER;
        } else if (Math.abs(intakePower) > 0.05) {
            transferPower = Math.signum(intakePower) * INTAKE_TRANSFER_POWER;
        }
        if (Math.abs(intakePower) <= 0.05 && Math.abs(transferPower) > 0.05) {
            intakePower = Math.signum(transferPower) * INTAKE_POWER; // bumper feed also spins the intake
        }
        // Ball counting: only while the intake trigger is held on its own (not A, not shooting).
        double transferAmps = transfer.getCurrent(CurrentUnit.AMPS);
        if (gamepad1.right_bumper || gamepad1.left_bumper || gamepad1.left_trigger > 0.05 || gamepad1.y) {
            ballCount = 0;
            ballSpikeActive = false;
        }
        boolean intakeTriggerOnly = BALL_COUNT_ENABLED && !feedButton && !feedToShooter
                && gamepad1.right_trigger > 0.05 && transferPower > 0.05;
        if (intakeTriggerOnly) {
            double nowS = System.nanoTime() / 1e9;
            if (!ballSpikeActive && transferAmps >= BALL_CURRENT_AMPS
                    && (lastBallTime < 0 || nowS - lastBallTime >= BALL_MIN_GAP_SEC)) {
                ballCount++;
                ballSpikeActive = true;
                lastBallTime = nowS;
            } else if (ballSpikeActive && transferAmps < BALL_RELEASE_AMPS) {
                ballSpikeActive = false;
            }
            if (ballCount >= BALL_LIMIT) {  // full: intake (and the transfer, which always spins with it) stop
                intakePower = 0;
                transferPower = 0;
            }
        } else {
            ballSpikeActive = false;
        }
        // Jam protection: sustained high transfer current locks the intake + transfer off until pad 2 A.
        double nowOc = System.nanoTime() / 1e9;
        if (OVERCURRENT_ENABLED && !intakeLocked && transferAmps >= OVERCURRENT_AMPS) {
            if (overSince < 0) overSince = nowOc;
            if (nowOc - overSince >= OVERCURRENT_HOLD_SEC) intakeLocked = true;
        } else {
            overSince = -1;
        }
        if (gamepad2.a && !prevPad2A) {
            intakeLocked = false;
            overSince = -1;
        }
        prevPad2A = gamepad2.a;
        if (intakeLocked) {
            intakePower = 0;
            transferPower = 0;
        }
        intake.setPower(Range.clip(intakePower, -1, 1));
        transfer.setPower(transferPower);

        // ---- Stop Servo: open only while A is held, blocking the rest of the time ----
        stopServo.setPosition(feedButton ? AutoActions.STOP_DOWN : AutoActions.STOP_BLOCKING);

        // ---- Flower Servo: controller 1 dpad-left toggles up <-> down ----
        if (gamepad1.dpad_left && !prevLeft) {
            flowerIsDown = !flowerIsDown;
            flowerServo.setPosition(flowerIsDown ? AutoActions.FLOWER_DOWN : AutoActions.FLOWER_UP);
        }

        prevUp = gamepad1.dpad_up; prevDown = gamepad1.dpad_down;
        prevLeft = gamepad1.dpad_left;
        prevL3 = gamepad1.left_stick_button;

        // ---- Additional Telemetry ----
        telemetry.addData("Shot Distance (in)", "%.1f", lastDistance);
        telemetry.addData("RPM Fit", "deg %d, R2 %.3f, %s",
                regression.getDegreeUsed(), regression.getRSquared(), regression.describeSamples());
        telemetry.addData("Shooter RPM", "%.0f / %.0f", shooter.getRpm(), shooter.getTargetRpm());
        telemetry.addData("Battery (V)", "%.2f", shooter.getVoltage());
        telemetry.addData("Voltage effect", regression.describeVoltageEffect());
        telemetry.addData("Shooter At Speed", shooter.atSpeed());
        telemetry.addData("Dip boost", shooter.isRecovering());
        telemetry.addData("Balls counted / limit (Y resets)", "%d / %d", ballCount, BALL_LIMIT);
        telemetry.addData("Intake LOCKED (pad 2 A re-enables)", intakeLocked);
        telemetry.addData("Transfer current (A)", "%.2f", transfer.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Flower / Stop Servo (dpad-left / A)", "%.2f / %.2f",
                flowerServo.getPosition(), stopServo.getPosition());
        telemetry.addData("Loop MS", "%.1f", dt * 1000);
        telemetry.update();
    }

    @Override
    public void stop() {
        turret.stop();
        shooter.stop();
        intake.setPower(0);
        transfer.setPower(0);
    }
}