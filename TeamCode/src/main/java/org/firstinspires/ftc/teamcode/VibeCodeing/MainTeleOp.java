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
    public static double TRANSFER_POWER = 1.0;
    public static boolean TRANSFER_WITH_INTAKE = true;
    public static double INTAKE_TRANSFER_POWER = 1.0;
    public static boolean REQUIRE_AT_SPEED = true;
    public static double RPM_STEP = 100;

    // Shooter distance regression
    public static boolean AUTO_RPM_FROM_DISTANCE = true;
    // 1 = straight line through the measured shots (safe outside 31-67 in).
    // 2 = curve, fits the measured band tighter but turns back down past about 70 in.
    public static int REGRESSION_DEGREE = 1;
    public static double MIN_AUTO_RPM = 1000;
    public static double MAX_AUTO_RPM = 3300;

    private Turret turret;
    private Shooter shooter;
    private ShooterRegression regression;
    private DcMotorEx intake, transfer;
    private Servo flowerServo, stopServo;

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
            lastDistance = Math.hypot(pose.x() - turret.getGoalX(), pose.y() - turret.getGoalY());
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
            commandedRpm = regression.predict(lastDistance, MIN_AUTO_RPM, MAX_AUTO_RPM) + rpmTrim;
            commandedRpm = Math.max(0, Math.min(MAX_AUTO_RPM, commandedRpm));
        } else {
            commandedRpm = Shooter.TARGET_RPM;
        }
        if (gamepad1.left_stick_button && !prevL3) { regression.addSample(lastDistance, commandedRpm); }
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
        intake.setPower(Range.clip(intakePower, -1, 1));

        double transferPower = 0;
        if (feedButton) {
            transferPower = INTAKE_TRANSFER_POWER;                 // A: intake and transfer together
        } else if (gamepad1.right_bumper) {
            boolean blocked = REQUIRE_AT_SPEED && !shooter.atSpeed();
            if (!blocked) transferPower = TRANSFER_POWER;
        } else if (gamepad1.left_bumper) {
            transferPower = -TRANSFER_POWER;
        } else if (TRANSFER_WITH_INTAKE && Math.abs(intakePower) > 0.05) {
            transferPower = Math.signum(intakePower) * INTAKE_TRANSFER_POWER;
        }
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
        telemetry.addData("Shooter At Speed", shooter.atSpeed());
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