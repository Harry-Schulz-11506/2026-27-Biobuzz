package org.firstinspires.ftc.teamcode.NATS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

//6/11/26  11:04AM
;
@Config
@TeleOp(name = "Main TeleOp", group = "Main")
public class MainTeleOp extends OpMode {

    private Turret turret;
    private ShooterMechanism shooter;
    private ShooterRegression regression;
    private IntakeMechanism intake;
    private StopperMechanism stopper;
    private FlowerMechanism flower;
    private PivotMechanism pivot;

    private boolean feedLatched = false;
    private double shootHoldUntil = 0;   // time (s) until which both intake motors keep running after a shot
    private double lastDistance = 0;
    private double lastX = 0, lastY = 0;
    private double rpmTrim = 0;
    private double commandedRpm = 0;
    private boolean flowerIsDown = false;
    private double flowerPos = RobotConfig.FLOWER_UP;
    private boolean prevUp, prevDown, prevLeft, prevL3;
    private boolean prevUp2, prevDown2;
    private double lastLoopTime = -1;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize mechanisms
        turret = new Turret();
        turret.hardwareMap = hardwareMap;
        turret.gamepad1 = gamepad1;
        turret.gamepad2 = gamepad2;
        turret.telemetry = telemetry;
        turret.updateTelemetry = false;
        turret.init();

        shooter = new ShooterMechanism();
        shooter.init(hardwareMap);
        regression = new ShooterRegression(RobotConfig.REGRESSION_DEGREE);

        intake = new IntakeMechanism();
        intake.init(hardwareMap);

        stopper = new StopperMechanism();
        stopper.init(hardwareMap);

        flower = new FlowerMechanism();
        flower.init(hardwareMap);

        pivot = new PivotMechanism();
        pivot.init(hardwareMap);

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
            lastX = pose.x();
            lastY = pose.y();
        }

        // ---- Shooter Control (always spinning) ----
        boolean autoRpm = RobotConfig.AUTO_RPM_FROM_DISTANCE;
        boolean up2 = gamepad2.dpad_up, down2 = gamepad2.dpad_down;
        if (up2 && !prevUp2) {
            if (autoRpm) rpmTrim += RobotConfig.RPM_STEP; else RobotConfig.SHOOTER_TARGET_RPM += RobotConfig.RPM_STEP;
        }
        if (down2 && !prevDown2) {
            if (autoRpm) rpmTrim -= RobotConfig.RPM_STEP; else RobotConfig.SHOOTER_TARGET_RPM = Math.max(0, RobotConfig.SHOOTER_TARGET_RPM - RobotConfig.RPM_STEP);
        }
        prevUp2 = up2; prevDown2 = down2;
        if (gamepad2.y) rpmTrim = 0;
        if (gamepad1.dpad_up && !prevUp) {
            if (autoRpm) rpmTrim += RobotConfig.RPM_STEP; else RobotConfig.SHOOTER_TARGET_RPM += RobotConfig.RPM_STEP;
        }
        if (gamepad1.dpad_down && !prevDown) {
            if (autoRpm) rpmTrim -= RobotConfig.RPM_STEP; else RobotConfig.SHOOTER_TARGET_RPM = Math.max(0, RobotConfig.SHOOTER_TARGET_RPM - RobotConfig.RPM_STEP);
        }
        if (regression.getRequestedDegree() != RobotConfig.REGRESSION_DEGREE) regression.refit(RobotConfig.REGRESSION_DEGREE);
        if (autoRpm) {
            commandedRpm = regression.predict(lastX, lastY, turret.getGoalX(), turret.getGoalY(), !turret.isRedAlliance(), shooter.getVoltage(), RobotConfig.SHOOTER_MIN_RPM, RobotConfig.SHOOTER_MAX_RPM) + rpmTrim;
            commandedRpm = Math.max(0, Math.min(RobotConfig.SHOOTER_MAX_RPM, commandedRpm));
        } else {
            commandedRpm = RobotConfig.SHOOTER_TARGET_RPM;
        }
        // Controller 2 left stick press records the shot into the table
        if (gamepad2.left_stick_button && !prevL3) { regression.addSample(lastX, lastY, turret.getGoalX(), turret.getGoalY(), !turret.isRedAlliance(), commandedRpm, shooter.getVoltage()); }
        shooter.setTargetRpm(commandedRpm);
        shooter.update();

        // ---- Intake Control ----
        double nowShoot = System.nanoTime() / 1e9;

        // Gate the shot once per bumper press
        if (!gamepad1.right_bumper) feedLatched = false;
        else if (!feedLatched && (!RobotConfig.REQUIRE_AT_SPEED || shooter.atSpeed())) feedLatched = true;
        boolean shooting = gamepad1.right_bumper && feedLatched;
        shooter.setFeeding(shooting);

        if (shooting) shootHoldUntil = nowShoot + RobotConfig.SHOOT_STOP_DELAY_SEC;
        boolean shootTail = !shooting && nowShoot < shootHoldUntil;

        // Intake state machine
        if (gamepad1.a) {
            intake.setState(IntakeMechanism.State.INTAKE);
        } else if (gamepad1.left_bumper) {
            intake.setState(IntakeMechanism.State.OUTTAKE);
            shootHoldUntil = 0;
        } else if (gamepad1.right_trigger > 0.05 || gamepad1.left_trigger > 0.05) {
            double power = (gamepad1.right_trigger - gamepad1.left_trigger) * RobotConfig.INTAKE_POWER;
            intake.setManualPower(power);
            intake.setState(IntakeMechanism.State.MANUAL);
            if (!shooting) shootHoldUntil = 0;
        } else if (shooting || shootTail) {
            intake.setState(IntakeMechanism.State.INTAKE);
        } else {
            intake.setState(IntakeMechanism.State.STOPPED);
        }

        // ---- Stop Servo: open only while shooting ----
        stopper.setPosition(shooting || shootTail ? StopperMechanism.Position.OPEN : StopperMechanism.Position.BLOCKING);

        // ---- Flower Servo: controller 1 dpad-left toggles up <-> down ----
        if (gamepad1.dpad_left && !prevLeft) {
            flowerIsDown = !flowerIsDown;
            flower.setPreset(flowerIsDown ? FlowerMechanism.Preset.DOWN : FlowerMechanism.Preset.UP);
        }
        // Controller 2 bumpers fine-adjust the flower
        double flowerDir = turret.isCornerMode() ? 0 : (gamepad2.right_bumper ? 1 : 0) - (gamepad2.left_bumper ? 1 : 0);
        if (flowerDir != 0) {
            flower.fineAdjust((int) flowerDir, dt);
        }

        // ---- Pivot: auto-managed by intake state (optional - can add button control) ----
        // Pivot is typically controlled in auto; in TeleOp it stays up

        prevUp = gamepad1.dpad_up; prevDown = gamepad1.dpad_down;
        prevLeft = gamepad1.dpad_left;
        prevL3 = gamepad2.left_stick_button;

        // ---- Additional Telemetry ----
        telemetry.addData("Shot X / Y / dist to goal (in)", "%.1f / %.1f / %.1f", lastX, lastY, lastDistance);
        telemetry.addData("RPM Fit", "deg %d, R2 %.3f, %s",
                regression.getDegreeUsed(), regression.getRSquared(), regression.describeSamples());
        shooter.addTelemetry(telemetry);
        telemetry.addData("Voltage effect", regression.describeVoltageEffect());
        intake.addTelemetry(telemetry);
        stopper.addTelemetry(telemetry);
        flower.addTelemetry(telemetry);
        telemetry.addData("Loop MS", "%.1f", dt * 1000);
        telemetry.update();
    }

    @Override
    public void stop() {
        turret.stop();
        shooter.stop();
        intake.stop();
        stopper.stop();
        flower.stop();
        pivot.stop();
    }
}
