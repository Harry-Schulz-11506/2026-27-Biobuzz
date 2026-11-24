package org.firstinspires.ftc.teamcode.NATS;

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


//6/11/26  11:04AM
;
@Config
@TeleOp(name = "Main TeleOp", group = "Main")
public class MainTeleOp extends OpMode {


    private Turret turret;
    private Shooter shooter;
    private ShooterRegression regression;
    private boolean feedLatched = false;
    private double shootHoldUntil = 0;   // time (s) until which the intake + transfer keep running after a shot
    private DcMotorEx intake, intake2;
    private Servo flowerServo, stopServo;

    private double lastDistance = 0;   // distance to the tracked goal (telemetry only now)
    private double lastX = 0, lastY = 0; // robot field position: what the RPM table uses
    private double rpmTrim = 0;
    private double commandedRpm = 0;

    private boolean flowerIsDown = false;
    private double flowerPos = RobotConfig.FLOWER_UP;   // where the flower servo is commanded (bumpers nudge it)
    private boolean prevUp, prevDown, prevLeft, prevL3;
    private boolean prevUp2, prevDown2;
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
        regression = new ShooterRegression(RobotConfig.REGRESSION_DEGREE);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake2 = hardwareMap.get(DcMotorEx.class, RobotConfig.INTAKE_MOTOR_2);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servos hold their last position, so start them at their rest state: flower up, stop
        // blocking (0 is the open position, 0.5 holds the pollen). The positions live in
        // AutoActions so TeleOp and auto cannot drift apart.
        flowerServo = hardwareMap.get(Servo.class, "flower");
        stopServo = hardwareMap.get(Servo.class, RobotConfig.STOP_SERVO);
        flowerServo.setPosition(RobotConfig.FLOWER_UP);
        stopServo.setPosition(RobotConfig.STOP_BLOCKING);

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

        // ---- Shooter Control (always spinning: there is no on/off toggle any more) ----
        boolean autoRpm = RobotConfig.AUTO_RPM_FROM_DISTANCE;
        // Controller 2: D-pad up / down adjust the RPM, Y resets the RPM trim (the turret trim resets on the same press).
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
        // Controller 2 left stick press records the shot into the table (robot X, Y relative to the selected goal, the RPM and the battery voltage); the drive is controller 1's sticks.
        if (gamepad2.left_stick_button && !prevL3) { regression.addSample(lastX, lastY, turret.getGoalX(), turret.getGoalY(), !turret.isRedAlliance(), commandedRpm, shooter.getVoltage()); }
        shooter.setTargetRpm(commandedRpm);
        shooter.update();

        // ---- Intake / Transfer Control (2 motors, always the same power) ----
        // The two motors both drive the intake + transfer, so they are never commanded separately.
        //   A (hold)      = intake: both motors in (the stop servo stays closed)
        //   Right bumper  = SHOOT, the only shoot control: opens the stop servo and feeds, but only once
        //                   the flywheel is at speed, and keeps feeding for SHOOT_STOP_DELAY_SEC after release
        //   Left bumper   = both motors reversed (unjam)
        //   Triggers      = both motors forward / reverse, proportional
        double nowShoot = System.nanoTime() / 1e9;

        // Gate the shot once per bumper press, so a ball dragging the RPM out of the band does not stop it.
        if (!gamepad1.right_bumper) feedLatched = false;
        else if (!feedLatched && (!RobotConfig.REQUIRE_AT_SPEED || shooter.atSpeed())) feedLatched = true;
        boolean shooting = gamepad1.right_bumper && feedLatched;
        shooter.setFeeding(shooting); // fight the RPM drop while a shot is being fed

        if (shooting) shootHoldUntil = nowShoot + RobotConfig.SHOOT_STOP_DELAY_SEC;
        boolean shootTail = !shooting && nowShoot < shootHoldUntil; // the post-shot run-on

        double feedPower;
        if (gamepad1.a) {
            feedPower = RobotConfig.INTAKE_POWER;                     // intake
        } else if (gamepad1.left_bumper) {
            feedPower = -RobotConfig.INTAKE_POWER;                    // unjam
            shootHoldUntil = 0;
        } else if (gamepad1.right_trigger > 0.05 || gamepad1.left_trigger > 0.05) {
            feedPower = (gamepad1.right_trigger - gamepad1.left_trigger) * RobotConfig.INTAKE_POWER;
            if (!shooting) shootHoldUntil = 0;                        // a manual control cancels the run-on
        } else if (shooting || shootTail) {
            feedPower = RobotConfig.INTAKE_POWER;
        } else {
            feedPower = 0;
        }

        feedPower = Range.clip(feedPower, -1, 1);
        intake.setPower(feedPower);
        intake2.setPower(feedPower); // both motors always get the same command

        // ---- Stop Servo: open only while shooting (and through the post-shot run-on) ----
        // Gamepad 1 B (hold) forces it open by hand: use it to check the servo and its two positions.
        boolean stopOpen = shooting || shootTail || gamepad1.b;
        stopServo.setPosition(stopOpen ? RobotConfig.STOP_DOWN : RobotConfig.STOP_BLOCKING);

        // ---- Flower Servo: controller 1 dpad-left toggles up <-> down ----
        if (gamepad1.dpad_left && !prevLeft) {
            flowerIsDown = !flowerIsDown;
            flowerPos = flowerIsDown ? RobotConfig.FLOWER_DOWN : RobotConfig.FLOWER_UP;
            flowerServo.setPosition(flowerPos);
        }
        // Controller 2 bumpers fine-adjust the flower: right bumper moves it toward UP, left toward DOWN
        // (held = keeps moving at RobotConfig.FLOWER_ADJUST_PER_SEC). The dpad-left toggle still jumps to the preset.
        double flowerDir = turret.isCornerMode() ? 0 : (gamepad2.right_bumper ? 1 : 0) - (gamepad2.left_bumper ? 1 : 0); // START held = corner buttons
        if (flowerDir != 0) {
            double lo = Math.min(RobotConfig.FLOWER_UP, RobotConfig.FLOWER_DOWN);
            double hi = Math.max(RobotConfig.FLOWER_UP, RobotConfig.FLOWER_DOWN);
            double up = Math.signum(RobotConfig.FLOWER_UP - RobotConfig.FLOWER_DOWN); // which way "up" is
            flowerPos = Math.max(lo, Math.min(hi, flowerPos + flowerDir * up * RobotConfig.FLOWER_ADJUST_PER_SEC * dt));
            flowerServo.setPosition(flowerPos);
        }

        prevUp = gamepad1.dpad_up; prevDown = gamepad1.dpad_down;
        prevLeft = gamepad1.dpad_left;
        prevL3 = gamepad2.left_stick_button;

        // ---- Additional Telemetry ----
        telemetry.addData("Shot X / Y / dist to goal (in)", "%.1f / %.1f / %.1f", lastX, lastY, lastDistance);
        telemetry.addData("RPM Fit", "deg %d, R2 %.3f, %s",
                regression.getDegreeUsed(), regression.getRSquared(), regression.describeSamples());
        telemetry.addData("Shooter RPM", "%.0f / %.0f", shooter.getRpm(), shooter.getTargetRpm());
        telemetry.addData("Battery (V)", "%.2f", shooter.getVoltage());
        telemetry.addData("Voltage effect", regression.describeVoltageEffect());
        telemetry.addData("Shooter At Speed", shooter.atSpeed());
        telemetry.addData("Dip boost", shooter.isRecovering());
        telemetry.addData("Flower / Stop Servo (dpad-left, pad 2 bumpers / right bumper, B = stopper open)", "%.2f / %.2f",
                flowerServo.getPosition(), stopServo.getPosition());
        telemetry.addData("Loop MS", "%.1f", dt * 1000);
        telemetry.update();
    }

    @Override
    public void stop() {
        turret.stop();
        shooter.stop();
        intake.setPower(0);
        intake2.setPower(0);
    }
}