package org.firstinspires.ftc.teamcode.NATS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Bench test TeleOp: drive, an adjustable flywheel RPM, and the 2 intake motors.
 *
 * There is no transfer motor: the intake is "intake" and "intake2", mounted mirrored so they spin
 * opposite ways (INTAKE_MOTOR_1_REVERSED / INTAKE_MOTOR_2_REVERSED handle that). Both always get
 * the same power, so the intake has two modes: FORWARD (pollen in) and BACKWARD (pollen out).
 *
 * No RPM table, no at-speed gate, no auto aiming, no auto cell swap: every mechanism is on its
 * own button so it can be watched while the robot is driven. All numbers come from RobotConfig.
 *
 * Driver (gamepad 1):
 *   Left stick            drive, right stick turn (same signs as Main TeleOp, via Turret.loop)
 *   D-pad up / down       raise / lower the target flywheel RPM by RobotConfig.RPM_STEP
 *   Right bumper          flywheel on / off at that RPM
 *   Y                     back to RobotConfig.SHOOTER_TARGET_RPM, flywheel off
 *   A                     intake FORWARD mode (both motors, RobotConfig.INTAKE_POWER) - press again to stop
 *   B                     intake BACKWARD mode (both motors, RobotConfig.OUTTAKE_POWER) - press again to stop
 *   X                     intake STOP
 *   Left bumper           toggle the stop servo open / blocking
 *
 * Gamepad 2 keeps the turret controls it has in Main TeleOp (A track, B hold 0, X relocalize,
 * Y reset trim, START + button = corner relocalize), because Turret.loop() runs the drive.
 */
@TeleOp(name = "Test TeleOp", group = "Test")
public class TestTeleOp extends OpMode {

    private Turret turret;
    private Shooter shooter;
    private DcMotorEx intake, intake2;
    private Servo flowerServo, stopServo;

    private double targetRpm = RobotConfig.SHOOTER_TARGET_RPM;
    private boolean flywheelOn = false;
    private int intakeMode = 0;        // +1 = forward, -1 = backward, 0 = stopped
    private boolean gateOpen = false;

    private boolean prevUp, prevDown, prevY, prevA, prevB, prevX, prevRightBumper, prevLeftBumper;
    private double lastLoopTime = -1;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turret = new Turret();
        turret.hardwareMap = hardwareMap;
        turret.gamepad1 = gamepad1;
        turret.gamepad2 = gamepad2;
        turret.telemetry = telemetry;
        turret.updateTelemetry = false;   // this opmode calls telemetry.update() once per loop
        turret.init();

        shooter = new Shooter();
        shooter.init(hardwareMap);

        // The 2 intake motors are mounted mirrored, so the direction switches are opposite.
        intake = hardwareMap.get(DcMotorEx.class, RobotConfig.INTAKE_MOTOR_1);
        intake2 = hardwareMap.get(DcMotorEx.class, RobotConfig.INTAKE_MOTOR_2);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake2.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);
        intake2.setPower(0);

        // Same rest state as Main TeleOp: flower up, stop gate blocking.
        flowerServo = hardwareMap.get(Servo.class, RobotConfig.FLOWER_SERVO);
        stopServo = hardwareMap.get(Servo.class, RobotConfig.STOP_SERVO);
        flowerServo.setPosition(RobotConfig.FLOWER_UP);
        stopServo.setPosition(RobotConfig.STOP_BLOCKING);

        telemetry.addLine("Test TeleOp initialized.");
        telemetry.addLine("Pad1: dpad-up/down = RPM, RB = flywheel on/off, Y = reset RPM");
        telemetry.addLine("      A = intake forward, B = intake backward, X = stop, LB = stop servo");
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
        double dt = (lastLoopTime < 0) ? 0 : now - lastLoopTime;
        lastLoopTime = now;

        // ---- Drive (and the pad 2 turret buttons) ----
        turret.loop();

        // ---- Flywheel: on/off + adjustable target RPM ----
        if (gamepad1.right_bumper && !prevRightBumper) flywheelOn = !flywheelOn;
        if (gamepad1.dpad_up && !prevUp) {
            targetRpm = Math.min(RobotConfig.SHOOTER_MAX_RPM, targetRpm + RobotConfig.RPM_STEP);
        }
        if (gamepad1.dpad_down && !prevDown) {
            targetRpm = Math.max(0, targetRpm - RobotConfig.RPM_STEP);
        }
        if (gamepad1.y && !prevY) {
            targetRpm = RobotConfig.SHOOTER_TARGET_RPM;
            flywheelOn = false;
        }
        shooter.setTargetRpm(flywheelOn ? targetRpm : 0);
        shooter.update();

        // ---- Intake: FORWARD / BACKWARD / STOP, both motors always the same power ----
        if (gamepad1.a && !prevA) intakeMode = (intakeMode == 1) ? 0 : 1;
        if (gamepad1.b && !prevB) intakeMode = (intakeMode == -1) ? 0 : -1;
        if (gamepad1.x && !prevX) intakeMode = 0;
        double feedPower = (intakeMode == 1) ? RobotConfig.INTAKE_POWER
                : (intakeMode == -1) ? RobotConfig.OUTTAKE_POWER : 0.0;
        intake.setPower(feedPower);
        intake2.setPower(feedPower);

        // ---- Stop servo: open only when asked, so a ball can be watched through the gate ----
        if (gamepad1.left_bumper && !prevLeftBumper) gateOpen = !gateOpen;
        stopServo.setPosition(gateOpen ? RobotConfig.STOP_DOWN : RobotConfig.STOP_BLOCKING);

        prevUp = gamepad1.dpad_up; prevDown = gamepad1.dpad_down; prevY = gamepad1.y;
        prevA = gamepad1.a; prevB = gamepad1.b; prevX = gamepad1.x;
        prevRightBumper = gamepad1.right_bumper; prevLeftBumper = gamepad1.left_bumper;

        // ---- Telemetry ----
        telemetry.addData("Flywheel", "%s  target %.0f rpm (dpad up/down, RB on/off, Y reset)",
                flywheelOn ? "ON" : "OFF", targetRpm);
        telemetry.addData("Shooter RPM", "%.0f / %.0f", shooter.getRpm(), shooter.getTargetRpm());
        telemetry.addData("Shooter Power", "%.3f", shooter.getPower());
        telemetry.addData("At Speed", shooter.atSpeed());
        telemetry.addData("Battery (V)", "%.2f", shooter.getVoltage());
        telemetry.addLine();
        telemetry.addData("Intake mode", "%s (%s %.2f, %s %.2f)  A fwd, B back, X stop",
                intakeMode == 1 ? "FORWARD" : intakeMode == -1 ? "BACKWARD" : "STOPPED",
                RobotConfig.INTAKE_MOTOR_1, intake.getPower(),
                RobotConfig.INTAKE_MOTOR_2, intake2.getPower());
        telemetry.addData("Stop servo", "%s (%.2f, LB toggles)",
                gateOpen ? "OPEN" : "BLOCKING", stopServo.getPosition());
        telemetry.addData("Loop MS", "%.1f", dt * 1000);
        telemetry.update();
    }

    @Override
    public void stop() {
        turret.stop();
        shooter.stop();
        intake.setPower(0);
        intake2.setPower(0);
        stopServo.setPosition(RobotConfig.STOP_BLOCKING);
    }
}
