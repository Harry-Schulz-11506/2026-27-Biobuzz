package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Shooter Test", group = "Test")
public class Shootertest extends OpMode {

    private DcMotor intake;
    private DcMotor transfer;
    private DcMotor shooter;

    private static final double SPEED_STEP = 0.05;
    private static final double INTAKE_POWER = 1;
    private static final double TRANSFER_POWER = 1;

    private double shooterPower = 0.5;
    private boolean shooterOn = false;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastA = false;

    @Override
    public void init() {
        // Names must match your Robot Configuration
        intake   = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        shooter  = hardwareMap.get(DcMotor.class, "shooter");

        // All motors reversed
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Shooter Test ready");
        telemetry.addLine("X (hold): intake + transfer");
        telemetry.addLine("A: toggle shooter on/off");
        telemetry.addLine("D-pad Up/Down: shooter speed +/- " + SPEED_STEP);
        telemetry.update();
    }

    @Override
    public void loop() {
        // Intake + transfer run only while X is held
        if (gamepad1.x) {
            intake.setPower(INTAKE_POWER);
            transfer.setPower(TRANSFER_POWER);
        } else {
            intake.setPower(0);
            transfer.setPower(0);
        }

        // Shooter speed adjust (edge detected so one press = one step)
        if (gamepad1.dpad_up && !lastDpadUp) {
            shooterPower += SPEED_STEP;
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            shooterPower -= SPEED_STEP;
        }
        shooterPower = Range.clip(shooterPower, 0.0, 1.0);

        // Shooter on/off toggle
        if (gamepad1.a && !lastA) {
            shooterOn = !shooterOn;
        }

        shooter.setPower(shooterOn ? shooterPower : 0);

        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastA = gamepad1.a;

        telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
        telemetry.addData("Shooter power", "%.2f", shooterPower);
        telemetry.addData("Intake/Transfer", gamepad1.x ? "RUNNING" : "stopped");
        telemetry.update();
    }

    @Override
    public void stop() {
        intake.setPower(0);
        transfer.setPower(0);
        shooter.setPower(0);
    }
}