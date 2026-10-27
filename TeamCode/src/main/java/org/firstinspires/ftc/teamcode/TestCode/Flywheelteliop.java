package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Flywheel (goBILDA 5000 series) run on raw motor power. No velocity control.
 *
 * Controls (gamepad1):
 *   Y             - toggle flywheel on/off
 *   A (hold)      - run intake + transfer
 *   B (hold)      - reverse intake + transfer (unjam)
 *   D-pad Up/Down - flywheel power +/- POWER_STEP
 *
 * Hardware config names: "shooter", "intake", "transfer"
 *
 * Change SHOOTER_POWER live on FTC Dashboard (http://192.168.43.1:8080/dash ->
 * Configuration -> FlywheelTeleOp).
 */
@Config
@TeleOp(name = "Flywheel TeleOp", group = "Competition")
public class Flywheelteliop extends LinearOpMode {

    // Flywheel motor power, 0.0 to 1.0. Editable live on Dashboard.
    public static double SHOOTER_POWER = 0.5;

    // How much each D-pad press changes the power
    public static double POWER_STEP = 0.05;

    public static double INTAKE_POWER = 1.0;
    public static double TRANSFER_POWER = 1.0;

    private DcMotorEx shooter;
    private DcMotor intake;
    private DcMotor transfer;

    private boolean shooterOn = false;
    private boolean lastY = false;
    private boolean lastUp = false;
    private boolean lastDown = false;

    @Override
    public void runOpMode() {
        // Telemetry goes to both the Driver Station and the Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");

        // Flip any of these if a motor spins the wrong way
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);

        // Raw power: no encoder-based speed control
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Ready. Y = toggle flywheel, A = intake/transfer, B = reverse");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Toggle flywheel on Y press (rising edge)
            boolean y = gamepad1.y;
            if (y && !lastY) {
                shooterOn = !shooterOn;
            }
            lastY = y;

            // D-pad Up / Down changes the power (rising edge), kept between 0 and 1
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            if (up && !lastUp) {
                SHOOTER_POWER = Math.min(1.0, SHOOTER_POWER + POWER_STEP);
            }
            if (down && !lastDown) {
                SHOOTER_POWER = Math.max(0.0, SHOOTER_POWER - POWER_STEP);
            }
            lastUp = up;
            lastDown = down;

            // Re-read SHOOTER_POWER every loop so Dashboard edits apply immediately
            shooter.setPower(shooterOn ? Math.max(0.0, Math.min(1.0, SHOOTER_POWER)) : 0);

            // Intake + transfer while button held
            if (gamepad1.a) {
                intake.setPower(INTAKE_POWER);
                transfer.setPower(TRANSFER_POWER);
            } else if (gamepad1.b) {
                intake.setPower(-INTAKE_POWER);
                transfer.setPower(-TRANSFER_POWER);
            } else {
                intake.setPower(0);
                transfer.setPower(0);
            }

            telemetry.addData("Flywheel", shooterOn ? "ON" : "OFF");
            telemetry.addData("Shooter Power", shooterOn ? SHOOTER_POWER : 0);
            telemetry.addData("Shooter Speed (ticks/s)", shooter.getVelocity());
            telemetry.addData("Intake/Transfer", gamepad1.a ? "FORWARD" : (gamepad1.b ? "REVERSE" : "off"));
            telemetry.update();
        }

        shooter.setPower(0);
        intake.setPower(0);
        transfer.setPower(0);
    }
}