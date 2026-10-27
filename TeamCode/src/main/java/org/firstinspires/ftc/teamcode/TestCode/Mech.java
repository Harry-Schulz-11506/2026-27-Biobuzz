package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Mecanum drivebase TeleOp.
 *
 * Hardware config names (must match your Robot Configuration exactly):
 *   FL = front left, FR = front right, BL = back left, BR = back right
 *   imu = the Control Hub's built-in IMU
 *
 * Controls (gamepad1):
 *   Left stick   = drive / strafe
 *   Right stick X = rotate
 *   Right bumper = slow mode (30% speed)
 *   Y            = toggle field-centric / robot-centric
 *   A            = reset heading (field-centric zero)
 */
@TeleOp(name = "Mecanum", group = "Drive")
public class Mech extends LinearOpMode {

    private DcMotor FL, FR, BL, BR;
    private IMU imu;

    private boolean fieldCentric = false;
    private boolean lastY = false;

    @Override
    public void runOpMode() {
        // ---------- Hardware map ----------
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        // Left side reversed so positive power = forward on all wheels.
        // If your robot drives backwards or spins wrong, flip these.
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{FL, FR, BL, BR}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // ---------- IMU (for field-centric) ----------
        // Change these to match how your Control Hub is mounted.
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        telemetry.addLine("Ready. Press START.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Toggle field-centric on Y press
            if (gamepad1.y && !lastY) fieldCentric = !fieldCentric;
            lastY = gamepad1.y;

            if (gamepad1.a) imu.resetYaw();

            double y = -gamepad1.left_stick_y;   // forward
            double x = gamepad1.left_stick_x;    // strafe right
            double rx = gamepad1.right_stick_x;  // rotate clockwise

            if (fieldCentric) {
                double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
                double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
                x = rotX;
                y = rotY;
            }

            // Counteract imperfect strafing
            x *= 1.1;

            // Normalize so no wheel exceeds 1.0 while keeping ratios
            double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double fl = (y + x + rx) / denom;
            double bl = (y - x + rx) / denom;
            double fr = (y - x - rx) / denom;
            double br = (y + x - rx) / denom;

            double speed = gamepad1.right_bumper ? 0.3 : 1.0;

            FL.setPower(fl * speed);
            BL.setPower(bl * speed);
            FR.setPower(fr * speed);
            BR.setPower(br * speed);

            telemetry.addData("Mode", fieldCentric ? "Field-centric" : "Robot-centric");
            telemetry.addData("Speed", gamepad1.right_bumper ? "Slow" : "Full");
            telemetry.addData("FL/FR", "%.2f / %.2f", fl, fr);
            telemetry.addData("BL/BR", "%.2f / %.2f", bl, br);
            telemetry.update();
        }
    }
}