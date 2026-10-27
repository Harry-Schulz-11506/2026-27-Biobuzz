package org.firstinspires.ftc.teamcode.VibeCodeing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Flywheel: bare goBILDA 5000 series motor (28 PPR at the motor shaft, 6000 RPM free speed).
 * 10T motor gear drives 18T flywheel gear (1.8:1 reduction -> ~3333 RPM max at the flywheel).
 * Velocity PIDF in FLYWHEEL RPM, with voltage-compensated feedforward.
 * Hardware name: "shooter".
 */
@Config
public class Shooter {

    // ---- Velocity PIDF (tune in Dashboard, or run the ShooterAutoTune OpMode) ----
    public static double kP = 0.000564;   // power per RPM of error
    public static double kI = 0.000368;
    public static double kD = 0.0;
    public static double kV = 0.000344;   // power per RPM  (~1 / 3333 at 12 V)
    public static double kS = 0.12706;      // static offset power while target > 0
    public static double I_LIMIT_POWER = 0.25;
    public static double NOMINAL_VOLTAGE = 12.0;

    // ---- Recovery boost (extra power when a ball drags the RPM down) ----
    // When RPM falls more than RECOVERY_DEADBAND_RPM below target, add RECOVERY_KP power per RPM of the
    // extra drop, plus RECOVERY_KICK_PER_RPM_PER_SEC for how fast it is falling. Capped at RECOVERY_MAX_BOOST.
    // Turns itself off once back inside the deadband. Set RECOVERY_ENABLED = false to compare.
    public static boolean RECOVERY_ENABLED = true;
    public static double RECOVERY_DEADBAND_RPM = 40;
    public static double RECOVERY_KP = 0.0006;              // extra power per RPM below the deadband
    public static double RECOVERY_KICK_PER_RPM_PER_SEC = 0.00004; // extra power per RPM/s of falling speed
    public static double RECOVERY_MAX_BOOST = 0.35;         // max extra power (total still capped at 1.0)

    // ---- Geometry ----
    public static double MOTOR_TICKS_PER_REV = 28;
    public static double MOTOR_TEETH = 10;
    public static double FLYWHEEL_TEETH = 10;

    // ---- Behaviour ----
    public static double TARGET_RPM = 2000;      // used by the TeleOp when the shooter is switched on
    public static double AT_SPEED_TOLERANCE_RPM = 50;
    public static double AT_SPEED_HOLD_SEC = 0.15;
    public static double VELOCITY_FILTER = 0.6;  // 0 = no filtering, closer to 1 = heavier filtering
    public static boolean REVERSED = true;       // read at init(); flip if the flywheel spins the wrong way

    private DcMotorEx motor;
    private VoltageSensor voltageSensor;
    private double voltage = 12.0;
    private double lastVoltageRead = -1;

    private double targetRpm = 0;
    private double rpm = 0;
    private double lastRpm = 0;
    private double integral = 0;
    private double lastTime = -1;
    private double inBandSince = -1;
    private double lastPower = 0;
    private double boost = 0;

    public void init(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "shooter");
        motor.setDirection(REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        voltageSensor = hw.voltageSensor.iterator().next();
        voltage = voltageSensor.getVoltage();
    }

    /**
     * Convert PID gains from the FtcAutoTune library (tuned in motor ticks/sec) into flywheel RPM units.
     * 1 flywheel-RPM of error = MOTOR_TICKS_PER_REV * (FLYWHEEL_TEETH/MOTOR_TEETH) / 60 motor ticks/sec,
     * so each gain is multiplied by that factor (about 0.84).
     */
    public static void applyTickGains(double kpTicks, double kiTicks, double kdTicks) {
        double ticksPerSecPerRpm = MOTOR_TICKS_PER_REV * (FLYWHEEL_TEETH / MOTOR_TEETH) / 60.0;
        kP = kpTicks * ticksPerSecPerRpm;
        kI = kiTicks * ticksPerSecPerRpm;
        kD = kdTicks * ticksPerSecPerRpm;
    }

    /** Flywheel RPM from motor ticks/sec through the 10T:18T gear stage. */
    private double rawRpm() {
        double motorRpm = motor.getVelocity() / MOTOR_TICKS_PER_REV * 60.0;
        return motorRpm * (MOTOR_TEETH / FLYWHEEL_TEETH);
    }

    private void readVoltage(double now) {
        if (lastVoltageRead < 0 || now - lastVoltageRead > 0.25) {
            voltage = Math.max(voltageSensor.getVoltage(), 7.0);
            lastVoltageRead = now;
        }
    }

    public void setTargetRpm(double rpm) {
        if (rpm <= 0) integral = 0;
        targetRpm = Math.max(0, rpm);
    }

    public double getTargetRpm() { return targetRpm; }
    public double getRpm() { return rpm; }
    public double getPower() { return lastPower; }
    public double getVoltage() { return voltage; }
    /** Extra recovery power currently being added (0 when not recovering). */
    public double getBoost() { return boost; }

    public boolean atSpeed() {
        return targetRpm > 0 && inBandSince >= 0
                && (System.nanoTime() / 1e9 - inBandSince) >= AT_SPEED_HOLD_SEC;
    }

    /** Measure velocity only (no control). Used by autotune. Call every loop. */
    public void updateMeasurement() {
        double now = System.nanoTime() / 1e9;
        readVoltage(now);
        rpm = VELOCITY_FILTER * rpm + (1 - VELOCITY_FILTER) * rawRpm();
    }

    /** Run the velocity controller once. Call every loop. */
    public void update() {
        updateMeasurement();
        double now = System.nanoTime() / 1e9;
        double dt = (lastTime < 0) ? 0 : now - lastTime;
        lastTime = now;

        if (targetRpm <= 0) {
            lastPower = 0;
            motor.setPower(0);
            inBandSince = -1;
            boost = 0;
            lastRpm = rpm;
            return;
        }

        double error = targetRpm - rpm;

        double derivative = 0;
        double fallRate = 0; // RPM per second, positive while the flywheel is slowing down
        if (dt > 1e-4) {
            integral += error * dt;
            derivative = (rpm - lastRpm) / dt;
            fallRate = Math.max(0, -derivative);
        }
        lastRpm = rpm;

        if (kI > 1e-12) {
            double limit = I_LIMIT_POWER / kI;
            integral = Range.clip(integral, -limit, limit);
        } else {
            integral = 0;
        }

        double feedforward = (kS + kV * targetRpm) * (NOMINAL_VOLTAGE / voltage);
        boost = 0;
        if (RECOVERY_ENABLED && error > RECOVERY_DEADBAND_RPM) {
            boost = RECOVERY_KP * (error - RECOVERY_DEADBAND_RPM) + RECOVERY_KICK_PER_RPM_PER_SEC * fallRate;
            boost = Range.clip(boost, 0, RECOVERY_MAX_BOOST);
        }
        double out = feedforward + kP * error + kI * integral - kD * derivative + boost;
        out = Range.clip(out, 0, 1);

        lastPower = out;
        motor.setPower(out);

        if (Math.abs(error) <= AT_SPEED_TOLERANCE_RPM) {
            if (inBandSince < 0) inBandSince = now;
        } else {
            inBandSince = -1;
        }
    }

    /** Bypasses the controller (used by autotune). */
    public void setOpenLoopPower(double power) {
        lastPower = Range.clip(power, 0, 1);
        motor.setPower(lastPower);
    }

    public void stop() {
        targetRpm = 0;
        lastPower = 0;
        motor.setPower(0);
    }
}