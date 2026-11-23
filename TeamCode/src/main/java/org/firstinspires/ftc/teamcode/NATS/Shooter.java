package org.firstinspires.ftc.teamcode.NATS;

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

    private DcMotorEx motor;
    private VoltageSensor voltageSensor;
    private double voltage = 12.0;
    private double lastVoltageRead = -1;

    private boolean feeding = false;
    private boolean armed = false;       // true once the flywheel has reached its target (so a dip can be told from spin-up)
    private boolean recovering = false;  // true while the dip boost is on
    private double targetRpm = 0;
    private double rpm = 0;
    private double lastRpm = 0;
    private double integral = 0;
    private double lastTime = -1;
    private double inBandSince = -1;
    private double lastPower = 0;

    public void init(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "shooter");
        motor.setDirection(RobotConfig.SHOOTER_MOTOR_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        voltageSensor = hw.voltageSensor.iterator().next();
        voltage = voltageSensor.getVoltage();
    }

    /**
     * Convert PID gains from the FtcAutoTune library (tuned in motor ticks/sec) into flywheel RPM units.
     * 1 flywheel-RPM of error = RobotConfig.SHOOTER_MOTOR_TICKS_PER_REV * (RobotConfig.SHOOTER_FLYWHEEL_TEETH/RobotConfig.SHOOTER_MOTOR_TEETH) / 60 motor ticks/sec,
     * so each gain is multiplied by that factor (about 0.84).
     */
    public static void applyTickGains(double kpTicks, double kiTicks, double kdTicks) {
        double ticksPerSecPerRpm = RobotConfig.SHOOTER_MOTOR_TICKS_PER_REV * (RobotConfig.SHOOTER_FLYWHEEL_TEETH / RobotConfig.SHOOTER_MOTOR_TEETH) / 60.0;
        RobotConfig.SHOOTER_kP = kpTicks * ticksPerSecPerRpm;
        RobotConfig.SHOOTER_kI = kiTicks * ticksPerSecPerRpm;
        RobotConfig.SHOOTER_kD = kdTicks * ticksPerSecPerRpm;
    }

    /** Flywheel RPM from motor ticks/sec through the 10T:18T gear stage. */
    private double rawRpm() {
        double motorRpm = motor.getVelocity() / RobotConfig.SHOOTER_MOTOR_TICKS_PER_REV * 60.0;
        double rpm = motorRpm * (RobotConfig.SHOOTER_MOTOR_TEETH / RobotConfig.SHOOTER_FLYWHEEL_TEETH);
        return RobotConfig.SHOOTER_ENCODER_REVERSED ? -rpm : rpm;
    }

    private void readVoltage(double now) {
        if (lastVoltageRead < 0 || now - lastVoltageRead > RobotConfig.SHOOTER_VOLTAGE_READ_SEC) {
            double v = Math.max(voltageSensor.getVoltage(), RobotConfig.SHOOTER_MIN_VOLTAGE);
            voltage = (lastVoltageRead < 0) ? v : RobotConfig.SHOOTER_VOLTAGE_FILTER * voltage + (1 - RobotConfig.SHOOTER_VOLTAGE_FILTER) * v;
            lastVoltageRead = now;
        }
    }

    public void setTargetRpm(double rpm) {
        if (rpm <= 0) integral = 0;
        if (Math.abs(rpm - targetRpm) > 150) { armed = false; recovering = false; } // a real target change, not a nudge
        targetRpm = Math.max(0, rpm);
    }

    /** Tell the shooter an pollen is being fed, so it fights the RPM drop. Call every loop before update(). */
    public void setFeeding(boolean feeding) { this.feeding = feeding; }

    public boolean isRecovering() { return recovering; }

    public double getTargetRpm() { return targetRpm; }
    public double getRpm() { return rpm; }
    public double getPower() { return lastPower; }
    public double getVoltage() { return voltage; }

    public boolean atSpeed() {
        return targetRpm > 0 && inBandSince >= 0
                && (System.nanoTime() / 1e9 - inBandSince) >= RobotConfig.SHOOTER_AT_SPEED_HOLD_SEC;
    }

    /** Measure velocity only (no control). Used by autotune. Call every loop. */
    public void updateMeasurement() {
        double now = System.nanoTime() / 1e9;
        readVoltage(now);
        rpm = RobotConfig.SHOOTER_VELOCITY_FILTER * rpm + (1 - RobotConfig.SHOOTER_VELOCITY_FILTER) * rawRpm();
    }

    /** Run the velocity controller once. Call every loop. */
    public void update() {
        updateMeasurement();
        double now = System.nanoTime() / 1e9;
        double dt = (lastTime < 0) ? 0 : now - lastTime;
        lastTime = now;

        if (targetRpm <= 0) {
            feeding = false;
            armed = false;
            recovering = false;
            lastPower = 0;
            motor.setPower(0);
            inBandSince = -1;
            lastRpm = rpm;
            return;
        }

        double error = targetRpm - rpm;

        double derivative = 0;
        // Dip detection: armed once at the target; the first drop of RobotConfig.SHOOTER_DIP_THRESHOLD_RPM or more starts the boost.
        if (Math.abs(error) <= RobotConfig.SHOOTER_AT_SPEED_TOLERANCE_RPM) armed = true;
        if (!recovering && armed && error >= RobotConfig.SHOOTER_DIP_THRESHOLD_RPM) recovering = true;
        if (recovering && error <= RobotConfig.SHOOTER_RECOVERY_END_RPM) recovering = false;

        if (dt > 1e-4 && !recovering) { // the integral is frozen during the boost so it cannot wind up
            integral += error * dt;
            derivative = (rpm - lastRpm) / dt;
        }
        lastRpm = rpm;

        if (RobotConfig.SHOOTER_kI > 1e-12) {
            double limit = RobotConfig.SHOOTER_I_LIMIT_POWER / RobotConfig.SHOOTER_kI;
            integral = Range.clip(integral, -limit, limit);
        } else {
            integral = 0;
        }

        double voltageScale = RobotConfig.SHOOTER_VOLTAGE_COMP_ENABLED
                ? 1.0 + RobotConfig.SHOOTER_VOLTAGE_COMP_STRENGTH * (RobotConfig.SHOOTER_NOMINAL_VOLTAGE / voltage - 1.0)
                : 1.0;
        double feedforward = (RobotConfig.SHOOTER_kS + RobotConfig.SHOOTER_kV * targetRpm) * voltageScale;
        double p = feeding ? RobotConfig.SHOOTER_kP * RobotConfig.SHOOTER_FEED_KP_MULT : RobotConfig.SHOOTER_kP;
        double boost = feeding ? RobotConfig.SHOOTER_FEED_BOOST_POWER : 0.0;
        if (recovering) boost += RobotConfig.SHOOTER_RECOVERY_BOOST_POWER;
        double out = feedforward + boost + p * error + RobotConfig.SHOOTER_kI * integral - RobotConfig.SHOOTER_kD * derivative;
        out = Range.clip(out, 0, 1);

        lastPower = out;
        motor.setPower(out);

        if (Math.abs(error) <= RobotConfig.SHOOTER_AT_SPEED_TOLERANCE_RPM) {
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

    /** Signed open-loop power for the relay auto-tuner: negative values reverse the flywheel. */
    public void setOpenLoopPowerSigned(double power) {
        lastPower = Range.clip(power, -1, 1);
        motor.setPower(lastPower);
    }

    public void stop() {
        targetRpm = 0;
        lastPower = 0;
        motor.setPower(0);
    }
}