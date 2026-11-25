package org.firstinspires.ftc.teamcode.NATS;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.Iterator;

/**
 * One battery reading, shared by everything that moves a motor.
 *
 * A motor's speed is set by the voltage it actually gets, not by the number you write with setPower:
 * "1.0" at 12.6 V turns the wheels faster than "1.0" at 10.8 V. Voltage compensation scales the power
 * by VOLTAGE_NOMINAL / measured battery voltage, so the same command gives the same speed all match.
 *
 * Usage, exactly one read per loop:
 *   VoltageComp.init(hardwareMap)   once, at init
 *   VoltageComp.update()            every loop (it only touches the sensor every VOLTAGE_READ_SEC)
 *   VoltageComp.scale()             multiply a power by this (1.0 when the battery reads NOMINAL)
 *
 * The sensor is read rarely on purpose: a voltage read is a separate, slow I2C/bulk transaction, and
 * the battery changes far slower than the control loop. The reading is filtered so a spike while a
 * motor starts cannot yank the power around.
 */
public final class VoltageComp {

    private static VoltageSensor sensor = null;
    private static double voltage = RobotConfig.VOLTAGE_NOMINAL;
    private static double lastRead = -1;

    private VoltageComp() {}

    /** Find the battery sensor. Safe to call from every OpMode and again from the auto: the first wins. */
    public static void init(HardwareMap hw) {
        if (sensor != null || hw == null) return;
        // DeviceMapping is Iterable, not a Collection: there is no isEmpty(), so ask the iterator.
        Iterator<VoltageSensor> batteries = hw.voltageSensor.iterator();
        if (batteries.hasNext()) sensor = batteries.next();
        lastRead = -1;
        update();
    }

    /** Read the battery if VOLTAGE_READ_SEC has passed. Call every loop; extra calls are free. */
    public static void update() {
        if (sensor == null) return;
        double now = System.nanoTime() / 1e9;
        if (lastRead >= 0 && now - lastRead < RobotConfig.VOLTAGE_READ_SEC) return;

        double raw = Math.max(sensor.getVoltage(), RobotConfig.VOLTAGE_MIN);
        double filter = Math.min(Math.max(RobotConfig.VOLTAGE_FILTER, 0.0), 1.0);
        voltage = (lastRead < 0) ? raw : filter * voltage + (1.0 - filter) * raw;
        lastRead = now;
    }

    /** The filtered battery voltage in volts. VOLTAGE_NOMINAL when there is no sensor. */
    public static double voltage() {
        return sensor == null ? RobotConfig.VOLTAGE_NOMINAL : voltage;
    }

    /**
     * Power multiplier for the current battery: 1.0 at VOLTAGE_NOMINAL, above 1 when the battery sags,
     * below 1 on a fresh battery. 1.0 always when VOLTAGE_COMP_ENABLED is false.
     */
    public static double scale() {
        if (!RobotConfig.VOLTAGE_COMP_ENABLED) return 1.0;
        double v = Math.max(voltage(), 1e-3);
        return 1.0 + RobotConfig.VOLTAGE_COMP_STRENGTH * (RobotConfig.VOLTAGE_NOMINAL / v - 1.0);
    }

    /** A signed motor power with voltage compensation applied, kept inside -1..1. */
    public static double scaledPower(double power) {
        double scaled = power * scale();
        if (scaled > 1.0) return 1.0;
        if (scaled < -1.0) return -1.0;
        return scaled;
    }
}
